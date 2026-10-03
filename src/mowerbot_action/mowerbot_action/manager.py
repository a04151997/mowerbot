#!/usr/bin/env python3
import math
import tf2_ros
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from mowerbot_interfaces.srv import GenerateCoveragePath
from mowerbot_interfaces.msg import ObstaclePolygons, MowerStatus, MissionStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_msgs.action import FollowPath
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from mowerbot_interfaces.srv import SetDriveMode

# ---- 跨檔共用的預設值：單一來源 (階段 38 修正 3 原則 7) ----
# 以前 launch 檔、HMI、smoke_test 各寫一份並註明「要一致」；現在它們都 import 這裡。
# 割草線重疊率預設值 (選定理由見 __init__ 裡 overlap_ratio 的註解)
DEFAULT_OVERLAP_RATIO = 0.4
# 邊界最小面積預設值 (理由見 __init__ 裡 min_boundary_area 的註解)
DEFAULT_MIN_BOUNDARY_AREA = 4.0
# 航點間距：割草線、周邊環繞 (f2c_server 由 launch 傳入)、approach、跑道共用
WAYPOINT_SPACING = 0.1


def perimeter_corner_cuts(xy, window, angle):
    """周邊環繞的轉角偵測 (階段 38 決定 3)，回傳要切開的航點索引 (遞增)。

    為什麼不用割草線的「單步轉角 >= 90°」規則：SLAM 地圖萃取的邊界不是完美直角，
    真正的轉角是 85° ~ 89.5°，有的還分散在好幾步 (79° − 46°、40° − 39° − 86° …)，
    單步規則一刀都切不開 (stage29_lawn、headland 1.20：整圈一段、起點 = 終點)。

    做法：沿路徑累積「有號」轉角；從每個頂點往前看一個 window 的弧長，
    累積轉角絕對值 >= angle 就是一個轉角區，切在區內累積轉角過半的那個頂點，
    然後跳過整個區 (同一個轉角只切一刀)。直線上的雜訊轉角正負抵消、累積不起來。
    """
    n = len(xy)
    if n < 3:
        return []
    head, pos, s = [], [], 0.0
    for i in range(n - 1):
        dx, dy = xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1]
        head.append(math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else None)
        pos.append(s)
        s += math.hypot(dx, dy)
    turn = [0.0] * n          # turn[i] = 在頂點 i 的轉角 (步 i-1 -> 步 i)
    last = None
    for i in range(n - 1):
        if head[i] is None:
            continue
        if last is not None:
            turn[i] = math.atan2(math.sin(head[i] - last), math.cos(head[i] - last))
        last = head[i]
    cuts, j = [], 1
    while j < n - 1:
        acc, k, best = 0.0, j, None
        while k < n - 1 and pos[k] - pos[j] < window:
            acc += turn[k]
            k += 1
        if abs(acc) >= angle:
            half, run = acc / 2.0, 0.0
            for m in range(j, k):
                run += turn[m]
                if abs(run) >= abs(half):
                    best = m
                    break
            cuts.append(best)
            j = k
        else:
            j += 1
    return cuts

class MowerManager(Node):
    def __init__(self):
        super().__init__('mower_manager')
        # 預設為手動模式
        self.nav_client = ActionClient(self, FollowPath, 'follow_path')
        # 發布者：將 F2C 算出的全局路徑發布給 RViz2 顯示
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/f2c_path', qos_profile)
        self.current_mode = 2
        # 保存 Nav2 FollowPath 的 goal handle，急停時用來取消任務
        self._goal_handle = None
        # 覆蓋任務是一條一條割草線循序執行的：
        # 整條覆蓋路徑會先被切成多條割草線放進佇列，每次只送一條給 Nav2，
        # 等它回報 SUCCEEDED 再送下一條。
        # 佇列存的是 (nav_msgs/Path, label)，label 是 'approach' 或 '割草線 N/M'，
        # 讓結果回呼可以分辨剛跑完的是移動任務還是割草作業。
        self._swath_queue = []
        self._current_swath_idx = 0     # 目前執行到第幾個任務 (0-based)
        self._swath_total = 0           # 割草線總數 (不含 approach)
        # 失敗處理的狀態 (詳見 follow_path_result_callback 的說明)
        self._consecutive_failures = 0  # 連續失敗次數，任一段成功就歸零
        self._skipped = []              # [(label, (x, y) 或 None, 失敗原因)]
        self._succeeded = 0             # 成功完成的段數 (含 approach / 環繞)
        # 【安全關鍵】「這次的結束是我們主動取消的」旗標。
        # 急停與模式切換會呼叫 cancel_current_goal()，結果回呼必須能分辨
        # 「控制器自己放棄 (ABORTED)」與「我們喊停 (CANCELED)」——
        # 前者跳過繼續，後者一定要整個停掉。
        self._cancelling = False
        # 對外發布的任務狀態 (階段 18)。這些數字 manager 本來就有，
        # 只是以前沒有發出去，GUI 因此無從得知任務跑到哪裡。
        self._mission_state = MissionStatus.STATE_IDLE
        self._mission_total = 0        # 任務開始時的佇列總段數 (任務結束後仍保留，給畫面顯示)
        self._lead_in_hist = {}        # 跑道長度 -> 幾條割草線用了這個長度
        self._current_label = ''
        self._mission_message = ''     # 任務層級的說明訊息，給 HMI 顯示 (階段 23)
        # 對正階段 (階段 38 決定 1)：True = 目前在跑的 goal 是對正 (AlignController)；
        # _aligned_idx = 已經對正完成的佇列索引 (對正成功之後才送那一段本身)
        self._align_phase = False
        self._aligned_idx = None
        # approach 迴圈的終止：記住「現在在接近哪一段」與歷次量到的距離
        self._approach_for = None      # 目前在為哪一個段落插 approach
        self._approach_dists = []      # 每一輪量到的「離目標多遠」
        self._approach_tries = 0       # 已經為這個目標插了幾段 approach
        self._result_future = None

        # TF：manager 本來完全不知道車子在哪裡，但要算出「從當下位置前往第 1 條
        # 割草線起點」的 approach 路徑就必須知道。F2C 路徑的 frame_id 是 "map"，
        # 所以這裡查 map -> base_footprint，保持在同一個座標系。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 跑道 (lead-in) 長度，單位公尺。
        # 每條割草線送出去之前，會先在它的起點「往後」沿著同一個方向延伸這段距離，
        # 讓車子掉完頭有一段共線的距離可以把橫向誤差收斂掉，
        # 真正進入割草段時已經貼在線上。這是農機的 headland(地頭)概念。
        # 設 0 代表停用跑道。
        #
        # 預設 0.5 m 是實測掃描 L = 0.5 / 1.0 / 1.5 之後選的 (數據見
        # docs/simulation_results.md)。三個值的覆蓋落差在誤差內分不出高下
        # (最大 0.635~0.672 m，超標比例 14.1~16.5%)，都沒有達到 < 0.25 m 的目標；
        # 真正有差別的是「有沒有跑道」：L = 0 時第 3 條割草線會因為
        # "Controller patience exceeded" 直接 ABORTED，車子無法在原地掉頭接上
        # 一條剛好起點就在旁邊的路徑。所以跑道是可靠度功能而不是覆蓋品質功能。
        # 既然覆蓋品質分不出高下，就取「能用的最小值」：L 同時是階段 2 的地頭寬度，
        # 而地頭會直接吃掉可作業面積 (5x5 m 的地上 L=1.5 只剩 2 m 寬可割)。
        # 邊界的最小面積。小於這個值的邊界一律拒絕，不會更新 latest_boundary。
        #
        # 為什麼需要：map_to_boundary 在建圖還沒完成時輸出很不穩定
        # (報告 11.4 節：同一次執行依序報出 0 -> 17 -> 19 -> 20 -> 4 -> 1 個障礙物，
        # 邊界面積也一樣會從幾 m² 慢慢長到正常值)。使用者若在那個瞬間切自動割草，
        # 任務就會拿著垃圾邊界啟動 —— 實測收過 2.93 m² 的邊界，
        # 正常值是 25 m²。
        #
        # 預設 4.0 m² 的理由 (階段 33 改寫；舊的說法是 headland 0.5 時「2m x 2m 扣掉地頭剩零」，已過期)：
        # * 幾何下限：地頭 (headland_width，現在 0.70 m) 從四周往內縮，正方形邊長要超過
        #   2 x 0.70 = 1.40 m、面積 1.96 m² 才有作業區；低於這個值 F2C 一定規劃不出割草線。
        # * 4.0 m² 是刻意放在幾何下限之上的實用下限：2m x 2m 扣掉地頭只剩 0.6m x 0.6m，
        #   約 2 條 0.6 m 長的割草線，已經沒有割草的意義；而實測收到的垃圾邊界
        #   (2.75 / 2.93 / 3.03 / 3.10 m²，報告 12.3、14.3) 全部落在 1.96 ~ 4.0 之間。
        #   真實草坪的邊界約 115 m²，遠在門檻之上。
        # 所以不要把它「修正」成 1.96 —— 那會讓這些垃圾邊界通過。
        self.declare_parameter('min_boundary_area', DEFAULT_MIN_BOUNDARY_AREA)
        self.min_boundary_area = float(
            self.get_parameter('min_boundary_area').value)

        # 連續失敗幾次就判定為系統性問題、停止整個任務。
        # 單獨一段失敗多半是臨時障礙物 (椅子、樹枝、有人站在草坪上)，
        # 那種情況跳過那一段繼續割其餘的才合理；但如果連續好幾段都失敗，
        # 通常是定位跑掉或 Nav2 掛了，繼續送下去只是讓車子在場上亂撞。
        self.declare_parameter('max_consecutive_failures', 3)
        self.max_consecutive_failures = int(
            self.get_parameter('max_consecutive_failures').value)

        self.declare_parameter('lead_in_length', 0.5)
        self.lead_in_length = float(
            self.get_parameter('lead_in_length').value)
        self.get_logger().info(
            f'🛬 跑道 (lead-in) 長度 = {self.lead_in_length:.2f} m')

        # 割草線重疊：刀盤寬 0.5 m 時，如果割草線間距也取 0.5 m 就是零重疊設計，
        # 任何循跡誤差都會直接在兩條線之間留下沒割到的帶狀區。
        # 真實農機的標準作法是把間距縮小一點，用重疊去吸收循跡誤差。
        #   割草線間距 = 刀盤寬 x (1 - 重疊率)
        # 注意：覆蓋落差的判定基準仍然是「實際刀盤寬的一半」= blade_width / 2，
        # 不會因為間距變小就跟著變鬆。
        # 預設 0.4 是實測掃描 0 / 0.1 / 0.2 / 0.3 / 0.4 之後選的 (每個值各跑 3 次，
        # 數據見 docs/simulation_results.md 4.6 節)。0.4 在「超標航點比例」與
        # 「實際未割面積比例」兩個指標上都是最低的 (7.82% / 10.70%)，
        # 代價是任務耗時比零重疊多 45% (107.9 s -> 156.8 s)。
        # 注意曲線到 0.4 還沒有平掉，0.5 可能更好，只是沒有測。
        # 刀盤寬來自 vehicle.yaml (由 mower_control.launch.py 傳入)，這裡不給預設值：
        # 沒傳就在 get_parameter 時直接失敗，不要安靜地用一個寫死的數字。
        self.declare_parameter('blade_width', Parameter.Type.DOUBLE)
        self.declare_parameter('overlap_ratio', DEFAULT_OVERLAP_RATIO)
        self.blade_width = float(self.get_parameter('blade_width').value)
        self.overlap_ratio = float(self.get_parameter('overlap_ratio').value)
        self.swath_spacing = self.blade_width * (1.0 - self.overlap_ratio)
        self.get_logger().info(
            f'🔪 實際刀盤寬 = {self.blade_width:.3f} m，'
            f'重疊率 = {self.overlap_ratio:.2f}，'
            f'割草線間距 = {self.swath_spacing:.3f} m')
        # 參數之間的矛盾檢查 (階段 38 後續 6)：APPROACH_SKIP_DISTANCE >= 割草線間距時，相鄰割草線之間
        # 永遠不會插入 approach —— 車子在上一條線的終點原地掉頭，帶著一個間距的側向差開始下一條
        # (simulation_results.md 38.17)。先只警告不 abort：現有設定 (0.5 vs 0.30) 就是這個情況，abort 會起不來。
        if self.APPROACH_SKIP_DISTANCE >= self.swath_spacing:
            self.get_logger().warn(
                f'⚠️ 參數矛盾：APPROACH_SKIP_DISTANCE = {self.APPROACH_SKIP_DISTANCE:.2f} m >= '
                f'割草線間距 {self.swath_spacing:.3f} m —— 相鄰割草線之間永遠不會插入 approach，'
                f'每一條割草線都會帶著約 {self.swath_spacing:.2f} m 的側向差起步')

        # 淨空門檻：來自 mowerbot_description/vehicle_geometry.py (由 mower_control.launch.py
        # 傳入)，不給預設值，理由同 blade_width。manager 不自己算幾何 (階段 38)。
        # 說明見 lead_in_point_unsafe() 上方。
        self.declare_parameter('lateral_half_extent', Parameter.Type.DOUBLE)
        self.declare_parameter('rotation_swept_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('soft_inflation_radius', Parameter.Type.DOUBLE)
        self.LEAD_IN_BOUNDARY_CLEARANCE = float(
            self.get_parameter('lateral_half_extent').value)
        self.ROTATION_CLEARANCE = float(
            self.get_parameter('rotation_swept_radius').value)
        self.LEAD_IN_OBSTACLE_CLEARANCE = float(
            self.get_parameter('soft_inflation_radius').value)
        # 周邊環繞的轉角偵測與「起點 = 終點」判定 (階段 38 決定 3)，沒有預設值：
        #   perimeter_corner_window / perimeter_corner_angle  來自 vehicle_geometry
        #   goal_xy_tolerance  nav2 general_goal_checker 的 xy_goal_tolerance (起點離終點不超過它的段落，
        #                      goal checker 會在送出當下判定到達 —— 一公尺都不會開)
        self.declare_parameter('perimeter_corner_window', Parameter.Type.DOUBLE)
        self.declare_parameter('perimeter_corner_angle', Parameter.Type.DOUBLE)
        self.declare_parameter('goal_xy_tolerance', Parameter.Type.DOUBLE)
        self.perimeter_corner_window = float(self.get_parameter('perimeter_corner_window').value)
        self.perimeter_corner_angle = float(self.get_parameter('perimeter_corner_angle').value)
        self.goal_xy_tolerance = float(self.get_parameter('goal_xy_tolerance').value)
        self.get_logger().info(
            f'🚙 直線通過門檻 (lateral_half_extent) = {self.LEAD_IN_BOUNDARY_CLEARANCE!r} m，'
            f'掉頭門檻 (rotation_swept_radius) = {self.ROTATION_CLEARANCE!r} m，'
            f'跑道離障礙物門檻 (soft_inflation_radius) = {self.LEAD_IN_OBSTACLE_CLEARANCE!r} m')
        # mode 3 是**保留值，沒有實作**(階段 23)。
        #
        # 【為什麼不重新編號】
        # mode 4 必須維持是急停。把 3 拿掉再往前挪會動到急停的編號，
        # 那是安全介面，不能為了讓表格好看而改。所以 3 留著當空號。
        #
        # 【為什麼仲裁行為要保留】
        # 介面上已經沒有任何按鈕可以切到 3，但服務 change_mower_mode 仍然
        # 接受它。萬一有東西(手把、外部腳本、之後的程式碼)把模式設成 3，
        # 行為必須是定義好而且安全的 —— 所以 nav_vel_cb 的轉發、joy_vel_cb
        # 的擋手把、急停與離開時的清空佇列，全部維持原狀，測試也全部保留。
        #
        # 【要做出來需要什麼】
        # 點對點導航要 planner_server(ComputePathToPose) 與 bt_navigator，
        # 兩者都沒有啟動；本專題只跑 controller_server。列為未來工作。
        self.mode_map = {
            0: '建圖模式(SLAM)',
            1: '自動割草(F2C規劃+執行)',
            2: '手動模式',
            3: '保留(未實作)',
            4: '緊急停止(E-STOP)'
        }

        # 安全計時器 如果0.5秒內沒收到任何速度指令則停止馬達
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timeout = 0.5
        # 發布者：發給真正的底盤驅動
        self.real_vel_pub = self.create_publisher(Twist, '/cmd_vel' ,10)

        # 訂閱者：接收來自不同來源的速度指令
        self.joy_sub = self.create_subscription(Twist ,'/cmd_vel_joy', self.joy_vel_cb,10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_vel_cb,10)
        
        self.srv = self.create_service(SetDriveMode,'change_mower_mode',self.change_mode_callback)

        self.timer = self.create_timer(0.05,self.safety_check)
        # 訂閱最新算出的綠色邊界
        self.latest_boundary = None
        self.latest_boundary_area = 0.0
        self._boundary_seen = 0        # 總共收到幾個邊界
        self._boundary_rejected = 0    # 其中幾個沒通過檢查
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/f2c_boundary', self.boundary_cb, 10)

        # 訂閱作業區內部的障礙物輪廓 (階段 11)。
        # 沒有收到訊息時維持空 list，送給 F2C 的 obstacles 就是空的，
        # 行為與階段 10 完全相同。
        self.latest_obstacles = []
        # 訂閱 local costmap，只為了在任務失敗時能報出「那個點的 cost 是多少」。
        # 不拿它做任何判斷 —— 判斷是 controller_server 的事，
        # 這裡只是把證據記下來，免得每次都要重跑一遍才知道發生什麼事。
        # Nav2 的 costmap publisher 是 transient local 的，QoS 要對得上才收得到。
        costmap_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.local_costmap = None
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap',
            self.costmap_cb, costmap_qos)

        self.obstacles_sub = self.create_subscription(
            ObstaclePolygons, '/f2c_obstacles', self.obstacles_cb, 10)

        # 狀態發布 (階段 18)：GUI 與任何外部工具都靠這兩支知道系統在做什麼。
        # 5 Hz 足夠給人看，又不會塞爆 log 與網路。
        self.mower_status_pub = self.create_publisher(MowerStatus, 'mower_status', 10)
        self.mission_status_pub = self.create_publisher(MissionStatus, 'mission_status', 10)
        self.status_timer = self.create_timer(0.2, self.publish_status)

        # 建立呼叫 C++ F2C 伺服器的 Client
        self.f2c_client = self.create_client(GenerateCoveragePath, 'generate_coverage_path')
        self.get_logger().info('Mower Manager 啟動成功,目前模式：【手動模式】')

    # 速度指令處理
    """
    接收手把速度：僅在手動(2)或建圖(0)模式下轉發
    """
    def joy_vel_cb(self,msg):
        if self.current_mode == 0 or self.current_mode == 2:
            self.publish_and_update(msg)
        # 修正拼字錯誤：cureent_mode -> current_mode
        elif self.current_mode == 4:
            self.handle_estop_violation('手把 (Teleop)')

    """ 
    接收導航速度：僅在 F2C(1) 或保留的 mode 3 下轉發 
    """
    def nav_vel_cb(self,msg):
        if self.current_mode == 1 or self.current_mode == 3:
            self.publish_and_update(msg)
        elif self.current_mode == 4:
            self.handle_estop_violation('導航系統 (Nav2)')

    def publish_and_update(self,msg):
        self.real_vel_pub.publish(msg)
        self.last_cmd_time = self.get_clock().now()
        
    def handle_estop_violation(self, source_name):
        """
        處理急停狀態下的違規指令。
        確保即使收到指令，底盤依然保持靜止，並使用 throttle_duration 限制日誌刷屏。
        """
        self.stop_robot()
        self.get_logger().warn(
            f'急停鎖定中！攔截到來自 {source_name} 的異常移動指令。', 
            throttle_duration_sec=2.0  # 每 2 秒最多印出一次，避免日誌崩潰
        )
    @staticmethod
    def polygon_area(points):
        """shoelace 公式算多邊形面積。

        不用 bounding box：L 形或凹形的草坪，bounding box 會高估面積，
        那樣「看起來夠大、實際上割不了」的邊界就會通過檢查。
        """
        n = len(points)
        if n < 3:
            return 0.0
        acc = 0.0
        for i in range(n):
            a = points[i]
            b = points[(i + 1) % n]
            acc += a.x * b.y - b.x * a.y
        return abs(acc) * 0.5

    def boundary_cb(self, msg):
        """收到邊界時先做合理性檢查，通過才更新 latest_boundary。

        以前是無條件接受。問題在於 /f2c_boundary 隨時可能收到
        建圖還沒完成時算出來的小邊界 (報告 11.4 與 12.3 節)，
        而 manager 用的是「最後收到的那一個」——
        使用者剛好在那個瞬間切自動割草，任務就拿著垃圾邊界啟動了。

        不通過的邊界會被丟掉，**保留上一個有效的邊界**，
        並且把拒絕原因與實際數值印出來 (不要安靜地忽略，
        那會變成「為什麼沒反應」這種最難查的問題)。
        """
        self._boundary_seen += 1
        pts = msg.polygon.points
        area = self.polygon_area(pts)

        reason = None
        if len(pts) < 4:
            reason = '頂點數 %d < 4' % len(pts)
        elif area < self.min_boundary_area:
            reason = ('面積 %.2f m² < 門檻 %.2f m²'
                      % (area, self.min_boundary_area))

        if reason is not None:
            self._boundary_rejected += 1
            if self.latest_boundary is not None:
                keep = ('保留上一個有效邊界（面積 %.1f m²，頂點 %d 個）'
                        % (self.latest_boundary_area,
                           len(self.latest_boundary.points)))
            else:
                keep = '目前還沒有任何有效邊界'
            self.get_logger().warn(
                f'⚠️ 拒絕邊界：{reason}，{keep}',
                throttle_duration_sec=5.0)
            return

        self.latest_boundary = msg.polygon
        self.latest_boundary_area = area

    def costmap_cb(self, msg):
        self.local_costmap = msg

    def costmap_cost_at(self, x, y):
        """回傳 (x, y) 在 local costmap 上的值，拿不到就回傳 None。

        Nav2 發布的 OccupancyGrid 是換算過的：0~100，其中 99 對應 253
        (INSCRIBED，車體一定碰到)、100 對應 254 (LETHAL)、-1 是未知。
        """
        cm = self.local_costmap
        if cm is None:
            return None
        res = cm.info.resolution
        if res <= 0.0:
            return None
        col = int((x - cm.info.origin.position.x) / res)
        row = int((y - cm.info.origin.position.y) / res)
        if not (0 <= col < cm.info.width and 0 <= row < cm.info.height):
            return None
        return int(cm.data[row * cm.info.width + col])

    @staticmethod
    def describe_cost(cost):
        if cost is None:
            return '(不在 local costmap 範圍內或還沒收到 costmap)'
        if cost < 0:
            return 'cost=%d (未知區域)' % cost
        if cost >= 100:
            return 'cost=%d (LETHAL，等同障礙物本體)' % cost
        if cost >= 99:
            return 'cost=%d (INSCRIBED，車體一定碰到)' % cost
        if cost > 0:
            return 'cost=%d (膨脹層內)' % cost
        return 'cost=0 (自由)'

    def obstacles_cb(self, msg):
        """隨時更新作業區內部的障礙物輪廓"""
        self.latest_obstacles = list(msg.polygons)

    def call_f2c_planner(self):
        """打包邊界並發送給 C++ 伺服器"""
        if self.latest_boundary is None:
            if self._boundary_seen > 0:
                self.get_logger().error(
                    f'⚠️ 收過 {self._boundary_seen} 個邊界但都沒通過檢查'
                    f'（面積至少要 {self.min_boundary_area:.2f} m²、頂點至少 4 個）。'
                    f'請繼續在建圖模式下把場地繞完，地圖夠大之後邊界才會合理。')
            else:
                self.get_logger().error(
                    '⚠️ 尚未接收到草地邊界！請先在建圖模式下遙控車輛探索。')
            self.set_mission_state(MissionStatus.STATE_ABORTED)
            return

        # 規劃之前先看車子自己站得下站不下。這一項擋的是「車子當下停的位置」，
        # 與階段 21 那些針對「規劃出來的段落」的檢查互補 —— 見 start_pose_blocked()。
        blocked = self.start_pose_blocked()
        if blocked is not None:
            text = ('無法開始：%s，不足以原地掉頭。'
                    '請先用手動模式(模式 0)把車輛移到開闊處，再切回自動割草。'
                    % blocked)
            self.get_logger().error('🚫 ' + text)
            self.set_mission_state(MissionStatus.STATE_ABORTED, message=text)
            return

        if not self.f2c_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('⚠️ F2C 伺服器未上線，請確認 f2c_server 已經啟動！')
            self.set_mission_state(MissionStatus.STATE_ABORTED)
            return

        # 建立請求
        req = GenerateCoveragePath.Request()
        req.boundary = self.latest_boundary
        # 傳給 F2C 的是「割草線間距」而不是刀盤寬：F2C 的 tool_width 決定的是
        # 相鄰兩條 swath 的距離，縮小它就等於讓相鄰兩刀互相重疊。
        req.tool_width = self.swath_spacing
        req.turning_radius = 1.0 # 迴轉半徑
        # 內部障礙物：F2C 會把它們挖成內環，割草線不會穿過去
        req.obstacles = list(self.latest_obstacles)

        # 從這裡到拿到路徑之間都算「規劃中」，GUI 才不會看起來像沒反應
        self.set_mission_state(MissionStatus.STATE_PLANNING)
        self._mission_total = 0
        self._succeeded = 0
        self._skipped = []
        self.get_logger().info(
            f'🚀 正在將邊界發送給 F2C 伺服器進行運算...'
            f' (內部障礙物 {len(req.obstacles)} 個)')
        future = self.f2c_client.call_async(req)
        future.add_done_callback(self.f2c_response_callback)

    # 模式切換服務
    # 模式切換服務
    def change_mode_callback(self,request,response):
        if request.mode in self.mode_map:
            previous_mode = self.current_mode
            self.current_mode = request.mode
            mode_name = self.mode_map[self.current_mode]
            
            # 加入針對急停的專屬提示
            if self.current_mode == 4:
                self.get_logger().error(f'🚨 系統強制鎖定：切換至 {mode_name}')
                # 只有「真的有任務在跑」時急停才算把任務中止。
                #
                # MissionStatus.state 描述的是「任務」，急停狀態是由
                # MowerStatus 的 stop_active 與 mode 表達的 —— 兩者是不同的關注點。
                # 分開之後「已完成 12/19」與「急停中」可以同時呈現；
                # 混在一起的話，任務跑完之後按急停會把那份完成紀錄蓋掉
                # (而 total_segments 不歸零正是為了保住那份紀錄)，
                # 待命中按急停也會顯示「任務已中止」，但使用者根本沒啟動過任務。
                self.abort_mission_if_running()
                # 取消 Nav2 正在執行的 FollowPath，避免解除急停後車子被拉回原路徑
                self.cancel_current_goal()
                # 【安全關鍵】急停不能只取消「當前這一條」割草線。
                # _swath_queue 裡可能還排著幾十條，若不清空，等一下解除急停時
                # 結果回呼會接著把下一條送出去，車子會在沒有人下令的情況下
                # 突然自己動起來。所以急停一定要把整個任務佇列倒掉。
                self.clear_swath_queue()
            else:
                self.get_logger().info(f'成功切換至 {mode_name}')

                # 【安全關鍵】從 F2C(1) 或保留的 mode 3 離開時同樣要清空佇列。
                # 理由同急停：殘留在佇列裡的割草線會被結果回呼接續執行，
                # 使用者以為已經離開自動模式，車子卻還會自己跑完剩下的任務。
                if previous_mode in (1, 3) and self.current_mode != previous_mode:
                    self.cancel_current_goal()
                    self.clear_swath_queue()
                    # 同上：只有跑到一半被切走才算中止。
                    # 任務已經跑完 (DONE) 之後離開自動模式，那份結果要留著。
                    self.abort_mission_if_running()

                # 【新增】：如果切換到 F2C 模式(1)，就自動呼叫算圖
                if self.current_mode == 1:
                    self.call_f2c_planner()
            
            self.stop_robot()
            # 修正：重置 watchdog 計時器，避免切換瞬間報錯
            self.last_cmd_time = self.get_clock().now()
            response.success = True

        else:
            self.get_logger().error(f'無效模式編號:{request.mode}')
            response.success = False
            
        return response
    
    def safety_check(self):
        # 急停模式(4)：每次 timer 觸發都持續送零速度，
        # 而不是只在切換的那一瞬間送一次。
        if self.current_mode == 4:
            self.stop_robot()
            return

        # 其餘模式(0/1/2/3)一律套用通用 watchdog：
        # 超過 watchdog_timeout 沒收到任何 cmd_vel(手把或 Nav2)就停車。
        now = self.get_clock().now()
        elapsed_time = (now-self.last_cmd_time).nanoseconds/1e9

        if elapsed_time > self.watchdog_timeout:
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        self.real_vel_pub.publish(stop_msg)
        
    # ==================================================================
    # 狀態發布 (階段 18)
    # ==================================================================
    def publish_status(self):
        """5 Hz 發布 /mower_status 與 /mission_status。

        這是 GUI 唯一的資訊來源，所以寧可多發不要少發：
        HMI 端超過 1 秒沒收到就會把畫面變灰並停用模式按鈕，
        讓使用者不會把舊畫面當成即時狀態。
        """
        now = self.get_clock().now().to_msg()

        st = MowerStatus()
        st.mode = int(self.current_mode)
        st.stop_active = (self.current_mode == 4)
        # 以下欄位目前沒有來源，一律填 0 / false。
        # 實車的 bridge_node 接上驅動板之後才有真值 (見 MowerStatus.msg 的註解)。
        st.battery_voltage = 0.0
        st.battery_current = 0.0
        st.battery_percentage = 0.0
        st.bumper_pressed = False
        st.is_overheated = False
        self.mower_status_pub.publish(st)

        ms = MissionStatus()
        ms.header.stamp = now
        ms.header.frame_id = 'map'
        ms.state = int(self._mission_state)
        ms.total_segments = int(self._mission_total)
        ms.completed_segments = int(self._succeeded)
        ms.skipped_segments = int(len(self._skipped))
        ms.current_label = self._current_label
        labels = []
        for label, start, reason in self._skipped:
            if start is not None:
                labels.append('%s (%.2f, %.2f)' % (label, start[0], start[1]))
            else:
                labels.append('%s (起點未知)' % label)
            _ = reason
        ms.skipped_labels = labels
        ms.message = self._mission_message
        self.mission_status_pub.publish(ms)

    def abort_mission_if_running(self):
        """只有 PLANNING / EXECUTING 才轉成 ABORTED；IDLE 與 DONE 維持原狀。

        呼叫時機是「外力把任務打斷」(急停、離開自動模式)。
        沒有任務在跑的時候不要覆寫 state —— 理由見 change_mode_callback 的註解。
        """
        if self._mission_state in (MissionStatus.STATE_PLANNING,
                                   MissionStatus.STATE_EXECUTING):
            self.set_mission_state(MissionStatus.STATE_ABORTED)

    def set_mission_state(self, state, label=None, message=''):
        """集中改任務狀態，順便把「目前段落」與說明訊息一起更新。

        message 預設是空字串而不是 None：每一次狀態轉換都要把上一則訊息清掉，
        否則「無法開始」的字樣會留在畫面上，使用者把車推開、任務正常跑起來之後
        還看得到它。要顯示訊息的人自己傳進來。
        """
        self._mission_state = state
        self._mission_message = message
        if label is not None:
            self._current_label = label
        if state in (MissionStatus.STATE_IDLE, MissionStatus.STATE_DONE,
                     MissionStatus.STATE_ABORTED):
            self._current_label = ''

    # ==================================================================
    # 覆蓋任務：路徑切割與循序執行
    # ==================================================================
    def clear_swath_queue(self):
        """倒掉整個割草線佇列，並把進度歸零"""
        if self._swath_queue:
            remaining = len(self._swath_queue) - self._current_swath_idx
            self.get_logger().warn(f'🧹 清空覆蓋任務佇列 (還有 {remaining} 個任務未執行)')
        self._swath_queue = []
        self._current_swath_idx = 0
        self._swath_total = 0
        self._result_future = None
        self._align_phase = False
        self._aligned_idx = None
        self._reset_approach_tracking()

    def cancel_current_goal(self):
        """取消目前正在執行的 FollowPath goal

        取消之前先立旗標：結果回呼會先看這個旗標，確保「我們主動喊停」
        永遠走「整個停掉」那條路，不會被誤判成控制器自己放棄而繼續下一段。
        """
        if self._goal_handle is not None:
            self._cancelling = True
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # approach 路徑的兩個常數
    APPROACH_SKIP_DISTANCE = 0.5      # 距離小於這個值就不用 approach
    APPROACH_WAYPOINT_SPACING = WAYPOINT_SPACING   # 與割草線同一個來源

    # ---- approach 迴圈的終止條件 (階段 23) ----
    #
    # 【為什麼需要】
    # approach 完成之後 manager 會重新量「離目標多遠」，還是太遠就再插一段。
    # 問題是這個迴圈由**連續成功**構成：每一段 approach 都回 SUCCEEDED，
    # 所以 max_consecutive_failures 永遠歸零、永遠擋不到。
    # 實測 (報告 21 節)：跳過周邊環繞 4/5 之後連插 5 段 approach，
    # 量到的距離是 0.55 → 0.63 → 0.52 → 0.55 → 0.51 → 0.50，
    # 在 0.5 m 門檻邊上徘徊了 87.8 秒，最後靠「剛好掉到 0.4999」才跳出來。
    # manager 手上一直有這串數字，卻沒有任何地方發現它不收斂。
    #
    # 原則：**一個沒有改變任何東西的重試，不應該被重試。**
    # 放棄接近之後直接送出原本的段落 —— 它本來就只差不到 0.5 m，
    # 控制器自己就走得到；繼續插 approach 只是換個包裝重試同一件事。
    APPROACH_MIN_IMPROVEMENT = 0.1    # 兩次之間距離至少要縮短這麼多才算有進展
    APPROACH_MAX_TRIES = 2            # 同一個目標最多連續插入幾段 approach

    def get_robot_pose_in_map(self):
        """查詢車子在 map 座標系的當下位置，查不到就回傳 None。

        查不到時絕對不能拿舊資料硬上：位置算錯會讓 approach 路徑把車子帶去
        完全錯誤的地方，寧可不啟動任務。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=2.0))
        except Exception as exc:
            self.get_logger().error(
                f'⚠️ 查不到 map -> base_footprint 的 TF，無法得知車子位置：{exc}')
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def build_approach_path(self, robot_xy, first_swath, label='approach'):
        """產生「從某個位置前往下一段路徑起點」的移動路徑。

        label 只影響 log 的字樣：起點是車子當下位置時叫 approach，
        用在「周邊環繞結束 -> 第 1 條割草線起點」時叫銜接段。

        為什麼 approach 要當成獨立的一個 FollowPath goal，而不是接在第 1 條割草線前面：
        (a) 接上去的話「割草線 1」會變成 L 形路徑，中間有一個大轉折，等於把我們
            做逐條分解想消滅的「單一路徑內含方向劇變」問題又種回去。
        (b) 語意上 approach 是「移動」不是「割草作業」，真實割草機在這段路上刀盤
            是關的；分成不同任務，之後要接刀盤控制時這個分界可以直接拿來用。
        (c) log 與報告數據分得開，才知道失敗是卡在移動還是卡在割草。

        距離小於 APPROACH_SKIP_DISTANCE 時回傳 None，代表不需要 approach。
        """
        start = first_swath.poses[0].pose.position
        dx = start.x - robot_xy[0]
        dy = start.y - robot_xy[1]
        distance = math.hypot(dx, dy)

        if distance < self.APPROACH_SKIP_DISTANCE:
            self.get_logger().info(
                f'🚗 跳過 {label}：車子距離第 1 條割草線起點只有 {distance:.2f} m '
                f'(< {self.APPROACH_SKIP_DISTANCE} m)，直接開始割草')
            return None

        num_segments = max(1, int(round(distance / self.APPROACH_WAYPOINT_SPACING)))
        approach = Path()
        # frame_id 跟割草線一致，都是 F2C 路徑的 "map"
        approach.header.frame_id = first_swath.header.frame_id
        approach.header.stamp = self.get_clock().now().to_msg()

        yaw = math.atan2(dy, dx)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        for k in range(num_segments + 1):
            ratio = float(k) / float(num_segments)
            pose = PoseStamped()
            pose.header = approach.header
            pose.pose.position.x = robot_xy[0] + dx * ratio
            pose.pose.position.y = robot_xy[1] + dy * ratio
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            approach.poses.append(pose)

        # 最後一個航點改成朝向第 1 條割草線的行進方向，
        # 讓車子走完 approach 就已經對準，減少開始割草前的原地迴轉。
        target_q = first_swath.poses[0].pose.orientation
        final_q = approach.poses[-1].pose.orientation
        final_q.x, final_q.y = target_q.x, target_q.y
        final_q.z, final_q.w = target_q.z, target_q.w

        self.get_logger().info(
            f'🚗 產生 {label} 路徑：距離 {distance:.2f} m，{len(approach.poses)} 個航點')
        return approach

    # 跑道的航點間距，與割草線、approach 同一個來源
    LEAD_IN_WAYPOINT_SPACING = WAYPOINT_SPACING

    # 跑道起點與障礙物之間至少要留這麼多 = local_costmap 的 inflation_radius，
    # 路徑落在膨脹層裡控制器會走不動 (不是改 costmap 參數，只是拿它當判斷依據)。
    # 階段 38 以前寫死 0.45 並註明「與 inflation_radius 一致」；現在 self.LEAD_IN_OBSTACLE_CLEARANCE
    # 在 __init__ 由參數 soft_inflation_radius 設定，與 nav2 讀同一個來源 (vehicle_geometry)。

    # ---- 兩種淨空門檻：直線通過 vs 原地掉頭 ----
    #
    # 【直線通過】車體側面不重疊 = 側向半寬 lateral_half_extent = body_width_total / 2
    #   (階段 38：0.42。舊車 base_link 在車體中心時它剛好等於內切半徑 0.34，是巧合；
    #    base_link 移到後輪軸之後內切半徑是 0.155，不能拿來當通過門檻，
    #    否則跑道點離邊界 0.16 m 就放行，車身側面已經壓出邊界 0.26 m)
    #
    # 【原地掉頭】繞後輪軸旋轉時車頭兩角掃出的圓 = rotation_swept_radius
    #   hypot(front_extent 0.975, 0.42) = 1.0616 m (階段 37 以前是舊車外接半徑 0.5841)
    #   淨空小於這個值時，原地旋轉一定會掃到障礙物。
    #
    # 兩個值在 __init__ 裡由參數設定 (self.LEAD_IN_BOUNDARY_CLEARANCE / self.ROTATION_CLEARANCE)，
    # 來源是 vehicle_geometry.py，不要寫死。costmap 的內接半徑 0.155 只給 Nav2 inflation 用。
    #
    # 這個區分是階段 21 才補上的。之前所有檢查都用 0.34，
    # 結果實車那次在 (-5.36, -4.57) 卡住：該點淨空 0.492 m，
    # 大於 0.34 所以檢查放行，但小於 0.5841，車子掉不了頭。
    # DWB 的 BaseObstacle critic 只看軌跡中心點的 cost (中心是自由的)，
    # 所以它不會判軌跡無效，只會一直發指令直到 progress checker
    # 15 秒後中止 —— 全程沒有任何一則錯誤訊息指出真正的原因。

    # 跑道長度的退讓階梯。不可行時依序縮短，取第一個可行的；全部不行就不加跑道。
    # 不是「有或沒有」二選一：0.3 m 的跑道仍然能收斂一部分橫向誤差，
    # 比完全沒有好。
    LEAD_IN_FALLBACKS = (0.3, 0.2, 0.0)

    # 檢查跑道時沿線取樣的間距
    LEAD_IN_CHECK_SPACING = 0.1

    @staticmethod
    def _point_in_polygon(x, y, pts):
        inside = False
        j = len(pts) - 1
        for i in range(len(pts)):
            xi, yi = pts[i]
            xj, yj = pts[j]
            if (yi > y) != (yj > y) and \
                    x < (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi:
                inside = not inside
            j = i
        return inside

    @staticmethod
    def _point_polygon_distance(x, y, pts):
        best = float('inf')
        for i in range(len(pts)):
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % len(pts)]
            vx, vy = bx - ax, by - ay
            norm2 = vx * vx + vy * vy
            t = 0.0 if norm2 < 1e-18 else max(
                0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / norm2))
            best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
        return best

    def boundary_clearance(self, x, y):
        """這個點離邊界多遠。在邊界外面回傳負值，沒有邊界時回傳 None。"""
        if self.latest_boundary is None:
            return None
        pts = [(p.x, p.y) for p in self.latest_boundary.points]
        if len(pts) < 3:
            return None
        d = self._point_polygon_distance(x, y, pts)
        return d if self._point_in_polygon(x, y, pts) else -d

    def obstacle_clearance(self, x, y):
        """這個點離最近的內部障礙物多遠。在障礙物裡回傳負值，沒有障礙物回傳 None。"""
        best = None
        for poly in self.latest_obstacles:
            pts = [(p.x, p.y) for p in poly.points]
            if len(pts) < 3:
                continue
            d = self._point_polygon_distance(x, y, pts)
            if self._point_in_polygon(x, y, pts):
                d = -d
            best = d if best is None else min(best, d)
        return best

    def lead_in_point_unsafe(self, x, y, need_rotation=False):
        """這個點適不適合當跑道的一部分？不安全就回傳原因字串，安全回傳 None。

        檢查兩件事 —— **邊界與內部障礙物都要看**：

        1. 邊界：點必須落在邊界多邊形**裡面**，而且離邊界至少
           LEAD_IN_BOUNDARY_CLEARANCE (側向半寬 lateral_half_extent)。
           跑道是沿著割草線往**反方向**延伸的，而 F2C 的地頭本來就貼著邊界，
           所以延伸出去的起點很容易落到邊界外。實測 demo_lawn 上
           割草線 2/36 的跑道起點 x=5.51 直接落在牆體 (5.50~5.70) 裡面，
           controller_server 立刻回 ABORTED。
        2. 內部障礙物：離障礙物至少 LEAD_IN_OBSTACLE_CLEARANCE (soft_inflation_radius，
           = costmap 的 inflation_radius)。

        舊版的 lead_in_blocked() 只看障礙物、不看邊界，
        名字卻讓人以為它把「跑道能不能走」都檢查過了。
        """
        # 要在這個點掉頭的話，門檻是外接半徑；只是直線通過就用內切半徑。
        # 不要一律用外接半徑 —— 那會把只是路過的點也擋掉，過度保守。
        need_b = (self.ROTATION_CLEARANCE if need_rotation
                  else self.LEAD_IN_BOUNDARY_CLEARANCE)
        need_o = (max(self.ROTATION_CLEARANCE, self.LEAD_IN_OBSTACLE_CLEARANCE)
                  if need_rotation else self.LEAD_IN_OBSTACLE_CLEARANCE)
        what = '掉頭' if need_rotation else '通過'

        od = self.obstacle_clearance(x, y)
        if od is not None:
            if od < 0:
                return '落在內部障礙物裡'
            if od < need_o:
                return ('離內部障礙物只有 %.2f m (%s需要 %.2f m)'
                        % (od, what, need_o))

        bd = self.boundary_clearance(x, y)
        if bd is not None:
            if bd < 0:
                return '落在邊界外面'
            if bd < need_b:
                return '離邊界只有 %.2f m (%s需要 %.2f m)' % (bd, what, need_b)
        return None

    def start_pose_blocked(self):
        """任務開始前，檢查**車子當下停的位置**自己轉不轉得了身。

        回傳不可行的原因字串，可以開始就回傳 None。

        【為什麼需要這一項】
        階段 21 把外接半徑 0.5841 m 的掉頭淨空檢查套在「規劃出來的段落」上：
        跑道起點、approach 目標、佇列每一段的起訖點都查過了。
        但是沒有任何一項檢查涵蓋「使用者按下開始的當下，車子停在哪裡」。
        實測 (報告 18.4 節)：建圖繞完之後車子停在離東牆 0.40 m 的角落，
        第一段 approach 連續三次 `Failed to make progress`，任務 46 秒就中止，
        而且 log 裡沒有任何一則訊息指出真正的原因。

        【為什麼不自己脫困】
        淨空不足時不產生任何脫困動作。74 kg 的機器在受限空間裡自己亂動，
        風險大於讓人把它推開；而且脫困路徑本身也需要它沒有的那塊空間。
        正確的預設是拒絕開始並把原因講清楚。
        """
        xy = self.get_robot_pose_in_map()
        if xy is None:
            # 查不到位置時**不擋**。這一項檢查的職責是「淨空不足就拒絕開始」，
            # 「不知道車子在哪裡」是另一回事，不要混進來變成第二種拒絕理由
            # （訊息會變成叫人去把車推開，但問題根本不在位置上）。
            # 處理方式與 maybe_insert_approach() 一致：警告並放行。
            self.get_logger().warn(
                '⚠️ 查不到車子位置，任務開始前的淨空檢查這次跳過')
            return None
        reason = self.lead_in_point_unsafe(xy[0], xy[1], need_rotation=True)
        if reason is None:
            return None
        return '車輛目前位置 (%.2f, %.2f) %s' % (xy[0], xy[1], reason)

    def lead_in_first_feasible(self, ax, ay, ux, uy, index, total):
        """從設定的長度開始往下試，回傳第一個「整條都安全」的跑道長度。

        回傳 (長度, 不可行的原因) —— 長度 0.0 代表不加跑道。
        整條都要檢查，不能只看起點：起點安全但中段壓在牆上的情況是存在的
        (邊界是凹多邊形時)。
        """
        candidates = [self.lead_in_length] + [
            c for c in self.LEAD_IN_FALLBACKS if c < self.lead_in_length]
        reason = None
        for length in candidates:
            if length <= 0.0:
                return 0.0, reason
            n = max(1, int(round(length / self.LEAD_IN_CHECK_SPACING)))
            bad = None
            for k in range(n + 1):
                d = length * k / n
                # k == n 就是跑道起點：車子會在那裡掉頭進入這條割草線，
                # 所以那個點的門檻是外接半徑；中間的點只是直線通過。
                why = self.lead_in_point_unsafe(
                    ax - d * ux, ay - d * uy, need_rotation=(k == n))
                if why is not None:
                    bad = '%s (距割草線起點 %.2f m 處)' % (why, d)
                    break
            if bad is None:
                # 【不得降低淨空】跑道把起點往邊界推，延伸後如果比原本的
                # 割草線起點更靠近邊界而且掉不了頭，那就是自己製造一個
                # 轉不過去的點 —— 實車那次 0.610 m 被 0.20 m 的跑道推到
                # 0.492 m，就這樣卡死。
                sx, sy = ax - length * ux, ay - length * uy
                new_c = self.boundary_clearance(sx, sy)
                if new_c is not None and new_c < self.ROTATION_CLEARANCE:
                    reason = ('延伸後淨空 %.2f m < 外接半徑 %.2f m'
                              % (new_c, self.ROTATION_CLEARANCE))
                    continue
                return length, reason
            reason = bad
        return 0.0, reason

    def build_lead_in(self, swath, index, total):
        """在割草線起點「往後」延伸一段共線的跑道，回傳 跑道 + 割草線 的完整路徑。

        幾何：割草線從 A 到 B，方向 d = normalize(B - A)，
        跑道起點 = A - L*d，所以整條路徑是 (A - L*d) -> A -> ... -> B，全部共線。
        因為跟割草線共線，不會在單一路徑內產生方向劇變，
        也就不會重新引入我們做逐條分解想消滅的問題。

        為什麼需要：實測每條割草線的最大覆蓋落差固定卡在 0.47~0.50 m，
        而 0.5 m 正是割草線間距。代表車子每次掉頭都是「切進去」而不是從端點
        進入，每條線開頭大約 0.5~1 m 根本沒走到。跑道就是給車子一段
        「還沒開始算覆蓋」的距離先把橫向誤差收斂掉。

        L <= 0 時直接回傳原本的割草線 (停用跑道)。
        """
        if self.lead_in_length <= 0.0 or len(swath.poses) < 2:
            return swath

        a = swath.poses[0].pose.position
        b = swath.poses[-1].pose.position
        dx, dy = b.x - a.x, b.y - a.y
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return swath
        ux, uy = dx / norm, dy / norm

        # 跑道起點 = A - L*d
        sx = a.x - self.lead_in_length * ux
        sy = a.y - self.lead_in_length * uy

        length, why = self.lead_in_first_feasible(a.x, a.y, ux, uy, index, total)
        self._lead_in_hist[length] = self._lead_in_hist.get(length, 0) + 1
        if length <= 0.0:
            self.get_logger().info(
                f'🛬 割草線 {index}/{total} 不加跑道：'
                f'{self.lead_in_length:.2f}/0.30/0.20 m 都不可行 —— {why}')
            return swath
        if length < self.lead_in_length:
            self.get_logger().info(
                f'🛬 割草線 {index}/{total} 跑道縮短為 {length:.2f} m '
                f'(原 {self.lead_in_length:.2f} m 不可行：{why})')
            sx = a.x - length * ux
            sy = a.y - length * uy

        num_segments = max(
            1, int(round(length / self.LEAD_IN_WAYPOINT_SPACING)))

        out = Path()
        out.header.frame_id = swath.header.frame_id
        out.header.stamp = swath.header.stamp

        # 跑道航點的朝向就是割草線的行進方向 (共線)
        yaw = math.atan2(uy, ux)
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        # k 只跑到 num_segments - 1：第 num_segments 個點就是 A 本身，
        # 由後面接上的割草線提供，避免重複航點。
        for k in range(num_segments):
            ratio = float(k) / float(num_segments)
            pose = PoseStamped()
            pose.header = out.header
            pose.pose.position.x = sx + (a.x - sx) * ratio
            pose.pose.position.y = sy + (a.y - sy) * ratio
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            out.poses.append(pose)

        n_lead = len(out.poses)
        out.poses.extend(swath.poses)

        # 這個分界之後接刀盤控制時會用到：跑道段刀盤要關，割草段才打開。
        self.get_logger().info(
            f'🛬 割草線 {index}/{total} 加跑道：長度 {length:.2f} m，'
            f'起點 ({sx:.2f}, {sy:.2f}) -> A ({a.x:.2f}, {a.y:.2f})；'
            f'航點 0..{n_lead - 1} 為跑道段，{n_lead}..{len(out.poses) - 1} 為割草段')
        return out

    PERIMETER_CLOSE_TOL = 1e-3

    def split_off_perimeter(self, path_msg):
        """把 F2C 路徑最前面的「周邊環繞」那一圈切下來。

        f2c_server 把環繞放在整條路徑的最前面，而且把環繞最後一個航點的座標
        設成與第 0 個航點完全相同 (回到起點)。弓字形割草線不會回到起點，
        所以用這個特徵就能把兩者分開，不必為了傳一個旗標去動 srv 介面。

        回傳 (環繞航點 list, 其餘航點 list)。沒有環繞時回傳 ([], 全部航點)，
        舊版 f2c_server 的路徑因此完全照原本的流程走。
        """
        poses = path_msg.poses
        if len(poses) < 4:
            return [], list(poses)
        p0 = poses[0].pose.position
        for k in range(3, len(poses)):
            pk = poses[k].pose.position
            if math.hypot(pk.x - p0.x, pk.y - p0.y) <= self.PERIMETER_CLOSE_TOL:
                return list(poses[:k + 1]), list(poses[k + 1:])
        return [], list(poses)

    # 相鄰兩個航點距離超過這個值就視為「跳接」而不是同一段路徑。
    # 割草線與環繞的內插間距都是 0.1 m，所以 0.3 m 已經有 3 倍餘裕；
    # 挖掉內部障礙物之後，同一列的割草線會被障礙物切成前後兩段，
    # 這兩段是「共線」的，方向完全沒變，只靠 90 度規則切不開，
    # 不切開的話送給 Nav2 的那條路徑會直接穿過障礙物 (報告 7.10 節)。
    GAP_CUT_DISTANCE = 0.3

    def cut_on_direction_change(self, poses):
        """依「方向變化達 90 度」或「相鄰航點距離過大」把航點切成多段，只留 >= 3 點的段。

        割草線與周邊環繞共用同一條規則：割草線之間夾的是垂直的橫向連接段，
        環繞的轉角同樣是接近 90 度的轉折，兩者都要在轉折處切開，
        才能讓控制器的視野裡一次只有一條直線 (理由見 split_path_into_swaths)。
        """
        if len(poses) < 2:
            return []

        cuts = set()
        prev_dir = None
        for i in range(len(poses) - 1):
            dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
            dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
            norm = math.hypot(dx, dy)
            if norm < 1e-9:
                continue
            if norm > self.GAP_CUT_DISTANCE:
                # 跳接：兩端都切開，中間那兩個航點自成一段 (只有 2 點，會被丟掉)
                cuts.add(i)
                cuts.add(min(i + 1, len(poses) - 1))
            cur_dir = (dx / norm, dy / norm)
            if prev_dir is not None:
                dot = prev_dir[0] * cur_dir[0] + prev_dir[1] * cur_dir[1]
                # dot <= 0 代表方向變化達到 90 度 (含) 以上就切開。
                # F2C 的橫向連接段剛好跟割草線垂直，用嚴格的「大於 90 度」會切不開。
                if dot <= 1e-9:
                    cuts.add(i)
            prev_dir = cur_dir

        segments = []
        start = 0
        for cut in sorted(cuts) + [len(poses) - 1]:
            segment = poses[start:cut + 1]
            if len(segment) >= 3:      # 2 個航點的是橫向連接段，不是割草線
                segments.append(list(segment))
            start = cut
        return segments

    def drop_closed_segments(self, queue):
        """【不變量 ②】起點等於終點的段落不得送出 (階段 38 決定 3)。

        起點與終點相距 <= goal_xy_tolerance 的段落，goal checker 會在送出當下判定到達 ——
        等於宣告完成了一段一公尺都沒開的路。這不是可調參數；這裡是送進佇列前的最後一道關。
        queue: [(Path, 標籤)]，回傳過濾後的 list。被丟掉的每一段都印 ERROR。
        """
        kept = []
        for pth, lbl in queue:
            if not pth.poses:
                continue
            a, b = pth.poses[0].pose.position, pth.poses[-1].pose.position
            d = math.hypot(b.x - a.x, b.y - a.y)
            if d <= self.goal_xy_tolerance:
                self.get_logger().error(
                    f'❌ [{lbl}] 起點 = 終點 (相距 {d:.3f} m <= goal_xy_tolerance '
                    f'{self.goal_xy_tolerance:.2f} m)，不送出')
                continue
            kept.append((pth, lbl))
        return kept

    def split_perimeter_into_edges(self, poses, header):
        """把周邊環繞那一圈依轉角切成一段一段的直邊。

        整圈當成單一個 FollowPath goal 送出去是行不通的：轉角處方向劇變，
        DWB 的距離場評分會在轉角前就把車子拉向下一條邊。切成直邊之後，
        每一段的處理方式就跟割草線一樣。

        環繞段不加跑道 (lead-in)。跑道是沿著行進方向「往後」延伸 0.5 m，
        對環繞的邊來說那個位置在轉角外側，已經超出作業區、落進
        costmap 的膨脹層，車子根本到不了那裡。
        """
        # 階段 38：環繞改用累積轉角偵測 (perimeter_corner_cuts)，不再用割草線的 90° 規則；
        # 相鄰航點的跳接 (GAP_CUT_DISTANCE，被內部障礙物挖開的缺口) 照舊切開。
        segs = []
        if len(poses) >= 3:
            xy = [(p.pose.position.x, p.pose.position.y) for p in poses]
            cuts = set(perimeter_corner_cuts(xy, self.perimeter_corner_window,
                                             self.perimeter_corner_angle))
            for i in range(len(poses) - 1):
                if math.hypot(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1]) > self.GAP_CUT_DISTANCE:
                    cuts.add(i)
                    cuts.add(i + 1)
            start = 0
            for cut in sorted(cuts) + [len(poses) - 1]:
                seg = poses[start:cut + 1]
                if len(seg) >= 3:
                    segs.append(list(seg))
                start = cut
        # 【不變量 ①】閉合迴圈不得整段送出：切完仍有一段「起點與終點距離 <= goal_xy_tolerance」
        # (轉角偵測一刀都沒切到時就是整圈)，在離起點最遠的航點切成兩段。這不是可調參數 ——
        # 送出一個起點等於終點的目標然後宣告完成，是邏輯錯誤。
        fixed = []
        for seg in segs:
            a, b = seg[0].pose.position, seg[-1].pose.position
            if len(seg) >= 5 and math.hypot(b.x - a.x, b.y - a.y) <= self.goal_xy_tolerance:
                far = max(range(len(seg)), key=lambda k: math.hypot(
                    seg[k].pose.position.x - a.x, seg[k].pose.position.y - a.y))
                self.get_logger().warn(
                    f'⚠️ 周邊環繞有一段起點 = 終點 (閉合迴圈，{len(seg)} 個航點)，'
                    f'在離起點最遠的航點 {far} 切成兩段')
                fixed += [seg[:far + 1], seg[far:]]
            else:
                fixed.append(seg)
        edges = []
        for seg in fixed:
            sub = Path()
            sub.header.frame_id = header.frame_id
            sub.header.stamp = header.stamp
            sub.poses = seg
            edges.append(sub)
        if edges:
            total_len = 0.0
            for e in edges:
                for k in range(len(e.poses) - 1):
                    total_len += math.hypot(
                        e.poses[k + 1].pose.position.x - e.poses[k].pose.position.x,
                        e.poses[k + 1].pose.position.y - e.poses[k].pose.position.y)
            self.get_logger().info(
                f'🔄 周邊環繞切出 {len(edges)} 段直邊，全長 {total_len:.2f} m '
                f'(環繞段不加跑道)')
        return edges

    def split_path_into_swaths(self, path_msg):
        """把整條覆蓋路徑依「行進方向反轉」切成一條一條的割草線。

        DWB 的 PathDist / GoalDist 都是 MapGridCritic，在 local costmap 上用網格
        距離場評分，沒有路徑順序或弧長的概念。割草線間距只有 0.5 公尺時，距離場在
        兩線之間幾乎是平的，沒有梯度推車子沿線前進；GoalDist 的梯度又直指終點，
        等於鼓勵車子斜切穿過所有割草線。所以這裡先把路徑拆開，讓控制器的視野裡
        永遠只有一條直線。

        F2C 送來的路徑形狀是 [割草線1 ... 割草線1, 割草線2 ... 割草線2, ...]，
        兩條割草線之間夾著一段垂直的橫向連接段 (長度等於刀盤寬度)。相鄰航點的方向
        變化達到 90 度就切開，因此連接段會自成一個「只有 2 個航點」的片段；那不是
        割草線 (內插過的割草線至少有 3 個航點)，直接丟掉不送。
        """
        swaths = []
        for segment in self.cut_on_direction_change(path_msg.poses):
            sub = Path()
            sub.header.frame_id = path_msg.header.frame_id
            sub.header.stamp = path_msg.header.stamp
            sub.poses = segment
            swaths.append(sub)

        self.get_logger().info(f'✂️ 覆蓋路徑切出 {len(swaths)} 條割草線')
        for idx, sw in enumerate(swaths):
            length = 0.0
            for k in range(len(sw.poses) - 1):
                length += math.hypot(
                    sw.poses[k + 1].pose.position.x - sw.poses[k].pose.position.x,
                    sw.poses[k + 1].pose.position.y - sw.poses[k].pose.position.y)
            self.get_logger().info(
                f'   割草線 {idx + 1}: {len(sw.poses)} 個航點, 長度 {length:.2f} m')
        return swaths

    def maybe_insert_approach(self):
        """送出目前這一段之前，先看車子離它的起點多遠；太遠就插一段 approach。

        **任務開頭、環繞結束、跳過某一段之後，全部走這一條路徑。**
        以前這件事分散在三個地方，而「跳過之後」那個地方根本沒有寫，
        於是跳過一段之後送出的下一段起點常常落在 local costmap 之外，
        controller_server 直接回 0 poses 再 ABORTED，
        skip-and-continue 因此保證會連續失敗到門檻。

        回傳 True 代表有插入。
        """
        if self._current_swath_idx >= len(self._swath_queue):
            return False
        path, label = self._swath_queue[self._current_swath_idx]
        if label == 'approach':
            return False            # 已經是 approach，不要再包一層
        robot_xy = self.get_robot_pose_in_map()
        if robot_xy is None:
            self.get_logger().warn(
                '⚠️ 查不到車子位置，這一段直接送出，不補 approach')
            return False
        target = self.approach_target(path, label)
        tgt = target.poses[0].pose.position
        distance = math.hypot(tgt.x - robot_xy[0], tgt.y - robot_xy[1])

        # 換了一個目標段落就重新計數
        if label != self._approach_for:
            self._reset_approach_tracking()
            self._approach_for = label
        self._approach_dists.append(distance)

        # 已經為這個目標插過 approach 了，檢查它到底有沒有讓事情前進。
        # 距離已經在門檻之內時不要走這裡：那是正常結束，交給
        # build_approach_path 印「只差 x.xx m，直接開始割草」就好，
        # 不要多一則看起來像出事的警告。
        if self._approach_tries > 0 and distance >= self.APPROACH_SKIP_DISTANCE:
            chain = ' → '.join('%.2f m' % d for d in self._approach_dists)
            improvement = self._approach_dists[-2] - distance
            give_up = None
            if improvement < self.APPROACH_MIN_IMPROVEMENT:
                give_up = (f'approach 連續 {self._approach_tries} 次未改善距離 '
                           f'({chain})')
            elif self._approach_tries >= self.APPROACH_MAX_TRIES:
                give_up = (f'approach 已連續插入 {self._approach_tries} 段仍未到達 '
                           f'({chain})，達到上限')
            if give_up is not None:
                # 【放棄接近之後的退路依距離分支（階段 26）】
                # 原本一律「直接送出該段落」，假設它只差不到 0.5 m。實測有一次
                # 目標在 9.8 m 外，起點落在 local costmap 之外，
                # controller_server 0.4 s 就回 0 poses —— 保證失敗，
                # 還白白吃掉一次連續失敗計數（報告 26 節）。
                reach = self.local_costmap_reach()
                if reach is not None and distance > reach:
                    self.get_logger().warn(
                        f'⚠️ [{label}] {give_up}，目標仍在 {distance:.2f} m 外、'
                        f'超出局部代價地圖範圍 ({reach:.2f} m)，'
                        f'直接送出必然失敗，跳過此段')
                    p0 = path.poses[0].pose.position if path.poses else None
                    self._skipped.append(
                        (label, (p0.x, p0.y) if p0 else None, '超出局部代價地圖範圍'))
                    # 沒有送給 Nav2，不算控制器失敗，連續失敗計數不動。
                    self._current_swath_idx += 1
                    self._reset_approach_tracking()
                    # 下一段同樣要先看要不要補 approach
                    return self.maybe_insert_approach()
                self.get_logger().warn(
                    f'⚠️ [{label}] {give_up}，放棄接近，直接送出該段落')
                return False

        approach = self.build_approach_path(robot_xy, target)
        if approach is None:
            return False            # 夠近，不需要 approach
        self._approach_tries += 1
        self._swath_queue.insert(self._current_swath_idx, (approach, 'approach'))
        # 佇列變長了，畫面上的總段數要跟著變，不然進度條會超過 100%
        self._mission_total = len(self._swath_queue)
        return True

    def local_costmap_reach(self):
        """送出去的路徑至少要有一個航點離車子在這個距離內，控制器才接得住。

        不寫死：從收到的 local costmap 算 max(寬, 高) 格數 × 解析度 / 2，
        也就是 DWB transformGlobalPlan 裡的 dist_threshold。現在的設定 (5 m x 5 m) 是 2.50 m。
        還沒收到 costmap 就回傳 None（無法判斷）。

        注意這是上界，不是 DWB 實際的門檻：prune_plan 開著（預設）時，
        DWB 用的是 min(dist_threshold, prune_distance)，prune_distance 沒寫在
        nav2_params.yaml 裡、預設 2.0 m。所以 2.0 ~ 2.5 m 之間的目標這裡會放行，
        控制器仍可能回 0 poses；這裡擋的是 9.8 m 那種遠遠超出的情況（報告 26 節）。
        """
        cm = self.local_costmap
        if cm is None:
            return None
        return max(cm.info.width, cm.info.height) * cm.info.resolution / 2.0

    def _reset_approach_tracking(self):
        self._approach_for = None
        self._approach_dists = []
        self._approach_tries = 0

    APPROACH_TARGET_SEARCH_M = 1.0     # 最多往後找這麼長

    def approach_target(self, path, label=''):
        """挑 approach 要開到哪個航點。

        預設是這一段的第 0 個航點，但那個點如果掉不了頭
        (淨空 < 外接半徑)，把車子開到那裡只是換個地方卡住 ——
        實車那次就是這樣：approach 把車開到 (-5.36, -4.57)，
        然後下一段要往反方向，車子在那裡轉不過來。

        所以往後找第一個「掉得了頭」的航點當終點，最多找
        APPROACH_TARGET_SEARCH_M 公尺；找不到就維持第 0 個
        (至少行為與以前一樣，而且會留下一行 log 說明)。
        """
        if not path.poses:
            return path
        limit = self.APPROACH_TARGET_SEARCH_M
        acc = 0.0
        chosen = 0
        for i, ps in enumerate(path.poses):
            p = ps.pose.position
            if i > 0:
                q = path.poses[i - 1].pose.position
                acc += math.hypot(p.x - q.x, p.y - q.y)
                if acc > limit:
                    break
            if self.lead_in_point_unsafe(p.x, p.y, need_rotation=True) is None:
                chosen = i
                break
        else:
            chosen = 0
        if chosen == 0:
            first = path.poses[0].pose.position
            why = self.lead_in_point_unsafe(first.x, first.y, need_rotation=True)
            if why is not None:
                self.get_logger().warn(
                    f'⚠️ [{label}] 的起點掉不了頭（{why}），'
                    f'往後 {limit:.1f} m 內也找不到可以掉頭的點，'
                    f'仍然開到原起點 —— 這一段有可能卡住')
            return path
        sub = Path()
        sub.header = path.header
        sub.poses = list(path.poses[chosen:])
        skipped = sum(
            math.hypot(path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
                       path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y)
            for i in range(chosen))
        self.get_logger().info(
            f'🚗 [{label}] 的起點掉不了頭，approach 改開到第 {chosen} 個航點 '
            f'(往後 {skipped:.2f} m)')
        return sub

    def send_next_swath(self):
        """送出佇列裡的下一個任務 (approach 或割草線)"""
        self.maybe_insert_approach()
        total = len(self._swath_queue)
        if self._current_swath_idx >= total:
            self.log_mission_summary()
            self.set_mission_state(MissionStatus.STATE_DONE)
            self.clear_swath_queue()
            self.stop_robot()
            return
        path, label = self._swath_queue[self._current_swath_idx]
        self._mission_state = MissionStatus.STATE_EXECUTING
        # 【兩階段，階段 38 決定 1】每一段 (含 approach) 先原地對正，對正成功之後才送那一段本身。
        # 原因：DWB 在段落開頭是「邊前進邊轉」(vx 0.70 + wz 1.2 的弧)，新車車頭在旋轉中心前方 0.975 m，
        # 同一個弧讓車頭角撞牆 / 頂住箱子 (docs/stage38_collision_brief.md)。
        if self._aligned_idx != self._current_swath_idx and self.send_align(path, label):
            return
        self._current_label = label
        self.get_logger().info(f'➡️ 送出任務 [{label}] ({len(path.poses)} 個航點)')
        self.send_path_to_nav2(path, label)

    # 對正階段用的控制器與 goal checker (階段 38 決定 1)，要與 nav2_params.yaml 一致
    ALIGN_CONTROLLER = 'AlignController'
    ALIGN_GOAL_CHECKER = 'align_goal_checker'
    # 取路徑方向時，跳過起點附近這個距離以內的航點 (第一步太短，方向不穩)
    ALIGN_HEADING_LOOKAHEAD = 0.2

    def send_align(self, path, label):
        """送出對正 goal：原地 (車子當下位置) 轉到這一段的行進方向。送出了回傳 True。

        目標是單一位姿 (當下 xy、段落方向)，用 AlignController (max_vel_x = 0，只能原地轉)
        與 align_goal_checker (yaw 0.10、stateful false)。**不要**改用 approach_goal_checker：
        它不檢查朝向，目標又在當下位置，送出的瞬間就會假成功。
        查不到車子位置時不對正、直接送那一段 (與 maybe_insert_approach 的處理一致：警告並放行)。
        """
        if len(path.poses) < 2:
            return False
        p0 = path.poses[0].pose.position
        far = next((q.pose.position for q in path.poses[1:]
                    if math.hypot(q.pose.position.x - p0.x, q.pose.position.y - p0.y)
                    >= self.ALIGN_HEADING_LOOKAHEAD), path.poses[-1].pose.position)
        yaw = math.atan2(far.y - p0.y, far.x - p0.x)
        xy = self.get_robot_pose_in_map()
        if xy is None:
            self.get_logger().warn(f'⚠️ [{label}] 查不到車子位置，這一段不對正、直接送出')
            return False
        target = Path()
        target.header.frame_id = path.header.frame_id or 'map'
        target.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position.x, pose.pose.position.y = xy[0], xy[1]
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        target.poses = [pose]
        self._align_phase = True
        self._current_label = f'對正 → {label}'
        self.get_logger().info(
            f'↪️ 對正 [{label}]：原地轉到 {math.degrees(yaw):.1f}° (位置 {xy[0]:.2f}, {xy[1]:.2f})')
        self.send_path_to_nav2(target, label, controller_id=self.ALIGN_CONTROLLER,
                               goal_checker_id=self.ALIGN_GOAL_CHECKER)
        return True

    def log_mission_summary(self, ended_early=False):
        """任務結束時印一份摘要：總段數 / 完成 / 跳過，以及每個跳過段落的座標。

        座標印出來是為了實車：割完之後要知道「哪裡沒割到」才能回頭補，
        只說「跳過 2 段」沒有用。
        """
        total = len(self._swath_queue)
        n_skip = len(self._skipped)
        head = '🏁 覆蓋任務結束' if (ended_early or n_skip) else '🏁 覆蓋任務完成'
        self.get_logger().info(
            f'{head}：共 {total} 段，完成 {self._succeeded} 段，跳過 {n_skip} 段')
        for label, start, reason in self._skipped:
            where = f'(起點 {start[0]:.2f}, {start[1]:.2f})' if start else '(起點未知)'
            self.get_logger().info(f'   未完成：{label} {where} 原因 {reason}')
        # 全部做完而且沒有跳過任何一段時，才印這一行。
        # 它的意思就是「整個任務完整跑完」，測試套件也是靠它判斷任務有沒有走完，
        # 所以有跳過的時候不能印，否則等於謊報。
        if not ended_early and n_skip == 0:
            self.get_logger().info(
                f'🏁 覆蓋任務完成！共完成 {self._swath_total} 條割草線')

    def follow_path_result_callback(self, future):
        """一段路徑執行完的結果處理。

        三種結果分開處理，不要混在一起：

        SUCCEEDED  送下一段，並把連續失敗次數歸零。

        CANCELED（我們主動喊停：急停 mode 4、或離開 mode 1/3）
                   清空佇列、整個停掉。**這條路徑的行為與 log 都維持原樣，
                   一個位元都不能變**：解除急停之後車子絕對不能自己接著跑。
                   判斷依據是 cancel_current_goal() 事先立的 _cancelling 旗標，
                   不是只看 status —— 旗標比較可靠。

        ABORTED（控制器自己放棄，例如臨時障礙物擋住那一段）
                   **跳過這一段，繼續下一段。** 割草環境會變化：椅子被搬出來、
                   樹枝掉下來、有人站在草坪上。這些由 local costmap 即時偵測，
                   不需要重新建圖，也不該讓整片草坪停擺。
                   但連續失敗達到 max_consecutive_failures（預設 3）時就停止：
                   那通常不是單一障礙物，而是定位跑掉或 Nav2 掛了，
                   繼續送下去只會讓車子在場上亂撞。
        """
        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        total = len(self._swath_queue)
        if self._current_swath_idx < total:
            path, label = self._swath_queue[self._current_swath_idx]
        else:
            path, label = None, '(未知任務)'

        status_name = {
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(status, f'STATUS_{status}')

        # ---- 0. 對正階段的結果 (階段 38 決定 1) ----
        # 取消 (急停 / 離開 mode 1) 不在這裡處理：往下走第 1 條，行為與訊息一個位元都不變。
        if self._align_phase and not (self._cancelling or status == GoalStatus.STATUS_CANCELED):
            self._align_phase = False
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._aligned_idx = self._current_swath_idx
                self._current_label = label
                self.get_logger().info(f'✅ 對正完成 [{label}]')
                self.get_logger().info(f'➡️ 送出任務 [{label}] ({len(path.poses)} 個航點)')
                self.send_path_to_nav2(path, label)
                return
            # 對正失敗 = 該段失敗，走下面既有的跳過邏輯 (計入 max_consecutive_failures)。
            # 原地旋轉不平移，progress checker (只看平移) 15 s 一到就判 ABORTED —— 這幾乎一定是對正超時。
            status_name = 'ALIGN_TIMEOUT'
            self.get_logger().error(
                f'⏱️ [{label}] 對正超時 (原地旋轉 15 s 內沒有完成；progress checker 只看平移，'
                f'原地旋轉期間永遠看不到進展) —— 視為這一段失敗')
        self._align_phase = False

        # ---- 1. 主動取消：維持原本的行為與訊息 ----
        if self._cancelling or status == GoalStatus.STATUS_CANCELED:
            self._cancelling = False
            self.get_logger().error(
                f'❌ [{label}] 失敗 (status={status_name})，'
                f'停止整個覆蓋任務，不自動重試')
            self.set_mission_state(MissionStatus.STATE_ABORTED)
            self.clear_swath_queue()
            self.stop_robot()
            return

        # ---- 2. 成功 ----
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._consecutive_failures = 0
            self._succeeded += 1
            if label == 'approach':
                self.get_logger().info('✅ approach 完成，開始割草')
            else:
                self.get_logger().info(f'✅ {label} 完成')
            self._current_swath_idx += 1
            if self._current_swath_idx >= total:
                self.log_mission_summary()
                self.set_mission_state(MissionStatus.STATE_DONE)
                self.clear_swath_queue()
                self.stop_robot()
            else:
                self.send_next_swath()
            return

        # ---- 3. 控制器放棄 (ABORTED 或未知狀態) ----
        self._consecutive_failures += 1
        start = None
        if path is not None and path.poses:
            p0 = path.poses[0].pose.position
            start = (p0.x, p0.y)
        self._skipped.append((label, start, status_name))
        self.get_logger().error(
            f'❌ [{label}] 失敗 (status={status_name})，'
            f'連續失敗 {self._consecutive_failures}/{self.max_consecutive_failures} 次')

        if self._consecutive_failures >= self.max_consecutive_failures:
            # 【只列事實，不要猜原因】
            # 舊版印的是「(定位跑掉、Nav2 異常、或整片區域無法通行)」——
            # 那是猜的，而且實測那次的真正原因 (跑道起點落在牆體裡、
            # 下一段起點在 local costmap 之外) 三個都不在列舉裡，
            # 只會把看 log 的人帶去查錯的方向。
            self.get_logger().error(
                f'🛑 連續 {self._consecutive_failures} 段失敗，'
                f'達到 max_consecutive_failures={self.max_consecutive_failures}，'
                f'停止整個覆蓋任務。這 {self._consecutive_failures} 段分別是：')
            for label, start, reason in self._skipped[-self._consecutive_failures:]:
                if start is None:
                    self.get_logger().error(f'   {label}：起點未知，status={reason}')
                    continue
                cost = self.costmap_cost_at(start[0], start[1])
                self.get_logger().error(
                    f'   {label}：起點 ({start[0]:.2f}, {start[1]:.2f})，'
                    f'status={reason}，起點 {self.describe_cost(cost)}')
            self.log_mission_summary(ended_early=True)
            self.set_mission_state(MissionStatus.STATE_ABORTED)
            self.clear_swath_queue()
            self.stop_robot()
            return

        where = f'(起點 {start[0]:.2f}, {start[1]:.2f})' if start else '(起點未知)'
        self.get_logger().warn(
            f'⏭️ 跳過此段 {label} {where}，繼續下一段'
            f'（臨時障礙物不應該讓整片草坪停擺）')
        self._current_swath_idx += 1
        if self._current_swath_idx >= total:
            self.log_mission_summary()
            self.set_mission_state(MissionStatus.STATE_DONE)
            self.clear_swath_queue()
            self.stop_robot()
        else:
            self.send_next_swath()

    # 兩個 goal checker 的名字 (階段 25)，要與 nav2_params.yaml 的
    # goal_checker_plugins 完全一致。
    #
    # 【為什麼不能用空字串表示「用預設」】
    # controller_server 只有在**只註冊一個** goal checker 時才接受空字串。
    # 一旦 goal_checker_plugins 有兩個，空字串會被拒絕：
    #   FollowPath called with goal_checker name  in parameter
    #   'current_goal_checker', which does not exist.
    #   Available goal checkers are: general_goal_checker approach_goal_checker .
    # 實測代價：那一趟 83 段裡有 40 段因此直接 ABORTED
    # （docs/simulation_results.md 25.2 節）。
    # 所以非 approach 的段落要把 general_goal_checker 明寫出來。
    APPROACH_GOAL_CHECKER = 'approach_goal_checker'
    DEFAULT_GOAL_CHECKER = 'general_goal_checker'

    def send_path_to_nav2(self, path_msg, label='', controller_id=None, goal_checker_id=None):
        """將一條割草線打包成 Action Goal 交給 Nav2 底層控制器。

        label 決定要用哪一個 goal checker：

        * approach —— 用 approach_goal_checker（只看位置，yaw 容忍 3.15 rad）。
          approach 的刀盤是關的，朝向沒有作業意義；而它到站時的車頭方向
          與下一段要走的方向平均差 171 度（報告 22.3 節）。用預設的
          goal checker 會讓車子在終點原地轉那 171 度，
          stateful 又已經把 xy 關掉，於是平均飄開 0.36 m 才回報成功。
        * 其他（割草線、周邊環繞）—— 明寫 general_goal_checker（不能留空字串，
          原因見 DEFAULT_GOAL_CHECKER 的註解）。
          這些段落到站時本來就已經對準（實測飄移只有 0.009 m），
          朝向也真的有意義，不要動它。
        """
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('⚠️ 找不到 Nav2 的 follow_path 伺服器，請確認 Nav2 是否正常啟動！')
            return

        self.get_logger().info('🚀 啟接割草任務！正在將路徑交給 Nav2 控制器...')
        
        # 建立 FollowPath 的目標請求
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = controller_id or 'FollowPath'  # 對正階段用 AlignController
        goal_msg.goal_checker_id = goal_checker_id or (
            self.APPROACH_GOAL_CHECKER if label == 'approach'
            else self.DEFAULT_GOAL_CHECKER)

        # 發送非同步 Action 請求
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        確認 Nav2 是否成功接受了我們的循跡任務
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 拒絕了割草路徑！(可能是路徑起點距離車體太遠)')
            # 被拒絕代表這條割草線送不出去，整個任務沒辦法繼續，比照失敗處理
            self.clear_swath_queue()
            self.stop_robot()
            return
        # 保存 handle，讓急停時可以取消這個任務
        self._goal_handle = goal_handle
        self.get_logger().info('✅ Nav2 已接受路徑，車輛開始移動！')
        # 接上結果回呼，這條走完才知道要送下一條還是整個停掉
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.follow_path_result_callback)

    def f2c_response_callback(self, future):
        """
        接收 C++ 算好的路徑並廣播與導航
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'🎉 成功拿到 F2C 路徑！總航點數: {len(response.coverage_path.poses)}')
                # 【關鍵】：把路徑廣播出去給 RViz2 畫圖
                self.path_pub.publish(response.coverage_path)
                # 先把最前面的周邊環繞那一圈切下來 (沒有環繞時就是空的)，
                # 剩下的才是弓字形割草線。
                perimeter_poses, swath_poses = self.split_off_perimeter(
                    response.coverage_path)
                rest = Path()
                rest.header = response.coverage_path.header
                rest.poses = swath_poses
                perimeter_edges = self.split_perimeter_into_edges(
                    perimeter_poses, response.coverage_path.header)
                # 切成一條一條割草線後循序執行，一次只給 Nav2 一條直線
                swaths = self.split_path_into_swaths(rest)
                if not swaths:
                    self.get_logger().error('⚠️ 這條覆蓋路徑切不出任何割草線，任務中止。')
                    self.set_mission_state(MissionStatus.STATE_ABORTED)
                    self.clear_swath_queue()
                    return

                # 查車子當下位置，算出前往第 1 條割草線起點的 approach 路徑。
                # 查不到 TF 就不啟動任務，不要用舊資料硬上。
                robot_xy = self.get_robot_pose_in_map()
                if robot_xy is None:
                    self.get_logger().error('⚠️ 不知道車子在哪裡，不啟動覆蓋任務。')
                    self.set_mission_state(MissionStatus.STATE_ABORTED)
                    self.clear_swath_queue()
                    return

                total = len(swaths)
                # 每條割草線前面接上共線的跑道，讓車子掉完頭先收斂橫向誤差，
                # 進入真正的割草段時已經貼在線上。
                # 跑道不可行 (會壓到邊界或障礙物) 時會自動縮短，見 build_lead_in。
                self._lead_in_hist = {}
                swaths = [self.build_lead_in(sw, i + 1, total)
                          for i, sw in enumerate(swaths)]
                hist = ', '.join(
                    '%.2f m x %d 條' % (k, v)
                    for k, v in sorted(self._lead_in_hist.items(), reverse=True))
                self.get_logger().info(f'🛬 跑道長度分布：{hist}')
                # 周邊環繞排在割草線前面：先把作業區邊緣繞一圈再開始弓字形。
                # _swath_total 仍然只算割草線，「共完成 N 條割草線」的意義不變。
                n_edges = len(perimeter_edges)
                queue = [(e, f'周邊環繞 {i + 1}/{n_edges}')
                         for i, e in enumerate(perimeter_edges)]
                queue += [(sw, f'割草線 {i + 1}/{total}') for i, sw in enumerate(swaths)]
                # 【不變量 ②】起點等於終點的段落不得送出 (見 drop_closed_segments)
                queue = self.drop_closed_segments(queue)
                # 【不在這裡插 approach】
                # 「車子離下一段的起點太遠就先開過去」這件事，以前在三個地方
                # 各寫一份 (任務開頭的 approach、環繞結束的銜接段、
                # 以及跳過某一段之後 —— 最後這個根本忘了寫)。
                # 忘了寫的後果是：跳過一段之後直接送下一段，而下一段的起點
                # 常常在 5x5 m 的 local costmap 之外，controller_server 立刻
                # 回「Resulting plan has 0 poses in it」再 ABORTED ——
                # skip-and-continue 因此在結構上保證自己會撞到連續失敗門檻。
                # 現在統一由 send_next_swath() 呼叫 maybe_insert_approach()，
                # 正常推進、跳過之後、任務開頭全部走同一條路徑。
                _ = robot_xy

                self._swath_queue = queue
                self._swath_total = total
                self._current_swath_idx = 0
                # 每次重新規劃都是一個新任務，失敗統計要歸零
                self._consecutive_failures = 0
                self._skipped = []
                self._succeeded = 0
                self._cancelling = False
                # 佇列總段數 (含 approach / 環繞 / 銜接段)，任務結束後仍保留給畫面顯示
                self._mission_total = len(queue)
                self.set_mission_state(MissionStatus.STATE_EXECUTING)
                # 把整個佇列印出來：Phase Q 靠這些行檢查「真正會送出去的段落」
                # 是否可通行 (靜態的 F2C 輸出看不到跑道與執行時插入的 approach)。
                for i, (pth, lbl) in enumerate(queue):
                    if not pth.poses:
                        continue
                    a = pth.poses[0].pose.position
                    b = pth.poses[-1].pose.position
                    length = 0.0
                    for k in range(len(pth.poses) - 1):
                        p0 = pth.poses[k].pose.position
                        p1 = pth.poses[k + 1].pose.position
                        length += math.hypot(p1.x - p0.x, p1.y - p0.y)
                    self.get_logger().info(
                        '📋 佇列 %d/%d [%s] 起點 (%.2f, %.2f) 終點 (%.2f, %.2f) '
                        '長度 %.2f m' % (i + 1, len(queue), lbl,
                                        a.x, a.y, b.x, b.y, length))
                self.send_next_swath()
            else:
                self.get_logger().error('⚠️ F2C 伺服器回報路徑規劃失敗！')
                self.set_mission_state(MissionStatus.STATE_ABORTED)
        except Exception as e:
            self.get_logger().error(f'呼叫 F2C 服務時發生錯誤: {str(e)}')
            self.set_mission_state(MissionStatus.STATE_ABORTED)


def main(args=None):
    rclpy.init(args=args)
    node = MowerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()