#!/usr/bin/env python3
import math
import tf2_ros
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from mowerbot_interfaces.srv import GenerateCoveragePath
from mowerbot_interfaces.msg import ObstaclePolygons, MowerStatus, MissionStatus
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import FollowPath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mowerbot_interfaces.srv import SetDriveMode

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
        self._current_label = ''
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
        self.declare_parameter('blade_width', 0.5)
        self.declare_parameter('overlap_ratio', 0.4)
        self.blade_width = float(self.get_parameter('blade_width').value)
        self.overlap_ratio = float(self.get_parameter('overlap_ratio').value)
        self.swath_spacing = self.blade_width * (1.0 - self.overlap_ratio)
        self.get_logger().info(
            f'🔪 實際刀盤寬 = {self.blade_width:.3f} m，'
            f'重疊率 = {self.overlap_ratio:.2f}，'
            f'割草線間距 = {self.swath_spacing:.3f} m')
        # 模式標籤說明現況：
        # mode 1 與 mode 3 在 nav_vel_cb 裡的行為完全相同(兩者都轉發 /cmd_vel_nav)，
        # 唯一差別是切到 mode 1 會觸發 call_f2c_planner() 去規劃並執行覆蓋任務。
        # 也就是說 mode 3 目前沒有任何獨立功能，標籤先改誠實，行為不動。
        # 未來 mode 3 可用於點對點導航(接 planner_server 的 ComputePathToPose)，
        # 屆時兩個模式才會真正分家。
        self.mode_map = {
            0: '建圖模式(SLAM)',
            1: '自動割草(F2C規劃+執行)',
            2: '手動模式',
            3: '自動導航(保留，目前與模式1行為相同)',
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
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/f2c_boundary', self.boundary_cb, 10)

        # 訂閱作業區內部的障礙物輪廓 (階段 11)。
        # 沒有收到訊息時維持空 list，送給 F2C 的 obstacles 就是空的，
        # 行為與階段 10 完全相同。
        self.latest_obstacles = []
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
    接收導航速度：僅在 F2C(1) 或自動導航(3) 模式下轉發 
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
    def boundary_cb(self, msg):
        """隨時更新最新圈出的綠色邊界"""
        self.latest_boundary = msg.polygon

    def obstacles_cb(self, msg):
        """隨時更新作業區內部的障礙物輪廓"""
        self.latest_obstacles = list(msg.polygons)

    def call_f2c_planner(self):
        """打包邊界並發送給 C++ 伺服器"""
        if self.latest_boundary is None:
            self.get_logger().error('⚠️ 尚未接收到草地邊界！請先在建圖模式下遙控車輛探索。')
            self.set_mission_state(MissionStatus.STATE_ABORTED)
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
                # 急停就是把「正在做的事」整個中止 —— 沒有任務在跑時也一樣標成
                # ABORTED，因為對操作者而言「系統被強制停下」比「待命」更精確。
                self.set_mission_state(MissionStatus.STATE_ABORTED)
                # 取消 Nav2 正在執行的 FollowPath，避免解除急停後車子被拉回原路徑
                self.cancel_current_goal()
                # 【安全關鍵】急停不能只取消「當前這一條」割草線。
                # _swath_queue 裡可能還排著幾十條，若不清空，等一下解除急停時
                # 結果回呼會接著把下一條送出去，車子會在沒有人下令的情況下
                # 突然自己動起來。所以急停一定要把整個任務佇列倒掉。
                self.clear_swath_queue()
            else:
                self.get_logger().info(f'成功切換至 {mode_name}')

                # 【安全關鍵】從 F2C(1) 或自動導航(3) 離開時同樣要清空佇列。
                # 理由同急停：殘留在佇列裡的割草線會被結果回呼接續執行，
                # 使用者以為已經離開自動模式，車子卻還會自己跑完剩下的任務。
                if previous_mode in (1, 3) and self.current_mode != previous_mode:
                    self.cancel_current_goal()
                    self.clear_swath_queue()
                    self.set_mission_state(MissionStatus.STATE_ABORTED)

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
        self.mission_status_pub.publish(ms)

    def set_mission_state(self, state, label=None):
        """集中改任務狀態，順便把「目前段落」一起更新，避免兩邊各改一半"""
        self._mission_state = state
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
    APPROACH_WAYPOINT_SPACING = 0.1   # 與割草線的航點間距一致

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

    # 跑道的航點間距，與割草線、approach 一致
    LEAD_IN_WAYPOINT_SPACING = 0.1

    # 跑道起點與障礙物之間至少要留這麼多。0.45 m 是 local_costmap 的
    # inflation_radius，路徑落在膨脹層裡控制器會走不動 (不是改 costmap 參數，
    # 只是拿它當判斷依據)。
    LEAD_IN_OBSTACLE_CLEARANCE = 0.45

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

    def lead_in_blocked(self, x, y):
        """跑道起點是不是落在障礙物裡、或離障礙物太近。

        作業區內部有障礙物時，被障礙物切成兩段的割草線，後半段的起點距離
        障礙物剛好是 F2C 的地頭寬度 (0.5 m)，再往後延伸 0.5 m 的跑道起點
        就正好壓在障礙物邊緣，整段跑道都在 costmap 的膨脹層裡，
        控制器會走不動。這種情況下寧可不要跑道。
        """
        for poly in self.latest_obstacles:
            pts = [(p.x, p.y) for p in poly.points]
            if len(pts) < 3:
                continue
            if self._point_in_polygon(x, y, pts):
                return True
            if self._point_polygon_distance(x, y, pts) < \
                    self.LEAD_IN_OBSTACLE_CLEARANCE:
                return True
        return False

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

        if self.lead_in_blocked(sx, sy):
            self.get_logger().info(
                f'🛬 割草線 {index}/{total} 不加跑道：跑道起點 ({sx:.2f}, {sy:.2f}) '
                f'落在內部障礙物裡或離它不到 '
                f'{self.LEAD_IN_OBSTACLE_CLEARANCE} m')
            return swath

        num_segments = max(
            1, int(round(self.lead_in_length / self.LEAD_IN_WAYPOINT_SPACING)))

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
            f'🛬 割草線 {index}/{total} 加跑道：長度 {self.lead_in_length:.2f} m，'
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

    def split_perimeter_into_edges(self, poses, header):
        """把周邊環繞那一圈依轉角切成一段一段的直邊。

        整圈當成單一個 FollowPath goal 送出去是行不通的：轉角處方向劇變，
        DWB 的距離場評分會在轉角前就把車子拉向下一條邊。切成直邊之後，
        每一段的處理方式就跟割草線一樣。

        環繞段不加跑道 (lead-in)。跑道是沿著行進方向「往後」延伸 0.5 m，
        對環繞的邊來說那個位置在轉角外側，已經超出作業區、落進
        costmap 的膨脹層，車子根本到不了那裡。
        """
        edges = []
        for seg in self.cut_on_direction_change(poses):
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

    def send_next_swath(self):
        """送出佇列裡的下一個任務 (approach 或割草線)"""
        total = len(self._swath_queue)
        if self._current_swath_idx >= total:
            self.log_mission_summary()
            self.set_mission_state(MissionStatus.STATE_DONE)
            self.clear_swath_queue()
            self.stop_robot()
            return
        path, label = self._swath_queue[self._current_swath_idx]
        self._current_label = label
        self._mission_state = MissionStatus.STATE_EXECUTING
        self.get_logger().info(f'➡️ 送出任務 [{label}] ({len(path.poses)} 個航點)')
        self.send_path_to_nav2(path)

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
            self.get_logger().error(
                f'🛑 連續 {self._consecutive_failures} 段都失敗，'
                f'判定為系統性問題（定位跑掉、Nav2 異常、或整片區域無法通行），'
                f'停止整個覆蓋任務。單一障礙物不會造成連續失敗。')
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

    def send_path_to_nav2(self, path_msg):
        """
        將一條割草線打包成 Action Goal 交給 Nav2 底層控制器
        """
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('⚠️ 找不到 Nav2 的 follow_path 伺服器，請確認 Nav2 是否正常啟動！')
            return

        self.get_logger().info('🚀 啟接割草任務！正在將路徑交給 Nav2 控制器...')
        
        # 建立 FollowPath 的目標請求
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath' # 呼叫 Nav2 預設的循跡控制器

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
                swaths = [self.build_lead_in(sw, i + 1, total)
                          for i, sw in enumerate(swaths)]
                # 周邊環繞排在割草線前面：先把作業區邊緣繞一圈再開始弓字形。
                # _swath_total 仍然只算割草線，「共完成 N 條割草線」的意義不變。
                n_edges = len(perimeter_edges)
                queue = [(e, f'周邊環繞 {i + 1}/{n_edges}')
                         for i, e in enumerate(perimeter_edges)]
                # 環繞是一個封閉的圈，走完會回到它的起點，那裡離第 1 條割草線的
                # 跑道起點可能有好幾公尺。不補一段銜接就直接送割草線的話，
                # 車子要自己從幾公尺外切進路徑，實測 DWB 會在 23 秒後
                # 以 Failed to make progress 中止整個任務。
                if perimeter_edges:
                    end = perimeter_edges[-1].poses[-1].pose.position
                    link = self.build_approach_path(
                        (end.x, end.y), swaths[0], label='銜接段')
                    if link is not None:
                        queue.append((link, '銜接段'))
                queue += [(sw, f'割草線 {i + 1}/{total}') for i, sw in enumerate(swaths)]
                # swaths[0] 已經是「跑道 + 第 1 條割草線」，所以 approach 的終點
                # 自然就是跑道起點而不是 A，所有割草線的處理方式一致。
                # 有環繞時 approach 的終點改成環繞的第一段起點。
                approach = self.build_approach_path(robot_xy, queue[0][0])
                if approach is not None:
                    queue.insert(0, (approach, 'approach'))

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