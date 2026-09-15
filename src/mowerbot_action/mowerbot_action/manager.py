#!/usr/bin/env python3
import math
import tf2_ros
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from mowerbot_interfaces.srv import GenerateCoveragePath
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
        self._result_future = None

        # TF：manager 本來完全不知道車子在哪裡，但要算出「從當下位置前往第 1 條
        # 割草線起點」的 approach 路徑就必須知道。F2C 路徑的 frame_id 是 "map"，
        # 所以這裡查 map -> base_footprint，保持在同一個座標系。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
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

    def call_f2c_planner(self):
        """打包邊界並發送給 C++ 伺服器"""
        if self.latest_boundary is None:
            self.get_logger().error('⚠️ 尚未接收到草地邊界！請先在建圖模式下遙控車輛探索。')
            return

        if not self.f2c_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('⚠️ F2C 伺服器未上線，請確認 f2c_server 已經啟動！')
            return

        # 建立請求
        req = GenerateCoveragePath.Request()
        req.boundary = self.latest_boundary
        req.tool_width = 0.5    # 74kg 割草機的刀盤寬度
        req.turning_radius = 1.0 # 迴轉半徑

        self.get_logger().info('🚀 正在將邊界發送給 F2C 伺服器進行運算...')
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
        """取消目前正在執行的 FollowPath goal"""
        if self._goal_handle is not None:
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

    def build_approach_path(self, robot_xy, first_swath):
        """產生「從車子當下位置前往第 1 條割草線起點」的 approach 路徑。

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
                f'🚗 跳過 approach：車子距離第 1 條割草線起點只有 {distance:.2f} m '
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
            f'🚗 產生 approach 路徑：距離 {distance:.2f} m，{len(approach.poses)} 個航點')
        return approach

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
        poses = path_msg.poses
        if len(poses) < 2:
            return []

        cut_indices = []
        prev_dir = None
        for i in range(len(poses) - 1):
            dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
            dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
            norm = math.hypot(dx, dy)
            if norm < 1e-9:
                continue
            cur_dir = (dx / norm, dy / norm)
            if prev_dir is not None:
                dot = prev_dir[0] * cur_dir[0] + prev_dir[1] * cur_dir[1]
                # dot <= 0 代表方向變化達到 90 度 (含) 以上就切開。
                # F2C 的橫向連接段剛好跟割草線垂直，用嚴格的「大於 90 度」會切不開。
                if dot <= 1e-9:
                    cut_indices.append(i)
            prev_dir = cur_dir

        swaths = []
        start = 0
        for cut in cut_indices + [len(poses) - 1]:
            segment = poses[start:cut + 1]
            if len(segment) >= 3:      # 2 個航點的是橫向連接段，不是割草線
                sub = Path()
                sub.header.frame_id = path_msg.header.frame_id
                sub.header.stamp = path_msg.header.stamp
                sub.poses = list(segment)
                swaths.append(sub)
            start = cut

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
            self.get_logger().info(f'🏁 覆蓋任務完成！共完成 {self._swath_total} 條割草線')
            self.clear_swath_queue()
            self.stop_robot()
            return
        path, label = self._swath_queue[self._current_swath_idx]
        self.get_logger().info(f'➡️ 送出任務 [{label}] ({len(path.poses)} 個航點)')
        self.send_path_to_nav2(path)

    def follow_path_result_callback(self, future):
        """一條割草線執行完的結果處理：成功就送下一條，失敗就整個停掉"""
        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        total = len(self._swath_queue)
        if self._current_swath_idx < total:
            label = self._swath_queue[self._current_swath_idx][1]
        else:
            label = '(未知任務)'

        if status == GoalStatus.STATUS_SUCCEEDED:
            if label == 'approach':
                self.get_logger().info('✅ approach 完成，開始割草')
            else:
                self.get_logger().info(f'✅ {label} 完成')
            self._current_swath_idx += 1
            if self._current_swath_idx >= total:
                self.get_logger().info(f'🏁 覆蓋任務完成！共完成 {self._swath_total} 條割草線')
                self.clear_swath_queue()
                self.stop_robot()
            else:
                self.send_next_swath()
        else:
            status_name = {
                GoalStatus.STATUS_CANCELED: 'CANCELED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
            }.get(status, f'STATUS_{status}')
            self.get_logger().error(
                f'❌ [{label}] 失敗 (status={status_name})，'
                f'停止整個覆蓋任務，不自動重試')
            self.clear_swath_queue()
            self.stop_robot()

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
                # 切成一條一條割草線後循序執行，一次只給 Nav2 一條直線
                swaths = self.split_path_into_swaths(response.coverage_path)
                if not swaths:
                    self.get_logger().error('⚠️ 這條覆蓋路徑切不出任何割草線，任務中止。')
                    self.clear_swath_queue()
                    return

                # 查車子當下位置，算出前往第 1 條割草線起點的 approach 路徑。
                # 查不到 TF 就不啟動任務，不要用舊資料硬上。
                robot_xy = self.get_robot_pose_in_map()
                if robot_xy is None:
                    self.get_logger().error('⚠️ 不知道車子在哪裡，不啟動覆蓋任務。')
                    self.clear_swath_queue()
                    return

                total = len(swaths)
                queue = [(sw, f'割草線 {i + 1}/{total}') for i, sw in enumerate(swaths)]
                approach = self.build_approach_path(robot_xy, swaths[0])
                if approach is not None:
                    queue.insert(0, (approach, 'approach'))

                self._swath_queue = queue
                self._swath_total = total
                self._current_swath_idx = 0
                self.send_next_swath()
            else:
                self.get_logger().error('⚠️ F2C 伺服器回報路徑規劃失敗！')
        except Exception as e:
            self.get_logger().error(f'呼叫 F2C 服務時發生錯誤: {str(e)}')


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