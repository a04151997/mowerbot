#!/usr/bin/env python3
"""bridge_node：ROS 與馬達驅動板之間的橋。

【這個節點在整條鏈路裡的位置】
  下行  /cmd_vel (geometry_msgs/Twist)
          -> 差速運動學 -> 左右輪角速度 -> driver.set_wheel_velocities()
  上行  driver.read_encoders()
          -> 里程計積分 -> /odom 與 odom -> base_footprint 的 TF
          -> 另外發布 MotorStatus

【為什麼上行這麼重要】
模擬時 Gazebo 的 diff_drive plugin 免費提供 /odom 與 odom->base_footprint 的 TF，
實車上沒有任何東西會發這兩樣。而 slam_toolbox 硬性要求 odom->base_footprint，
Nav2 的 local_costmap 也要。沒有里程計，實車上整條鏈路等於零。

【與驅動板無關】
這個節點不知道任何板子的通訊協定，只透過 drivers/base.py 的介面講話。
換板子時只要新增一個 driver 實作，這個檔案不用改（見 drivers/README.md）。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from mowerbot_interfaces.msg import MotorStatus

from .drivers import create_driver, DriverError
from .odometry import DifferentialOdometry, body_to_wheel


class MowerBridge(Node):

    def __init__(self):
        super().__init__('mower_bridge')

        # ==============================================================
        # 1. 參數
        # ==============================================================
        # 車體物理量：都是實車量到的值，不是調出來的。
        # 來自 mowerbot_description/config/vehicle.yaml (由 bringup_real.launch.py 傳入)，
        # 與 URDF、Gazebo 的 diff_drive plugin 同一個來源。這裡不給預設值：
        # 沒傳就在 get_parameter 時直接失敗，不要安靜地用一個寫死的數字。
        self.declare_parameter('wheel_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('wheel_separation', Parameter.Type.DOUBLE)

        # 每轉的編碼器 tick 數（含減速比與四倍頻）。
        # 【沒有合理的預設值】這個值只能從驅動板/編碼器的文件查到，
        # 猜錯的話里程計的尺度就是錯的，而且錯得很安靜（SLAM 會慢慢發散，
        # 看起來像「SLAM 不準」而不是「參數填錯」）。
        # 所以這裡用 0 當作「沒有設定」的哨兵值，未設定就拒絕啟動。
        self.declare_parameter('encoder_ticks_per_rev', 0)

        # 校正用的乘數，靠 test/tools/calibrate_odometry.py 實測決定。
        self.declare_parameter('wheel_radius_correction', 1.0)
        # 【這個值一定要校正】四輪 skid-steer 轉彎時輪胎會橫向滑動，
        # 有效輪距比幾何值大 1.3 ~ 1.8 倍。直接用幾何輪距 (vehicle.yaml) 計算的話，
        # 車子實際轉 360 度時里程計可能只累積出 240 度，
        # scan matching 會直接發散。預設 1.0 只是「還沒校正」的意思。
        self.declare_parameter('wheel_separation_correction', 1.0)

        # 馬達或編碼器方向接反時用這兩個修正，不要去改接線
        # （改接線之後別人看程式會對不起來）。
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # 編碼器計數器的模數（16 位元就填 65536）。0 = 驅動層回傳不會回繞的累計值。
        # 真實板子多半會回繞，位元寬度要查板子文件（drivers/README.md 的 C3）。
        self.declare_parameter('encoder_wrap', 0)

        self.declare_parameter('odom_rate', 30.0)
        # 超過這段時間沒收到 /cmd_vel 就送零速度。
        # 這與 mower_manager 的 watchdog 是兩層獨立保護：manager 掛掉、
        # 或 manager 到這裡之間的連線斷掉時，這一層仍然會把車停下來。
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('driver_type', 'loopback')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        p = self.get_parameter
        self.wheel_radius = float(p('wheel_radius').value)
        self.wheel_separation = float(p('wheel_separation').value)
        self.ticks_per_rev = int(p('encoder_ticks_per_rev').value)
        self.radius_correction = float(p('wheel_radius_correction').value)
        self.separation_correction = float(p('wheel_separation_correction').value)
        self.invert_left = bool(p('invert_left').value)
        self.invert_right = bool(p('invert_right').value)
        self.encoder_wrap = int(p('encoder_wrap').value)
        self.odom_rate = float(p('odom_rate').value)
        self.cmd_vel_timeout = float(p('cmd_vel_timeout').value)
        self.driver_type = str(p('driver_type').value)
        self.odom_frame = str(p('odom_frame').value)
        self.base_frame = str(p('base_frame').value)

        if self.ticks_per_rev <= 0:
            raise DriverError(
                'encoder_ticks_per_rev 沒有設定（目前 %d）。'
                '這個值必須從驅動板/編碼器的文件查到（每轉的 tick 數，含減速比與'
                '四倍頻），猜錯會讓里程計的尺度整個錯掉，所以不提供預設值。'
                '啟動時加上 -p encoder_ticks_per_rev:=<實際值>，'
                '或在 launch 檔裡指定。' % self.ticks_per_rev)

        # ==============================================================
        # 2. 驅動層
        # ==============================================================
        self.driver = create_driver(self.driver_type,
                                    ticks_per_rev=self.ticks_per_rev)
        self.driver.connect()

        # ==============================================================
        # 3. 里程計
        # ==============================================================
        self.odom = DifferentialOdometry(
            wheel_radius=self.wheel_radius,
            wheel_separation=self.wheel_separation,
            ticks_per_rev=self.ticks_per_rev,
            radius_correction=self.radius_correction,
            separation_correction=self.separation_correction,
            encoder_wrap=self.encoder_wrap,
            invert_left=self.invert_left,
            invert_right=self.invert_right)

        # ==============================================================
        # 4. ROS 介面
        # ==============================================================
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.motor_pub = self.create_publisher(MotorStatus, 'motor_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self._last_cmd_time = self.get_clock().now()
        self._cmd_is_zero = True
        self._watchdog_tripped = False
        self._last_odom_time = self.get_clock().now()
        self._last_ticks = (0, 0)
        self._warned_read_fail = False

        self.timer = self.create_timer(1.0 / self.odom_rate, self.on_timer)

        self.get_logger().info(
            '🔌 bridge_node 啟動：driver=%s, 輪半徑 %.3f m (x%.3f), '
            '輪距 %.3f m (x%.3f), %d ticks/rev, odom %.1f Hz, watchdog %.2f s'
            % (self.driver_type, self.wheel_radius, self.radius_correction,
               self.wheel_separation, self.separation_correction,
               self.ticks_per_rev, self.odom_rate, self.cmd_vel_timeout))
        if abs(self.separation_correction - 1.0) < 1e-9:
            self.get_logger().warn(
                '⚠️ wheel_separation_correction 還是 1.0（未校正）。'
                '四輪 skid-steer 的有效輪距通常是幾何值的 1.3 ~ 1.8 倍，'
                '沒校正的話轉彎角度會嚴重低估，SLAM 容易發散。'
                '實車上請先跑 test/tools/calibrate_odometry.py。')

    # ------------------------------------------------------------------
    # 下行：/cmd_vel -> 輪速
    # ------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        self._last_cmd_time = self.get_clock().now()
        self._cmd_is_zero = (abs(msg.linear.x) < 1e-9 and abs(msg.angular.z) < 1e-9)
        if self._watchdog_tripped:
            self.get_logger().info('✅ 重新收到 /cmd_vel，watchdog 解除')
            self._watchdog_tripped = False
        self._apply_body_velocity(msg.linear.x, msg.angular.z)

    def _apply_body_velocity(self, v, w):
        left, right = body_to_wheel(
            v, w, self.wheel_radius, self.wheel_separation,
            self.radius_correction, self.separation_correction)
        # 方向接反的修正放在這裡（而不是驅動實作裡），
        # 才能與里程計那邊的 invert 對稱，查問題時只要看這一層。
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right
        try:
            self.driver.set_wheel_velocities(left, right)
        except DriverError as exc:
            self.get_logger().error('驅動層拒絕速度指令: %s' % exc)

    # ------------------------------------------------------------------
    # 上行：編碼器 -> /odom + TF + MotorStatus
    # ------------------------------------------------------------------
    def on_timer(self):
        now = self.get_clock().now()

        # ---- watchdog ----
        elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.cmd_vel_timeout and not self._watchdog_tripped:
            self._watchdog_tripped = True
            self.get_logger().warn(
                '⏱️ 超過 %.2f 秒沒收到 /cmd_vel（實際 %.2f 秒），送零速度'
                % (self.cmd_vel_timeout, elapsed))
            try:
                self.driver.stop()
            except DriverError as exc:
                self.get_logger().error('驅動層停止失敗: %s' % exc)

        # ---- 讀編碼器並積分 ----
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        try:
            reading = self.driver.read_encoders()
        except DriverError as exc:
            self.get_logger().error('讀編碼器失敗: %s' % exc)
            reading = None

        state = self.odom.update(reading, dt)
        if not state.ok:
            if not self._warned_read_fail:
                self.get_logger().warn(
                    '⚠️ 讀不到編碼器，位姿維持上一次的值（不外插）。'
                    '累計失敗 %d 次' % self.odom.missed_reads)
                self._warned_read_fail = True
        else:
            self._warned_read_fail = False
            if reading is not None:
                self._last_ticks = (int(reading[0]), int(reading[1]))

        self._publish_odom(now, state)
        self._publish_motor_status(state)

    def _publish_odom(self, stamp, state):
        qz = math.sin(state.theta * 0.5)
        qw = math.cos(state.theta * 0.5)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = state.v
        msg.twist.twist.angular.z = state.w
        # 共變異數留 0：實車校正之後才有意義的數字可以填（見 hardware_bringup.md）。
        self.odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = state.x
        tf.transform.translation.y = state.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _publish_motor_status(self, state):
        # 由量到的車體速度反推兩側輪速（與下行用同一組幾何量）。
        # 四輪 skid-steer 的同側兩輪是一起驅動的，在板子能分別回報之前，
        # 同側的前後輪填同一個值。
        # TODO: 驅動板若能分別回報四個通道，這裡改成直接填實際值。
        left, right = body_to_wheel(
            state.v, state.w, self.wheel_radius, self.wheel_separation,
            self.radius_correction, self.separation_correction)
        to_rpm = 60.0 / (2.0 * math.pi)

        msg = MotorStatus()
        msg.left_front_rpm = float(left * to_rpm)
        msg.left_behind_rpm = float(left * to_rpm)
        msg.right_front_rpm = float(right * to_rpm)
        msg.right_behind_rpm = float(right * to_rpm)
        msg.left_front_encoder = int(self._last_ticks[0])
        msg.left_behind_encoder = int(self._last_ticks[0])
        msg.right_front_encoder = int(self._last_ticks[1])
        msg.right_behind_encoder = int(self._last_ticks[1])
        self.motor_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            # 參數檢查失敗時 self.driver 還不存在，關閉路徑不能假設它在
            driver = getattr(self, 'driver', None)
            if driver is not None:
                driver.stop()
                driver.disconnect()
        except Exception as exc:       # 關閉路徑不要再丟例外出去
            self.get_logger().error('關閉驅動時出錯: %s' % exc)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MowerBridge()
        rclpy.spin(node)
    except DriverError as exc:
        # 參數缺失或驅動連不上：印清楚的訊息並以非零狀態結束，
        # 不要退化成「節點起來了但車子不會動」。
        print('[bridge_node] 啟動失敗: %s' % exc)
        return 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
