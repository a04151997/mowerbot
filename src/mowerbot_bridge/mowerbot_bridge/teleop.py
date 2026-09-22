import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy                         # 接收手把原始資料
from geometry_msgs.msg import Twist                     # 發布速度指令 (linear.x, angular.z)
from mowerbot_interfaces.srv import SetDriveMode        # 自定義的模式切換服務
from mowerbot_interfaces.msg import JoyStatus           # 手把連線狀態 (給 HMI 看)
                                                        
class MowerTeleop(Node):
    def __init__(self):
        super().__init__('mower_teleop')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('scale_linear', 0.7),('scale_angular', 1.2),
                ('axis_linear', 1),('axis_angular',0),
                ('button_a', 0),('button_b', 1),('button_x',2),('button_y',3),('button_deadman',4),('button_stop',5)
            ]
        )


        self.p = {k: self.get_parameter(k).value for k in ['scale_linear' ,'scale_angular' ,
         'axis_linear' , 'axis_angular' , 
         'button_a' , 'button_b' , 'button_x' , 'button_y' , 'button_deadman','button_stop'
        ]
        }

        # 訂閱 joy 話題，讀取手把硬體狀態
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,10)
        # 發布 cmd_vel_joy，給 Manager 
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel_joy',10)
        self.mode_client = self.create_client(SetDriveMode ,'change_mower_mode')

        self.last_button = [0]*12

        # ---- 手把連線狀態 (階段 19) ----
        # button index 只有這裡從 joystick.yaml 讀得到，所以由 teleop 負責發布，
        # 不要讓第二個節點再抄一份 index。
        self.joy_status_pub = self.create_publisher(JoyStatus, '/joy_status', 10)
        self._last_joy_time = None      # 上一筆 /joy 的時間 (node clock)
        self._deadman_held = False
        self._stop_pressed = False
        self.create_timer(0.2, self.publish_joy_status)

        #手把鎖定
        #False表示目前是鎖定
        self.get_logger().info('手把目前為鎖定狀態，按下LB即可解鎖，解鎖後選可擇模式：A（建圖）, B（F2C）, X(手動), Y（導航）')
        #按鈕B是：1 按鈕A是：0 按鈕X是：2 按鈕Y是：3
        #滾輪左右是0 上下是1 左LB是4 右LB是5

    @staticmethod
    def _button(buttons, index):
        """安全地讀按鍵：index 超出範圍就當成沒按。

        只用在狀態回報上，控制邏輯維持原樣 ——
        /joy_status 不該因為某支手把的按鍵數比較少就把 teleop 弄掛。
        """
        return bool(index < len(buttons) and buttons[index] == 1)

    def publish_joy_status(self):
        """5 Hz 發布 /joy_status。

        失效方向必須是安全的：沒有連線時 deadman_held 與 stop_pressed
        一律回報 false。寧可誤報「安全鈕沒按住」(操作者會再按一次)，
        不可誤報「按住中」(那會讓人以為車子隨時可以動)。
        """
        msg = JoyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._last_joy_time is None:
            msg.connected = False
            msg.deadman_held = False
            msg.stop_pressed = False
            msg.last_msg_age = 999.0
            self.joy_status_pub.publish(msg)
            return
        age = (self.get_clock().now() - self._last_joy_time).nanoseconds * 1e-9
        connected = age < 0.5
        msg.connected = bool(connected)
        msg.deadman_held = bool(self._deadman_held and connected)
        msg.stop_pressed = bool(self._stop_pressed and connected)
        msg.last_msg_age = float(min(age, 999.0))
        self.joy_status_pub.publish(msg)

    def joy_callback(self, data):
        # 先記下狀態再處理控制：下面的急停分支會提早 return，
        # 記在後面的話急停按住期間 /joy_status 就會停在舊資料。
        self._last_joy_time = self.get_clock().now()
        self._deadman_held = self._button(data.buttons, self.p['button_deadman'])
        self._stop_pressed = self._button(data.buttons, self.p['button_stop'])

        if self.last_button is None:
            self.last_button = list(data.buttons)
            return
        
        is_estop_pressed = data.buttons[self.p['button_stop']] == 1
        if is_estop_pressed:
            # 立即發布全 0 的 Twist 強制煞車
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.vel_pub.publish(stop_twist)
        # 確保按鍵按下的瞬間 (Rising edge) 才呼叫服務，避免持續按住造成服務塞車
            if self.last_button[self.p['button_stop']] == 0:
                # 第二道防線：通知 Manager 進入急停模式
                # Manager 收到此模式後，必須拒絕轉發任何後續的移動指令
                self.call_service(4) 
                self.get_logger().error('觸發緊急停止,所有功能已鎖定')
            self.last_button = list(data.buttons)
            return

        LB_ishold = data.buttons[self.p['button_deadman']] ==1
        if LB_ishold:            
                if data.buttons[self.p['button_a']] == 1 and self.last_button[self.p['button_a']] == 0:
                    self.call_service(0) #建圖
                    self.get_logger().info('切換至建圖模式')
                elif data.buttons[self.p['button_b']] == 1 and self.last_button[self.p['button_b']] == 0:
                    self.call_service(1) #F2C
                    self.get_logger().info('切換至 F2C 模式')
                elif data.buttons[self.p['button_x']] == 1 and self.last_button[self.p['button_x']] == 0:
                    self.call_service(2) #手動
                    self.get_logger().info('切換至手動模式')
                elif data.buttons[self.p['button_y']] == 1 and self.last_button[self.p['button_y']] == 0:
                    self.call_service(3) #導航
                    self.get_logger().info('切換導航模式')

                #處理速度輸出：只有按住 LB (deadman) 時才把搖桿值算進 twist
                twist = Twist()
                linear_val = data.axes[self.p['axis_linear']]
                angular_val = data.axes[self.p['axis_angular']]
                if abs(linear_val) < 0.05: linear_val = 0.0
                if abs(angular_val) < 0.05: angular_val = 0.0
                twist.linear.x = linear_val * self.p['scale_linear']            #指令速度(Twist)=搖桿推動量(Joy Data)x最大速限(Scale)
                twist.angular.z = angular_val * self.p['scale_angular']

                self.vel_pub.publish(twist)
        else:
            # 沒按住 LB：持續發布全 0 的 Twist，確保車體停住而不是維持上一個速度
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.vel_pub.publish(stop_twist)

        self.last_button = list(data.buttons)

    def call_service(self,mode):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('服務連線失敗')
            return 
        req = SetDriveMode.Request()
        req.mode = mode
        self.mode_client.call_async(req)
        self.get_logger().info(f'發送切換指令：模式 {mode}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MowerTeleop())
    rclpy.shutdown()