#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mowerbot_interfaces.srv import SetDriveMode

class MowerManager(Node):
    def __init__(self):
        super().__init__('mower_manager')
        # 預設為手動模式
        self.current_mode = 2
        self.mode_map = {
            0: '建圖模式(SLAM)',
            1: 'F2C', 
            2: '手動模式',
            3: '自動導航(Nav)',
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
        
    # 模式切換服務
    def change_mode_callback(self,request,response):
        if request.mode in self.mode_map:
            self.current_mode = request.mode
            mode_name = self.mode_map[self.current_mode]
            
            # 加入針對急停的專屬提示
            if self.current_mode == 4:
                self.get_logger().error(f'🚨 系統強制鎖定：切換至 {mode_name}')
            else:
                self.get_logger().info(f'成功切換至 {mode_name}')
                
            self.stop_robot()
            # 修正：重置 watchdog 計時器，避免切換瞬間報錯
            self.last_cmd_time = self.get_clock().now()
            response.success = True

        else:
            self.get_logger().error(f'無效模式編號:{request.mode}')
            response.success = False
        return response
    
    def safety_check(self):
        # 修正：處於急停模式時，直接跳過超時檢查
        if self.current_mode == 4:
            return

        now = self.get_clock().now()
        elapsed_time = (now-self.last_cmd_time).nanoseconds/1e9

        if elapsed_time > self.watchdog_timeout:
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        self.real_vel_pub.publish(stop_msg)

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