#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mowerbot_interfaces.srv import SetDriveMode
#按鈕B是：1 按鈕A是：0 按鈕X是：2 按鈕Y是：3
        #滾輪左右是0 上下是1 左LB是4 右LB是5
#A（建圖）, B（F2C）, X(手動), Y（導航） 0 1 2 3 

class MowerManager(Node):
    def __init__(self):
        super().__init__('mower_manager')
        #預設為手動模式
        self.current_mode = 2
        self.mode_map = {
            0:'建圖模式(SLAM)',
            1:'F2C', 
            2:'手動模式',
            3:'自動導航(Nav)'
    }

        #安全計時器 如果0.5秒內沒收到任何速度指令則停止馬達
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

    #速度指令處理
    """
    接收手把速度：僅在手動(2)或建圖(0)模式下轉發
    """
    def joy_vel_cb(self,msg):
        if self.current_mode == 0 or self.current_mode == 2:
            self.publish_and_update(msg)

    """ 
    接收導航速度：僅在 F2C(1) 或自動導航(3) 模式下轉發 
    """
    def nav_vel_cb(self,msg):
        if self.current_mode == 1 or self.current_mode ==3:
            self.publish_and_update(msg)


    def publish_and_update(self,msg):
        self.real_vel_pub.publish(msg)
        self.last_cmd_time = self.get_clock().now()
        
    #模式切換服務
    def change_mode_callback(self,request,response):
        if request.mode in self.mode_map:
            self.current_mode = request.mode
            mode_name = self.mode_map[self.current_mode]
            self.get_logger().info(f'成功切換至{mode_name}')
            self.stop_robot()
            response.success = True

        else:
            self.get_logger().error(f'無效模式編號:{request.mode}')
            response.success = False
        return response
    
    def safety_check(self):
        now = self.get_clock().now()
        elapsed_time = (now-self.last_cmd_time).nanoseconds/1e9

        if elapsed_time>self.watchdog_timeout:
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

     