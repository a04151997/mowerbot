import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mowerbot_interfaces.msg import MowerStatus

 #class MowerBridge(Node):
#     def __init__(self):
#         super().__init__('mower_ackermann_bridge')
#         self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)


#         #self.wheels_base = 
#         #self.whheel_radius = 