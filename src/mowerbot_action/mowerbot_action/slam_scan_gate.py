#!/usr/bin/env python3
"""按 LB + A (建圖模式) 才開始建圖 (2026-10-18，will 要的手把三種模式)。

【為什麼】以前 bringup 一開，slam_toolbox 就開始建圖，連手動模式也在建。
實際的用法是：在實驗室開機，用手把開到外面的草地，到了才開始建圖。一路建過來的話，
實驗室、走廊、門口全部跟草地連成同一張圖 (割草範圍取最大的那塊空地，會把室內也算進去)，
路上累積的里程計誤差也會帶進草地的地圖。

【做法】實車上 slam_toolbox 改訂閱 /scan_slam (mapper_params 的 scan_topic 由 launch 覆寫)，
這個節點把 /scan 轉送過去，但要等第一次切到建圖模式 (/mower_status 的 mode == 0) 才開始轉送。
在那之前 slam_toolbox 一筆光達資料都收不到：沒有地圖，也沒有 map 座標系。

【開了就不關】建圖開始之後，切回手動 (mode 2) 或自動割草 (mode 1) 照樣轉送，直到 car-stop。
slam_toolbox 同時負責定位 (map -> odom)：停止餵資料，車子在地圖上的位置只剩里程計，
會越走越偏，再開始餵的時候掃描比對也可能對不上、把地圖弄花。要一張新的圖就重開 bringup。
光達本身 (/scan) 一直都在：光達停止區、local costmap、manager 的光達中斷檢查都直接讀 /scan，
不受這個節點影響。

發布 ~/active (std_msgs/Bool，transient local)：true = 已經開始建圖。car-status 讀它。
模擬 (use_sim_time:=true) 不啟動這個節點，slam_toolbox 直接讀 /scan (mower_control.launch.py)。
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from mowerbot_interfaces.msg import MowerStatus

# MowerStatus.mode：0 = 建圖 (manager.py 的 mode_map)
MAPPING_MODE = 0


class SlamScanGate(Node):

    def __init__(self):
        super().__init__('slam_scan_gate')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.open = False
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.active_pub = self.create_publisher(Bool, '~/active', latched)
        # slam_toolbox 用 sensor data (best effort) 訂閱；這裡發 reliable，兩種訂閱者都收得到
        self.scan_pub = self.create_publisher(LaserScan, output_topic, 5)
        self.create_subscription(LaserScan, input_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(MowerStatus, '/mower_status', self.status_cb, 10)
        self.active_pub.publish(Bool(data=False))
        self.get_logger().info(
            f'🗺️ 建圖還沒開始：按住 LB + A 切到建圖模式才開始 ({input_topic} -> {output_topic})。'
            f'在那之前只是遙控車，沒有地圖。')

    def status_cb(self, msg):
        if self.open or msg.mode != MAPPING_MODE:
            return
        self.open = True
        self.active_pub.publish(Bool(data=True))
        self.get_logger().info(
            '🗺️ 切到建圖模式，開始建圖。地圖從現在開始建，之後切回手動或自動割草也會繼續建 '
            '(SLAM 同時負責定位)。要重新建一張圖就 car-stop 再 car-start。')

    def scan_cb(self, msg):
        if self.open:
            self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamScanGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
