#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point32
# 導入 ROS 2 關鍵的 QoS 套件
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import cv2

class MapToBoundaryNode(Node):
    def __init__(self):
        super().__init__('map_to_boundary')
        
        # 【核心修正】：配置與 slam_toolbox 完美的 Transient Local QoS Profile
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,   # 必須是 TRANSIENT_LOCAL
            reliability=ReliabilityPolicy.RELIABLE         # 必須是 RELIABLE
        )
        
        # 使用修正後的 QoS 訂閱地圖
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        
        self.polygon_pub = self.create_publisher(PolygonStamped, '/f2c_boundary', 10)
        
        self.get_logger().info('✅ 邊界提取節點已成功掛載 Transient Local QoS，開始監聽 /map...')

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # 終端機 Debug 訊號一：證明回呼函式成功進來了
        self.get_logger().info(f'📥 成功接收到地圖數據！尺寸: {width}x{height}, 解析度: {resolution}')
        
        # 1. 轉換為 NumPy 二維陣列
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 2. 影像二值化
        # 排除 -1(未知) 與大於 20(有障礙物) 的區域，把 0~20 都當作安全草地
        img = np.zeros((height, width), dtype=np.uint8)
        free_space_mask = (grid >= 0) & (grid <= 20)
        img[free_space_mask] = 255 
        
        # 【影像防錯微調】：給影像四周加上 1 畫素的黑邊，防止草地頂到地圖邊緣導致 OpenCV 找不到輪廓
        padded_img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        
        # 3. 尋找輪廓
        contours, _ = cv2.findContours(padded_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('⚠️ 地圖中尚未發現明確的連續草地空白輪廓。')
            return
            
        # 找出面積最大的輪廓
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 4. 多邊形近似 (簡化頂點數量)
        epsilon = 0.01 * cv2.arcLength(largest_contour, True)
        approx_polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
        
        # 5. 封裝成 ROS 訊息並轉換座標系
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = self.get_clock().now().to_msg()
        poly_msg.header.frame_id = 'map' 
        
        for point in approx_polygon:
            # 因為前面加了 1 畫素的 padded 邊框，這裡座標要減 1 扣回來
            u = point[0][0] - 1
            v = point[0][1] - 1
            
            # 像素轉世界座標
            world_x = origin_x + (u * resolution)
            world_y = origin_y + (v * resolution)
            
            # 把 Z 軸抬高 0.15 公尺 (15公分)，確保絕對浮在灰色地圖上方，免除視覺遮擋
            p32 = Point32(x=float(world_x), y=float(world_y), z=0.15)
            poly_msg.polygon.points.append(p32)
            
        self.polygon_pub.publish(poly_msg)
        
        # 終端機 Debug 訊號二：證明邊界順利算完並送出
        self.get_logger().info(f'🎉 邊界提取成功！共化簡出 {len(approx_polygon)} 個多邊形頂點，已發布至 /f2c_boundary')

def main(args=None):
    rclpy.init(args=args)
    node = MapToBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()