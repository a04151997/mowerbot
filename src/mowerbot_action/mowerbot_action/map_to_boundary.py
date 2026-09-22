#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point32, Polygon
from mowerbot_interfaces.msg import ObstaclePolygons
# 導入 ROS 2 關鍵的 QoS 套件
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import cv2

class MapToBoundaryNode(Node):

    # ---- 邊界簡化的兩個參數 (階段 20，數字是量出來的不是喬出來的) ----
    #
    # 【為什麼不用 approxPolyDP(epsilon = 1% 弧長)】
    # 那個寫法沒有方向性保證：它會把凸出來的角截彎取直，產生一個比真實自由
    # 區域「更大」的多邊形，下游 (地頭、跑道、周邊環繞) 全部的餘裕計算因此
    # 失真而且沒有人會發現。
    # 用 demo_lawn 的真實 SLAM 地圖實測：原始輪廓 1589 個點簡化成 13 個點之後，
    # 最大向外偏差 **1.250 m**，而且簡化後的多邊形直接壓在 wall_east_south
    # 上面 (離真牆 0.000 m)。割草線 2/36 的跑道起點 x=5.51 落在牆體
    # 5.50~5.70 之內，就是這樣來的。
    #
    # 【改法】先把外輪廓填實、往內腐蝕，再用一個小的絕對 epsilon 簡化。
    # 腐蝕的方向性是數學保證的 (只會往內)，approxPolyDP 沒有這個保證。
    #
    # 絕對 epsilon = 0.05 m (1 個 costmap cell)：實測最大向外偏差 0.049 m，
    # 也就是「恰好一個 cell」，與 Douglas-Peucker 的容差定義一致。
    # 腐蝕 2 cell = 0.10 m：要蓋掉上面那 0.049 m 還留一個 cell 的餘裕。
    # 實測腐蝕 1 cell 時偏差 0.005 m (還沒歸零)，2 cell 時 0.000 m。
    BOUNDARY_EPSILON_M = 0.05
    BOUNDARY_ERODE_CELLS = 2

    # 小於這個面積的洞不當成障礙物。理由：佔據網格上零星的未知格 (-1) 與
    # 雷射打出來的小雜訊都會變成洞，全部送給 F2C 只會讓割草線被切得很碎。
    # 0.09 m² = 0.3 m x 0.3 m，比車體 (0.95 x 0.68) 小得多，
    # 真正需要繞開的東西不會低於這個尺寸。
    MIN_OBSTACLE_AREA = 0.09

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

        # 作業區內部障礙物的輪廓 (階段 11)。mower_manager 訂閱後轉給 F2C 挖成內環。
        self.obstacle_pub = self.create_publisher(
            ObstaclePolygons, '/f2c_obstacles', 10)
        
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
        # 【階段 11】改用 RETR_CCOMP：它會回傳兩層 —— 第一層是草地的外輪廓，
        # 第二層是外輪廓內部的洞。洞就是作業區內部的障礙物 (或還沒探索到的未知區)，
        # 階段 11 之前用 RETR_EXTERNAL 只拿得到外輪廓，內部障礙物在規劃階段
        # 完全不存在，割草線會直接穿過去 (報告 7.10 節)。
        contours, hierarchy = cv2.findContours(
            padded_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('⚠️ 地圖中尚未發現明確的連續草地空白輪廓。')
            return

        # 找出面積最大的「外輪廓」(hierarchy 的 parent == -1 才是外輪廓)
        outer_idx = [i for i in range(len(contours))
                     if hierarchy is None or hierarchy[0][i][3] == -1]
        if not outer_idx:
            self.get_logger().warn('⚠️ 找不到任何外輪廓。')
            return
        largest_idx = max(outer_idx, key=lambda i: cv2.contourArea(contours[i]))
        largest_contour = contours[largest_idx]

        # 4. 往內腐蝕再簡化，保證輸出的多邊形是真實自由區域的「內縮子集」
        #
        # 先把外輪廓填實再腐蝕，不要直接腐蝕自由區域的遮罩：
        # 建圖中的地圖內部散布著未知格，直接腐蝕會把區域打碎，
        # 最大連通塊會從 123.9 m² 掉到 84.3 m² (實測)。
        # 內部的洞本來就由下面的 RETR_CCOMP 另外當成障礙物處理，
        # 填實只影響「外圈邊界」怎麼畫。
        solid = np.zeros_like(padded_img)
        cv2.drawContours(solid, [largest_contour], -1, 255, cv2.FILLED)
        if self.BOUNDARY_ERODE_CELLS > 0:
            ksize = 2 * self.BOUNDARY_ERODE_CELLS + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
            solid = cv2.erode(solid, kernel)
        eroded, _hier = cv2.findContours(
            solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not eroded:
            self.get_logger().warn(
                '⚠️ 腐蝕 %.2f m 之後沒有剩下任何區域，地圖可能還太小，這次不發布邊界。'
                % (self.BOUNDARY_ERODE_CELLS * resolution))
            return
        inner_contour = max(eroded, key=cv2.contourArea)

        # 絕對 epsilon (公尺) 而不是弧長的百分比：
        # 百分比會讓「場地越大、允許的偏差越大」，那正好與安全需求相反。
        epsilon = self.BOUNDARY_EPSILON_M / resolution
        approx_polygon = cv2.approxPolyDP(inner_contour, epsilon, True)
        
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
        self.get_logger().info(
            f'🎉 邊界提取成功！共化簡出 {len(approx_polygon)} 個多邊形頂點，'
            f'已發布至 /f2c_boundary '
            f'(往內腐蝕 {self.BOUNDARY_ERODE_CELLS * resolution:.2f} m、'
            f'簡化容差 {self.BOUNDARY_EPSILON_M:.2f} m)')

        # 6. 內部障礙物：最大外輪廓底下的每一個洞 (階段 11)
        obs_msg = ObstaclePolygons()
        obs_msg.header = poly_msg.header
        n_skipped = 0
        if hierarchy is not None:
            child = hierarchy[0][largest_idx][2]      # first_child
            while child != -1:
                hole = contours[child]
                area_m2 = abs(cv2.contourArea(hole)) * resolution * resolution
                if area_m2 >= self.MIN_OBSTACLE_AREA:
                    eps = 0.01 * cv2.arcLength(hole, True)
                    approx_hole = cv2.approxPolyDP(hole, eps, True)
                    if len(approx_hole) >= 3:
                        poly = Polygon()
                        for point in approx_hole:
                            u = point[0][0] - 1
                            v = point[0][1] - 1
                            poly.points.append(Point32(
                                x=float(origin_x + u * resolution),
                                y=float(origin_y + v * resolution),
                                z=0.15))
                        obs_msg.polygons.append(poly)
                    else:
                        n_skipped += 1
                else:
                    n_skipped += 1
                child = hierarchy[0][child][0]        # next sibling

        # 空的也要發：manager 才知道「這張地圖裡沒有內部障礙物」，
        # 而不是沿用上一次的結果。
        self.obstacle_pub.publish(obs_msg)
        self.get_logger().info(
            f'🕳️ 內部障礙物 {len(obs_msg.polygons)} 個已發布至 /f2c_obstacles'
            f' (另有 {n_skipped} 個小於 {self.MIN_OBSTACLE_AREA} m² 或頂點不足，已略過)')

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