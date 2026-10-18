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

from mowerbot_action import keepout
from mowerbot_action import lawn_boundary

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

    # 禁區 (2026-10-18) 套到地圖上時往外多擴 1 格：建圖中的地圖 origin 不一定跟存圖時對齊，
    # 換格子時最多差半格，多擴 1 格保證禁區只會變大、不會縮小。
    KEEPOUT_DILATE_CELLS = 1

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
        
        # 草地邊界 (2026-10-08)：boundary_recorder 記下的「沿草地邊緣開一圈」的多邊形。
        # 收到之後自由區域只取它裡面的部分 (戶外草地旁邊是水泥地時，光達看不出草地在哪結束)。
        # 沒收到、或收到空的多邊形 (清除) 時，行為與之前完全相同。
        # 發布端是 transient local，這裡也要 transient local 才收得到比自己早發的那一個。
        self.lawn_polygon = None
        self.last_map = None
        self.lawn_sub = self.create_subscription(
            PolygonStamped, '/lawn_boundary', self.lawn_callback, map_qos)

        # 禁區 (2026-10-18)：使用者在存好的地圖上用紅色畫的區域，由 keepout_server 發布。
        # 禁區裡的格子當成不能割：在作業區中間就變成內部障礙物 (F2C 的內環)，碰到外圈就切掉外圈。
        # 收到「沒有禁區」(全部是 0) 時行為與之前完全相同。
        self.keepout = None
        self.keepout_sub = self.create_subscription(
            OccupancyGrid, '/keepout_mask', self.keepout_callback, map_qos)

        self.get_logger().info('✅ 邊界提取節點已成功掛載 Transient Local QoS，開始監聽 /map...')

    def lawn_callback(self, msg):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            self.lawn_polygon = None
            self.get_logger().info('🌱 草地邊界已清除，割草範圍回到只由地圖決定。')
        elif msg.header.frame_id and msg.header.frame_id != 'map':
            # 這個節點把地圖格子直接換算成 map 座標，不做 TF 轉換
            self.get_logger().error(
                f'⚠️ 草地邊界的 frame_id 是 {msg.header.frame_id}，不是 map，忽略。')
            return
        else:
            self.lawn_polygon = pts
            self.get_logger().info(
                f'🌱 收到草地邊界：{len(pts)} 個頂點、面積 '
                f'{lawn_boundary.polygon_area(pts):.1f} m²，之後的割草範圍只取它裡面的部分。')
        # 不用等下一張地圖，拿最近一張馬上重算
        if self.last_map is not None:
            self.map_callback(self.last_map)

    def keepout_callback(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width)) > 50
        # 沒有禁區的空遮罩 keepout_server 放在 odom (還沒開始建圖時沒有 map)，空的就不必看座標系
        if grid.any() and msg.header.frame_id and msg.header.frame_id != 'map':
            self.get_logger().error(
                f'⚠️ 禁區的 frame_id 是 {msg.header.frame_id}，不是 map，忽略。')
            return
        if not grid.any():
            if self.keepout is not None:
                self.get_logger().info('⛔ 禁區已清除，割草範圍回到只由地圖決定。')
            self.keepout = None
        else:
            self.keepout = (grid, msg.info.resolution,
                            msg.info.origin.position.x, msg.info.origin.position.y)
            self.get_logger().info(
                f'⛔ 收到禁區：{int(np.count_nonzero(grid))} 格 '
                f'({np.count_nonzero(grid) * msg.info.resolution ** 2:.2f} m²)，之後的割草範圍會避開。')
        if self.last_map is not None:
            self.map_callback(self.last_map)

    def map_callback(self, msg):
        self.last_map = msg
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

        # 2.5 草地邊界：自由區域只留草地多邊形裡面的部分 (取交集只會變小，不會變大)。
        # 放在找輪廓之前，所以落在草地外面的未知格、障礙物也不會被當成內部障礙物送出去。
        if self.lawn_polygon is not None:
            lawn_mask = lawn_boundary.rasterize(
                self.lawn_polygon, origin_x, origin_y, resolution, width, height)
            img = cv2.bitwise_and(img, lawn_mask)
            self.get_logger().info(
                f'🌱 套用草地邊界：自由區域 {int(np.count_nonzero(free_space_mask))} 格 -> '
                f'{int(np.count_nonzero(img))} 格')
        
        # 2.6 禁區：禁區裡的格子一律當成不能割 (只會讓作業區變小)。跟草地邊界一樣放在找輪廓之前。
        if self.keepout is not None:
            ko_grid, ko_res, ko_ox, ko_oy = self.keepout
            ko = keepout.resample(ko_grid, ko_res, ko_ox, ko_oy,
                                  width, height, resolution, origin_x, origin_y)
            if self.KEEPOUT_DILATE_CELLS > 0 and ko.any():
                k = 2 * self.KEEPOUT_DILATE_CELLS + 1
                ko = cv2.dilate(ko.astype(np.uint8), np.ones((k, k), np.uint8)) > 0
            before = int(np.count_nonzero(img))
            img[ko] = 0
            self.get_logger().info(
                f'⛔ 套用禁區：自由區域 {before} 格 -> {int(np.count_nonzero(img))} 格')

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