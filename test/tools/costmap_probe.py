# -*- coding: utf-8 -*-
"""local_costmap 在車體周圍的 cost 取樣 (階段 38，引擎遮擋的影響)。

用法 (整套 demo 起來之後)：python3 test/tools/costmap_probe.py <秒數> <輸出 csv>
每收到一張 /local_costmap/costmap，就用當下 odom -> base_footprint 的位姿統計：
  in_fp    footprint 多邊形內 cost >= 99 (OccupancyGrid：100 致命 / 99 內切) 的格數
  engine   引擎方塊投影 (vehicle.yaml engine_*) 內 cost >= 99 的格數
  front    車頭前方 0 ~ 0.5 m、車寬範圍內 cost >= 99 的格數
引擎在 footprint 裡面。obstacle_layer 的 footprint_clearing 會把 footprint 內的格子清掉，
所以 in_fp / engine 應該是 0；不是 0 就代表光達打到引擎的點被標成障礙物而且沒被清掉。
"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from mowerbot_description import vehicle_geometry

SECS = float(sys.argv[1]); OUT = sys.argv[2]
g = vehicle_geometry.load()
rclpy.init(); n = Node('costmap_probe')
buf = tf2_ros.Buffer(); tl = tf2_ros.TransformListener(buf, n)
rows = []


def on_cm(m):
    try:
        tr = buf.lookup_transform(m.header.frame_id, 'base_footprint', Time())
    except Exception:
        return
    q = tr.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    bx, by = tr.transform.translation.x, tr.transform.translation.y
    c, s = math.cos(-yaw), math.sin(-yaw)
    info = m.info
    cnt = {'in_fp': 0, 'engine': 0, 'front': 0}
    ex0 = g.engine_center_x - g.engine_length / 2; ex1 = g.engine_center_x + g.engine_length / 2
    for j in range(info.height):
        for i in range(info.width):
            v = m.data[j * info.width + i]
            if v < 99:
                continue
            wx = info.origin.position.x + (i + 0.5) * info.resolution - bx
            wy = info.origin.position.y + (j + 0.5) * info.resolution - by
            lx, ly = c * wx - s * wy, s * wx + c * wy
            if abs(ly) <= g.lateral_half_extent:
                if -g.rear_extent <= lx <= g.front_extent:
                    cnt['in_fp'] += 1
                    if ex0 <= lx <= ex1 and abs(ly) <= g.engine_width / 2:
                        cnt['engine'] += 1
                elif g.front_extent < lx <= g.front_extent + 0.5:
                    cnt['front'] += 1
    rows.append((time.time(), bx, by, yaw, cnt['in_fp'], cnt['engine'], cnt['front']))


n.create_subscription(OccupancyGrid, '/local_costmap/costmap', on_cm, 5)
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(n, timeout_sec=0.2)
with open(OUT, 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['t', 'x', 'y', 'yaw', 'in_fp', 'engine', 'front'])
    w.writerows(rows)
if rows:
    print('costmap %d 張；footprint 內 >=99 的格數 最大 %d (非零 %d 張)；引擎投影內 最大 %d；'
          '車頭前 0.5 m 最大 %d (非零 %d 張)'
          % (len(rows), max(r[4] for r in rows), sum(1 for r in rows if r[4]),
             max(r[5] for r in rows), max(r[6] for r in rows), sum(1 for r in rows if r[6])))
else:
    print('沒有收到 /local_costmap/costmap')
