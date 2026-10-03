# -*- coding: utf-8 -*-
"""原地旋轉的死區與「指令 vs 實際」曲線 (階段 38 決定 1-D)。繞過 DWB，直接對 /cmd_vel 下固定 wz。

用法 (只要 Gazebo + 車子，不需要 Nav2)：python3 spin_deadband.py <輸出 csv>
每一級：先送零 1.5 s 讓車子完全停下 (量的是「從靜止起轉」)，再送固定 wz 3 s，
取後 2 s 的 odom 平均角速度 (前 1 s 是加速段)。wz = ±0.02, ±0.04, ... ±0.40。
同時記錄旋轉期間後輪軸的平移 (理想原地旋轉是 0)。
"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

OUT = sys.argv[1]
# --prime：每個方向開始前先用同方向 0.40 rad/s 轉 3 s，讓腳輪先轉到該方向的拖曳位置
#          (用來分辨「死區」是低速本身，還是腳輪換向卡住)
PRIME = '--prime' in sys.argv
rclpy.init(); n = Node('spin_deadband')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
st = {'o': None, 'tr': []}


def on_odom(m):
    q = m.pose.pose.orientation
    o = (time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    st['o'] = o
    st['tr'].append(o)


n.create_subscription(Odometry, '/odom', on_odom, 50)
t0 = time.time()
while st['o'] is None and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.1)


def hold(wz, sec):
    m = Twist(); m.angular.z = float(wz)
    t = time.time()
    while time.time() - t < sec:
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)


rows = []
for sign in (1, -1):
    if PRIME:
        hold(sign * 0.40, 3.0)
    for k in range(1, 21):
        wz = sign * 0.02 * k
        hold(0.0, 1.5)
        if PRIME:
            hold(sign * 0.40, 1.0)      # 每一級前都重新把腳輪轉到同方向
            hold(0.0, 1.5)
        st['tr'] = []
        hold(wz, 3.0)
        tr = [o for o in st['tr'] if o[0] >= st['tr'][0][0] + 1.0]
        acc = sum(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])) for a, b in zip(tr, tr[1:]))
        rate = acc / max(tr[-1][0] - tr[0][0], 1e-6)
        move = max(math.hypot(o[1] - st['tr'][0][1], o[2] - st['tr'][0][2]) for o in st['tr'])
        rows.append((round(wz, 2), round(rate, 4), round(rate / wz, 3), round(move, 4)))
        print('cmd wz %+.2f -> 實際 %+.4f rad/s (比值 %.2f)，平移 %.3f m' % rows[-1], flush=True)
hold(0.0, 1.0)
with open(OUT, 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['cmd_wz', 'actual_wz', 'ratio', 'translation_m']); w.writerows(rows)
