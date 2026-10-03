# -*- coding: utf-8 -*-
"""光達有效視野 (階段 38)：統計 /scan 每一條射線是否被車體自己擋住。

用法 (Gazebo 起來、車子停在開闊處)：python3 test/tools/scan_fov.py [秒數]
判定：射線距離 < SELF_RANGE (預設 1.2 m，大於車體最遠點 1.06 m) 且連續 N 幀都如此，
視為打到車體自己 (引擎 / 車架)。印出被遮擋的角度範圍 (光達座標，0 = 車頭正前方，逆時針為正)。
"""
import math, sys, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
SELF_RANGE = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2
rclpy.init(); n = Node('scan_fov')
scans = []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), 10)
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(n, timeout_sec=0.1)
if not scans:
    print('NO_SCAN'); sys.exit(1)
m = scans[-1]
N = len(m.ranges)
blocked = [all((not math.isinf(s.ranges[i])) and s.ranges[i] < SELF_RANGE for s in scans[-5:])
           for i in range(N)]
ang = [math.degrees(m.angle_min + i * m.angle_increment) for i in range(N)]
runs, cur = [], None
for i in range(N):
    if blocked[i] and cur is None:
        cur = i
    if (not blocked[i] or i == N - 1) and cur is not None:
        end = i if blocked[i] else i - 1
        runs.append((cur, end)); cur = None
nb = sum(blocked)
print('rays=%d inc=%.3f deg  scans=%d  self-hit (<%.2f m, 5 幀都是) = %d 條 = %.1f deg' % (
    N, math.degrees(m.angle_increment), len(scans), SELF_RANGE, nb, 360.0 * nb / N))
print('有效視野 = %.1f deg' % (360.0 * (N - nb) / N))
for a, b in runs:
    rr = [m.ranges[i] for i in range(a, b + 1)]
    print('  遮擋 %.1f ~ %.1f deg (%d 條)，距離 %.2f ~ %.2f m' % (ang[a], ang[b], b - a + 1, min(rr), max(rr)))
