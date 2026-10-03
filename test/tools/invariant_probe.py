# -*- coding: utf-8 -*-
"""周邊環繞兩條正確性不變量的確定性檢查 (階段 38 決定 3-A，smoke_test B6 / B7 用)。

不需要 Gazebo、不需要 Nav2：建一個 mower_manager 節點 (不 spin，不會送任何速度)，直接呼叫它的函式。

B6 【不變量 ①】閉合迴圈不得整段送出
   輸入：半徑 3 m 的圓形環繞 (首尾相同)。曲率 1/3 m⁻¹，一個車身長 1.13 m 內只轉 21.6°，
         轉角偵測一刀都切不到 —— 正是「整圈一段、起點 = 終點」的情境。
   正常：split_perimeter_into_edges() 切出的每一段起點 ≠ 終點。
   對照 (植入違規)：把 goal_xy_tolerance 設成 −1 (等於關掉不變量 ①)，必須切出一段起點 = 終點。
B7 【不變量 ②】起點 = 終點的段落不得送出
   輸入：佇列 [正常直線段, 起點 = 終點的段落]。
   正常：drop_closed_segments() 只留下正常那一段。
   對照：goal_xy_tolerance = −1 時兩段都留下 (違規段通過)。
對照組必須「抓到違規」，正常組必須「乾淨」，兩者同時成立才算 PASS ——
否則一個永遠回傳乾淨的實作也會讓測試變綠。輸出 KEY=VALUE。
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mowerbot_action.manager import MowerManager
from mowerbot_description import vehicle_geometry

g = vehicle_geometry.load()
with open(os.path.join(get_package_share_directory('mowerbot_bringup'), 'config',
                       'nav2_params.yaml')) as fh:
    XY_TOL = float(yaml.safe_load(fh)['controller_server']['ros__parameters']
                   ['general_goal_checker']['xy_goal_tolerance'])
rclpy.init(args=['--ros-args'] + [
    a for k in ('lateral_half_extent', 'rotation_swept_radius', 'soft_inflation_radius',
                'blade_width', 'perimeter_corner_window', 'perimeter_corner_angle')
    for a in ('-p', '%s:=%r' % (k, getattr(g, k)))] + ['-p', 'goal_xy_tolerance:=%r' % XY_TOL])
mgr = MowerManager()


def pose(x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.w = 1.0
    return p


def closed_count(edges):
    n = 0
    for e in edges:
        a, b = e.poses[0].pose.position, e.poses[-1].pose.position
        if math.hypot(b.x - a.x, b.y - a.y) <= XY_TOL:
            n += 1
    return n


# ---- B6 ----
R = 3.0
N = int(2 * math.pi * R / 0.1)
circle = [pose(R * math.cos(2 * math.pi * k / N), R * math.sin(2 * math.pi * k / N)) for k in range(N)]
circle.append(pose(circle[0].pose.position.x, circle[0].pose.position.y))
hdr = Path().header
hdr.frame_id = 'map'
edges = mgr.split_perimeter_into_edges(circle, hdr)
print('B6_EDGES=%d' % len(edges))
print('B6_CLOSED=%d' % closed_count(edges))
mgr.goal_xy_tolerance = -1.0                    # 植入違規：關掉不變量 ①
edges_bad = mgr.split_perimeter_into_edges(circle, hdr)
print('B6_CONTROL_EDGES=%d' % len(edges_bad))
print('B6_CONTROL_CLOSED=%d' % closed_count(edges_bad))
mgr.goal_xy_tolerance = XY_TOL

# ---- B7 ----
line = Path()
line.header.frame_id = 'map'
line.poses = [pose(0.1 * k, 0.0) for k in range(30)]
loop = Path()
loop.header.frame_id = 'map'
loop.poses = [pose(math.cos(t) - 1.0, math.sin(t)) for t in
              [2 * math.pi * k / 40 for k in range(41)]]
queue = [(line, '割草線 1/2'), (loop, '割草線 2/2 (起點 = 終點)')]
kept = mgr.drop_closed_segments(queue)
print('B7_KEPT=%s' % ','.join(l for _p, l in kept))
mgr.goal_xy_tolerance = -1.0                    # 植入違規：關掉不變量 ②
kept_bad = mgr.drop_closed_segments(queue)
print('B7_CONTROL_KEPT=%s' % ','.join(l for _p, l in kept_bad))
mgr.goal_xy_tolerance = XY_TOL
print('XY_TOL=%.2f' % XY_TOL)
mgr.destroy_node()
rclpy.shutdown()
