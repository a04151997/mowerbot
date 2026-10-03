# -*- coding: utf-8 -*-
"""周邊環繞「實際會開的長度」(階段 38，smoke_test B5 用)。不需要 Gazebo、不需要 Nav2。

前提：map_server (存檔地圖)、map_to_boundary、f2c_server 已經起來 (perimeter_probe.sh 會起)。
做法：收 /f2c_boundary -> 呼叫 F2C (與 manager 相同的 tool_width) -> 用 mower_manager 自己的
split_off_perimeter() / split_perimeter_into_edges() 切段 (建一個 MowerManager 但不 spin，不會送速度)。

有效環繞長度 = 各段路徑長度的總和，但「起點與終點距離 <= xy_goal_tolerance」的段落算 0：
goal checker 在送出的當下就判定到達，那一段實際上一公尺都不會開 (階段 38 的「0 秒完成」)。
輸出 KEY=VALUE，由 smoke_test B5 判定。
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from mowerbot_interfaces.msg import ObstaclePolygons
from mowerbot_interfaces.srv import GenerateCoveragePath
from mowerbot_action.manager import MowerManager, DEFAULT_OVERLAP_RATIO
from mowerbot_description import vehicle_geometry
import yaml, os
from ament_index_python.packages import get_package_share_directory

g = vehicle_geometry.load()
with open(os.path.join(get_package_share_directory('mowerbot_bringup'), 'config', 'nav2_params.yaml')) as fh:
    XY_TOL = float(yaml.safe_load(fh)['controller_server']['ros__parameters']
                   ['general_goal_checker']['xy_goal_tolerance'])
rclpy.init(args=['--ros-args'] + [a for k in ('lateral_half_extent', 'rotation_swept_radius',
                                               'soft_inflation_radius', 'blade_width',
                                               'perimeter_corner_window', 'perimeter_corner_angle')
                                  for a in ('-p', '%s:=%r' % (k, getattr(g, k)))]
          + ['-p', 'goal_xy_tolerance:=%r' % XY_TOL])
n = Node('perimeter_probe')
st = {'b': None, 'o': []}
n.create_subscription(PolygonStamped, '/f2c_boundary', lambda m: st.__setitem__('b', m.polygon), 10)
n.create_subscription(ObstaclePolygons, '/f2c_obstacles', lambda m: st.__setitem__('o', list(m.polygons)), 10)
cli = n.create_client(GenerateCoveragePath, 'generate_coverage_path')
print('READY', flush=True)          # perimeter_probe.sh 等到這一行才起 map_to_boundary (訂閱要先到)
t0 = time.time()
while (st['b'] is None or not cli.service_is_ready()) and time.time() - t0 < 90:
    rclpy.spin_once(n, timeout_sec=0.2)
if st['b'] is None:
    print('ERROR=90 秒內沒收到 /f2c_boundary 或 F2C 服務'); sys.exit(1)
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.2)
req = GenerateCoveragePath.Request()
req.boundary = st['b']; req.obstacles = st['o']
req.tool_width = g.blade_width * (1.0 - DEFAULT_OVERLAP_RATIO); req.turning_radius = 1.0
f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
r = f.result()
if r is None or not r.success:
    print('ERROR=F2C 失敗'); sys.exit(1)
mgr = MowerManager()          # 不 spin：只借用它的切段函式
per, _rest = mgr.split_off_perimeter(r.coverage_path)
edges = mgr.split_perimeter_into_edges(per, r.coverage_path.header)


def plen(ps):
    return sum(math.hypot(ps[k + 1].pose.position.x - ps[k].pose.position.x,
                          ps[k + 1].pose.position.y - ps[k].pose.position.y) for k in range(len(ps) - 1))


bp = [(p.x, p.y) for p in st['b'].points]
b_perim = sum(math.hypot(bp[(i + 1) % len(bp)][0] - bp[i][0], bp[(i + 1) % len(bp)][1] - bp[i][1])
              for i in range(len(bp)))
planned = plen(per)
eff = 0.0
closed = 0
for e in edges:
    a, b = e.poses[0].pose.position, e.poses[-1].pose.position
    if math.hypot(b.x - a.x, b.y - a.y) <= XY_TOL:
        closed += 1          # 起點 = 終點：送出當下就「到達」，實際不會開
    else:
        eff += plen(e.poses)
# 環繞的原始航點 (F2C 輸出、切段前)，給 perimeter_corner_sweep.py 離線掃門檻用
import os as _os
with open(_os.path.join(_os.environ.get('PERIMETER_DUMP_DIR', '/tmp'), 'perimeter_xy.csv'), 'w') as fh:
    fh.write('x,y\n')
    for p in per:
        fh.write('%.4f,%.4f\n' % (p.pose.position.x, p.pose.position.y))
print('BOUNDARY_PERIMETER=%.2f' % b_perim)
print('PLANNED_PERIMETER=%.2f' % planned)
print('EDGES=%d' % len(edges))
print('CLOSED_EDGES=%d' % closed)
print('EFFECTIVE_PERIMETER=%.2f' % eff)
print('XY_TOL=%.2f' % XY_TOL)
