# -*- coding: utf-8 -*-
"""blade_width 敏感度的「規劃面」掃描 (階段 38 修正 1 F 節)：不開車，只叫 F2C。

用法：python3 test/tools/blade_sweep_plan.py <輸出 csv> [0.40 0.60 0.01]
前提：map_server (存檔地圖)、map_to_boundary、f2c_server 已經起來
(test/tools/stage38_blade_sweep.sh 會幫忙起)。

對每一個 blade_width：tool_width = blade_width x (1 - DEFAULT_OVERLAP_RATIO)，與 manager 相同；
割草線條數用 manager 自己的切段規則 (MowerManager.cut_on_direction_change) 數；
規劃涵蓋面積 = 整條 F2C 路徑以刀盤寬光柵化 ∩ 草坪 (世界檔幾何，與 coverage_budget 同一套)。
這一項只由幾何與計畫決定，不受執行 (會不會撞牆) 影響。
"""
import csv, math, os, sys, time, types
import numpy as np, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PolygonStamped
from mowerbot_interfaces.msg import ObstaclePolygons
from mowerbot_interfaces.srv import GenerateCoveragePath
from mowerbot_action.manager import MowerManager, DEFAULT_OVERLAP_RATIO
from mowerbot_description import vehicle_geometry
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_budget import world_boxes

OUT = sys.argv[1]
lo, hi, step = (float(v) for v in (sys.argv[2:5] if len(sys.argv) >= 5 else ('0.40', '0.60', '0.01')))
WORLD = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                     'src/mowerbot_bringup/worlds/demo_lawn.world')
CELL = 0.02
g = vehicle_geometry.load()

# ---- 草坪 (與 coverage_budget 相同：世界檔 box 光柵化，取含原點的連通自由區) ----
boxes = world_boxes(WORLD)
x0 = min(b[1] for b in boxes) - 1.0; y0 = min(b[2] for b in boxes) - 1.0
x1 = max(b[1] for b in boxes) + 1.0; y1 = max(b[2] for b in boxes) + 1.0
W = int(round((x1 - x0) / CELL)); H = int(round((y1 - y0) / CELL))
px = lambda x, y: (int(round((x - x0) / CELL)), int(round((y - y0) / CELL)))
occ = np.zeros((H, W), np.uint8)
for _n, cx, cy, yaw, sx, sy in boxes:
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [px(cx + c * dx - s * dy, cy + s * dx + c * dy)
           for dx, dy in ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
    cv2.fillConvexPoly(occ, np.array(pts, np.int32), 1)
_n, lab = cv2.connectedComponents((occ == 0).astype(np.uint8), 4)
lawn = lab == lab[px(0, 0)[1], px(0, 0)[0]]
A = lawn.sum() * CELL * CELL

rclpy.init(); n = Node('blade_sweep_plan')
st = {'b': None, 'o': []}
n.create_subscription(PolygonStamped, '/f2c_boundary', lambda m: st.__setitem__('b', m.polygon), 10)
n.create_subscription(ObstaclePolygons, '/f2c_obstacles',
                      lambda m: st.__setitem__('o', list(m.polygons)), 10)
cli = n.create_client(GenerateCoveragePath, 'generate_coverage_path')
print('READY', flush=True)          # stage38_blade_sweep.sh 等到這一行才起 map_to_boundary
t0 = time.time()
while (st['b'] is None or not cli.service_is_ready()) and time.time() - t0 < 90:
    rclpy.spin_once(n, timeout_sec=0.2)
if st['b'] is None:
    sys.exit('90 秒內沒收到 /f2c_boundary 或 F2C 服務')
for _ in range(10):
    rclpy.spin_once(n, timeout_sec=0.2)
fake = types.SimpleNamespace(GAP_CUT_DISTANCE=MowerManager.GAP_CUT_DISTANCE)
print(g.stamp() or '(全部 measured)')
print('邊界 %d 頂點，內部障礙物 %d 個，草坪 A = %.2f m²，headland %.2f，重疊率 %.2f'
      % (len(st['b'].points), len(st['o']), A, g.headland_width, DEFAULT_OVERLAP_RATIO))
rows = []
k = 0
while lo + k * step <= hi + 1e-9:
    bw = round(lo + k * step, 4); k += 1
    req = GenerateCoveragePath.Request()
    req.boundary = st['b']; req.obstacles = st['o']
    req.tool_width = bw * (1.0 - DEFAULT_OVERLAP_RATIO); req.turning_radius = 1.0
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    r = f.result()
    if r is None or not r.success:
        print('bw=%.2f F2C 失敗' % bw); continue
    poses = list(r.coverage_path.poses)
    # 周邊環繞 = 開頭到「第一個回到第 0 點」為止 (與 manager / coverage_budget 相同)
    p0 = poses[0].pose.position; end = 0
    for i in range(3, len(poses)):
        q = poses[i].pose.position
        if math.hypot(q.x - p0.x, q.y - p0.y) <= MowerManager.PERIMETER_CLOSE_TOL:
            end = i; break
    swaths = MowerManager.cut_on_direction_change(fake, poses[end + 1:])
    m = np.zeros((H, W), np.uint8)
    piece = []
    for p in poses:
        q = (p.pose.position.x, p.pose.position.y)
        if piece and math.hypot(q[0] - piece[-1][0], q[1] - piece[-1][1]) > 0.5:
            cv2.polylines(m, [np.array([px(*a) for a in piece], np.int32)], False, 1,
                          max(1, int(round(bw / CELL))))
            piece = []
        piece.append(q)
    if len(piece) > 1:
        cv2.polylines(m, [np.array([px(*a) for a in piece], np.int32)], False, 1,
                      max(1, int(round(bw / CELL))))
    planned = (m.astype(bool) & lawn).sum() * CELL * CELL
    rows.append((bw, req.tool_width, len(swaths), planned, 100 * planned / A))
    print('blade_width %.2f  間距 %.3f  割草線 %2d 條  規劃涵蓋 %6.2f m² = %5.2f %% of A'
          % rows[-1])
with open(OUT, 'w', newline='') as fh:
    if g.stamp():
        fh.write(''.join('# %s\n' % l for l in g.stamp().splitlines()))
    w = csv.writer(fh); w.writerow(['blade_width', 'swath_spacing', 'n_swaths', 'planned_m2', 'planned_pct_A'])
    w.writerows(rows)
