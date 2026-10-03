# -*- coding: utf-8 -*-
"""前 2 m 車頭角偏移的拆解 (階段 38 後續 6)：偏航誤差的貢獻 vs base_link 自身側向偏移的貢獻。

用法：python3 test/tools/first2m_decompose.py <run 目錄>
取樣條件與 corner_offset.py 的【前 2 m】相同 (割草線、0 <= s <= 2、odom_vx > 0.1)。
對每一段割草線取前 2 m 內：
  實際 = 四角最大側向偏移 (corner_offset.py 的量)
  零偏航下限 = |base 側向偏移| + lateral_half_extent   (假設偏航誤差 = 0 時仍然會有的偏移)
  判準 = lateral_half_extent + (headland - rotation_swept_radius) = 0.558
若「零偏航下限」本身就超過判準，代表任何 yaw 容忍值都救不了那一段。
"""
import csv, math, re, sys
from mowerbot_description import vehicle_geometry
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from corner_offset import to_map_frame  # noqa: E402
g = vehicle_geometry.load()
crit = g.lateral_half_extent + (g.headland_width - g.rotation_swept_radius)
fp = [tuple(c) for c in g.footprint]
d = sys.argv[1]
RE = re.compile(r'📋 佇列 (\d+)/(\d+) \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) 終點 \(([-\d.]+), ([-\d.]+)\)')
q = {}
for l in open(d + '/manager.log', encoding='utf-8', errors='replace'):
    m = RE.search(l)
    if m:
        if int(m.group(1)) == 1:
            q = {}
        q[m.group(3)] = tuple(float(m.group(k)) for k in range(4, 8))
rows = list(csv.DictReader(l for l in open(d + '/run_traj.csv') if not l.startswith('#')))
if '--frame' not in sys.argv or sys.argv[sys.argv.index('--frame') + 1] != 'odom':
    print('(map 座標，量測方式修正：map->odom 中位 %.3f m)' % to_map_frame(d, rows)[1])
else:
    print('(odom 座標，舊量法)')
per = {}
for r in rows:
    lb = r['label']
    if not lb.startswith('割草線') or lb not in q or float(r['odom_vx']) <= 0.1:
        continue
    sx, sy, ex, ey = q[lb]; L = math.hypot(ex - sx, ey - sy)
    if L < 1e-6:
        continue
    ux, uy = (ex - sx) / L, (ey - sy) / L
    x, y, yaw = float(r['x']), float(r['y']), float(r['yaw'])
    s = (x - sx) * ux + (y - sy) * uy
    if not 0 <= s <= 2.0:
        continue
    lat = -(x - sx) * uy + (y - sy) * ux
    th = math.atan2(math.sin(yaw - math.atan2(uy, ux)), math.cos(yaw - math.atan2(uy, ux)))
    c, sn = math.cos(th), math.sin(th)
    cm = max(abs(lat + sn * cx + c * cy) for cx, cy in fp)
    a = per.setdefault(lb, [0.0, 0.0, 0.0])
    a[0] = max(a[0], cm); a[1] = max(a[1], abs(lat) + g.lateral_half_extent); a[2] = max(a[2], abs(math.degrees(th)))
n = len(per)
floor_over = sum(1 for v in per.values() if v[1] >= crit)
act_over = sum(1 for v in per.values() if v[0] >= crit)
print('割草線 %d 段 (前 2 m 有取樣)，判準 %.3f m' % (n, crit))
print('  實際超過判準        ：%d / %d 段' % (act_over, n))
print('  零偏航下限就超過判準：%d / %d 段  (這些段任何 yaw 容忍值都救不了)' % (floor_over, n))
fl = sorted(v[1] for v in per.values())
print('  零偏航下限：最小 %.3f  中位 %.3f  最大 %.3f m' % (fl[0], fl[len(fl) // 2], fl[-1]))
print('  前 2 m 最大偏航：中位 %.1f°  最大 %.1f°' % (sorted(v[2] for v in per.values())[n // 2], max(v[2] for v in per.values())))
