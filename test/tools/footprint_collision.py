# -*- coding: utf-8 -*-
"""全程碰撞檢查 (階段 38 後續 6)：車身四角 footprint 對世界檔裡每一個方塊 (牆、障礙物) 的幾何距離。

用法：python3 test/tools/footprint_collision.py <run 目錄> [--world <world 檔>]
      (需要 run_traj.csv；座標系與 coverage_budget.py 相同：軌跡 x, y 直接對世界檔座標)

兩種量法一起報：
  A. 幾何 (直接)：每一筆軌跡取樣，footprint 多邊形 (vehicle_geometry.footprint，base_link 座標)
     轉到 (x, y, yaw) 後，與每個方塊 (含自身 yaw) 的最小距離；相交記為 0 並計入「重疊取樣」。
  B. 頂住 (間接，與 38.14 的定義相同)：cmd_vx > 0.05 而 |odom_vx| < 0.02 連續超過 2 s。
判定「無碰撞」= A 的重疊取樣 0 筆 而且 B 的事件 0 次。
--min-clearance <m> (預設 0.15)：驗收第 3 項 (階段 38 後續 6 改訂)「車身四角到任何牆面 / 障礙物的最小幾何距離 >= 門檻」。
  另外列出：每一筆取樣的最近距離分布 (第 1 / 5 百分位 = 99 % / 95 % 的時間比它遠)、每一段的最小距離、
  低於門檻 / 低於 0.20 m 的段數與時間 —— 用來判斷最小值是個案還是常態。
"""
import argparse, csv, math, os, sys

from mowerbot_description import vehicle_geometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_budget import world_boxes  # noqa: E402

STUCK_S = 2.0


def rect(cx, cy, yaw, sx, sy):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]


def _axes(poly):
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
        yield (-(y2 - y1), x2 - x1)


def overlap(a, b):
    """兩個凸多邊形是否相交 (分離軸定理)。"""
    for ax, ay in list(_axes(a)) + list(_axes(b)):
        pa = [x * ax + y * ay for x, y in a]
        pb = [x * ax + y * ay for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return False
    return True


def _pt_seg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - x1 - t * dx, py - y1 - t * dy)


def dist(a, b):
    if overlap(a, b):
        return 0.0
    d = float('inf')
    for P, Q in ((a, b), (b, a)):
        for px, py in P:
            for i in range(len(Q)):
                d = min(d, _pt_seg(px, py, *Q[i], *Q[(i + 1) % len(Q)]))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--world', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..',
        'src/mowerbot_bringup/worlds/demo_lawn.world'))
    ap.add_argument('--min-clearance', type=float, default=0.15)
    a = ap.parse_args()
    g = vehicle_geometry.load()
    fp = [tuple(c) for c in g.footprint]
    boxes = [(n, rect(x, y, yaw, sx, sy)) for n, x, y, yaw, sx, sy in world_boxes(a.world)
             if not n.startswith('ground') and sx * sy < 400]
    rows = [r for r in csv.DictReader(l for l in open(os.path.join(a.run, 'run_traj.csv'))
                                      if not l.startswith('#'))]
    mind, worst, hits, hit_ev, cur = float('inf'), None, 0, [], None
    stuck, st0 = [], None
    dists, seg_min, below = [], {}, {}
    prev_t = None
    for r in rows:
        x, y, yaw, t = float(r['x']), float(r['y']), float(r['yaw']), float(r['t'])
        c, s = math.cos(yaw), math.sin(yaw)
        poly = [(x + c * px - s * py, y + s * px + c * py) for px, py in fp]
        best = min(((dist(poly, b), n) for n, b in boxes), default=(float('inf'), '-'))
        lab = r['label'] or '(無標籤)'
        dists.append(best[0])
        if lab not in seg_min or best[0] < seg_min[lab][0]:
            seg_min[lab] = (best[0], best[1], x, y)
        if prev_t is not None and best[0] < 0.20:
            below.setdefault(lab, [0.0, 0.0])
            below[lab][1] += t - prev_t
            if best[0] < a.min_clearance:
                below[lab][0] += t - prev_t
        prev_t = t
        if best[0] < mind:
            mind, worst = best[0], (best[1], r['label'], x, y, t)
        if best[0] == 0.0:
            hits += 1
            if cur is None:
                cur = [best[1], r['label'] or '(無標籤)', t, t]
                hit_ev.append(cur)
            cur[3] = t
        else:
            cur = None
        if float(r['cmd_vx']) > 0.05 and abs(float(r['odom_vx'])) < 0.02:
            if st0 is None:
                st0 = (t, r['label'], x, y)
            elif t - st0[0] > STUCK_S and (not stuck or stuck[-1][0] != st0[0]):
                stuck.append(st0)
        else:
            st0 = None
    print('碰撞檢查：%d 筆取樣、%d 個方塊 (%s)' % (len(rows), len(boxes), os.path.basename(a.world)))
    if worst:
        print('A. footprint 與方塊的最小距離 %.3f m（%s，段 %s，位置 (%.2f, %.2f)）'
              % (mind, worst[0], worst[1] or '(無標籤)', worst[2], worst[3]))
    print('A. 重疊取樣 %d 筆，合併成 %d 次事件' % (hits, len(hit_ev)))
    for n, l, t0, t1 in hit_ev[:20]:
        print('     %s  段 %s  %.1f s' % (n, l, t1 - t0))
    print('B. 頂住 (前進指令下 |odom_vx| < 0.02 超過 %.0f s)：%d 次' % (STUCK_S, len(stuck)))
    for t0, l, x, y in stuck[:20]:
        print('     段 %s  (%.2f, %.2f)' % (l or '(無標籤)', x, y))
    ds = sorted(dists)
    q = lambda f: ds[min(len(ds) - 1, int(f * len(ds)))]
    print('每筆取樣的最近距離：第 1 百分位 %.3f m、第 5 百分位 %.3f m、中位 %.3f m'
          '（= 99 %% / 95 %% 的時間比這個值遠）' % (q(0.01), q(0.05), q(0.5)))
    segs = sorted(seg_min.items(), key=lambda kv: kv[1][0])
    sm = sorted(v[0] for _, v in segs)
    print('每一段的最小距離 (%d 段)：最小 %.3f、第 5 百分位 %.3f、中位 %.3f m'
          % (len(sm), sm[0], sm[min(len(sm) - 1, int(0.05 * len(sm)))], sm[len(sm) // 2]))
    n15 = [l for l, v in segs if v[0] < a.min_clearance]
    n20 = [l for l, v in segs if v[0] < 0.20]
    print('最小距離 < %.2f m 的段：%d 段；< 0.20 m：%d 段' % (a.min_clearance, len(n15), len(n20)))
    for l, (d0, box, x, y) in segs[:8]:
        bt = below.get(l, [0.0, 0.0])
        print('     %.3f m  %-22s 對 %-16s (%.2f, %.2f)  < %.2f m 共 %.1f s、< 0.20 m 共 %.1f s'
              % (d0, l, box, x, y, a.min_clearance, bt[0], bt[1]))
    print('CLEARANCE_OK=%s (最小 %.3f m，門檻 %.2f m)' % ('yes' if mind >= a.min_clearance else 'no', mind, a.min_clearance))
    print('COLLISION_FREE=%s' % ('yes' if hits == 0 and not stuck else 'no'))


if __name__ == '__main__':
    main()
