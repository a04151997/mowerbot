# -*- coding: utf-8 -*-
"""車身四角相對割草線中心線的側向偏移、直線段偏航誤差 (階段 38 修正 3 第 6 部分)。

用法：python3 test/tools/corner_offset.py <run 目錄> [--frame map|odom]   (需要 run_traj.csv 與 manager.log)
  預設 --frame map：先用 run_mapodom.csv 把軌跡轉到 map 座標 (量測方式修正，階段 38 後續 6，見 to_map_frame)。

base_link 在後輪軸，車頭在前方 front_extent。偏航 θ 時車頭角的側向位置
  = 側向半寬 x cos θ + front_extent x sin θ   (θ=10° 時 0.583 m，舊車只有 0.417)
所以「車身會不會超出 0.420」不能只看 base_link 的循跡誤差，要直接算四個角。

取樣點的篩選 (「直線段」)：
  * label 是割草線，而且該段在 manager.log 的最後一輪佇列裡找得到起訖點
  * base_footprint 投影落在 [0, 段長] 之內 (跑道與掉頭不算)
  * odom_vx > 0.1 m/s (原地旋轉不算直線段)
對每個取樣點：
  偏航誤差 = yaw - 割草線方向 (wrap 到 ±180°)
  四角側向偏移 = |base 的側向偏差 + 角點在車體座標的 (x, y) 轉到割草線法向的分量|，取四角最大
超過 lateral_half_extent 的取樣點依「同一段、連續」合併成事件，列出段名。
"""
import csv, math, os, re, sys
# 階段 38 決定 1-C：另外統計「割草線前 FIRST_M 公尺」(對正完成後、剛起步那一段)，
# 判準 = lateral_half_extent + (headland_width − rotation_swept_radius) —— 車頭角不得吃掉整個轉彎餘裕。
FIRST_M = 2.0

from mowerbot_description import vehicle_geometry

RE_QUEUE = re.compile(
    r'📋 佇列 (\d+)/(\d+) \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) '
    r'終點 \(([-\d.]+), ([-\d.]+)\)')


def to_map_frame(run, rows):
    """量測方式修正 (階段 38 後續 6)：run_traj.csv 的 x / y / yaw 是 /odom 座標，割草線 (manager 佇列) 是 map 座標。
    兩者差一個 map -> odom (實測中位 0.29 m)，不轉的話定位偏移會被算成「側向偏移」。
    用 run_mapodom.csv (每秒一筆，同一個 time.time() 時鐘) 取時間上最接近的一筆轉過去，就地改寫 rows。
    回傳 (有沒有轉, 用到的 map->odom 平移中位數)。沒有 run_mapodom.csv 時不轉 (舊資料)。"""
    import bisect
    f = os.path.join(run, 'run_mapodom.csv')
    if not os.path.exists(f):
        return False, None
    mo = [(float(r['t']), float(r['x']), float(r['y']), float(r['yaw']))
          for r in csv.DictReader(l for l in open(f) if not l.startswith('#'))]
    if not mo:
        return False, None
    ts = [m[0] for m in mo]
    for r in rows:
        t = float(r['t'])
        i = bisect.bisect_left(ts, t)
        if i == len(ts) or (i > 0 and t - ts[i - 1] < ts[i] - t):
            i -= 1
        _, tx, ty, tyaw = mo[i]
        x, y, yaw = float(r['x']), float(r['y']), float(r['yaw'])
        c, s = math.cos(tyaw), math.sin(tyaw)
        r['x'], r['y'] = repr(tx + c * x - s * y), repr(ty + s * x + c * y)
        r['yaw'] = repr(yaw + tyaw)
    mags = sorted(math.hypot(m[1], m[2]) for m in mo)
    return True, mags[len(mags) // 2]


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(math.ceil(q * len(v))) - 1)] if v else float('nan')


def main():
    run = sys.argv[1]
    g = vehicle_geometry.load()
    corners = [tuple(c) for c in g.footprint]
    limit = g.lateral_half_extent
    queue = {}
    for line in open(os.path.join(run, 'manager.log'), encoding='utf-8', errors='replace'):
        m = RE_QUEUE.search(line)
        if m:
            if int(m.group(1)) == 1:
                queue = {}
            queue[m.group(3)] = tuple(float(m.group(k)) for k in range(4, 8))
    rows = list(csv.DictReader(l for l in open(os.path.join(run, 'run_traj.csv'))
                               if not l.startswith('#')))
    frame = 'odom' if '--frame' in sys.argv and sys.argv[sys.argv.index('--frame') + 1] == 'odom' else 'map'
    if frame == 'map':
        conv, med = to_map_frame(run, rows)
        print('量測方式修正：軌跡先用 map->odom 轉到 map 座標再對割草線 (map->odom 平移中位 %.3f m)' % med
              if conv else '（沒有 run_mapodom.csv：軌跡維持 odom 座標，與割草線的 map 座標差一個定位偏移）')
    else:
        print('（--frame odom：舊量法，軌跡 odom 座標直接對 map 座標的割草線）')
    first_corner = []
    first_events, fcur = [], None
    yaw_err, corner_max, base_lat = [], [], []
    events, cur = [], None
    for r in rows:
        lbl = r['label']
        if not lbl.startswith('割草線') or lbl not in queue or float(r['odom_vx']) <= 0.1:
            cur = None
            continue
        sx, sy, ex, ey = queue[lbl]
        L = math.hypot(ex - sx, ey - sy)
        if L < 1e-6:
            continue
        ux, uy = (ex - sx) / L, (ey - sy) / L
        x, y, yaw = float(r['x']), float(r['y']), float(r['yaw'])
        s = (x - sx) * ux + (y - sy) * uy
        if s < 0.0 or s > L:
            cur = None
            continue
        lat = -(x - sx) * uy + (y - sy) * ux          # 左正
        th = math.atan2(math.sin(yaw - math.atan2(uy, ux)), math.cos(yaw - math.atan2(uy, ux)))
        c, sn = math.cos(th), math.sin(th)
        cm = max(abs(lat + sn * cx + c * cy) for cx, cy in corners)
        yaw_err.append(math.degrees(th))
        corner_max.append(cm)
        base_lat.append(abs(lat))
        if s <= FIRST_M:
            first_corner.append((cm, lbl))
        if cm > limit:
            if cur is None or cur[0] != lbl:
                cur = [lbl, 0, 0.0]
                events.append(cur)
            cur[1] += 1
            cur[2] = max(cur[2], cm)
        else:
            cur = None
    print(g.stamp() or '(車輛幾何全部 measured)')
    crit = g.lateral_half_extent + (g.headland_width - g.rotation_swept_radius)
    if first_corner:
        vals = [v for v, _l in first_corner]
        over = sorted({l for v, l in first_corner if v >= crit})
        print('【前 %.1f m】(對正後剛起步) 四角最大側向偏移：最大 %.3f m、p95 %.3f m、中位 %.3f m，%d 筆；'
              '判準 < lateral_half_extent %.3f + 轉彎餘裕 %.4f = %.3f m -> %s'
              % (FIRST_M, max(vals), pct(vals, 0.95), pct(vals, 0.5), len(vals), g.lateral_half_extent,
                 g.headland_width - g.rotation_swept_radius, crit, '通過' if max(vals) < crit else '超過'))
        if over:
            print('    超過判準的段：%s' % ', '.join(over))
    else:
        print('【前 %.1f m】沒有取樣點' % FIRST_M)
    print('直線段取樣點 %d 筆 (割草線 %d 段)' % (len(corner_max), len(queue)))
    if not corner_max:
        return
    ay = [abs(v) for v in yaw_err]
    print('偏航誤差 |θ| (deg)：中位 %.2f  p95 %.2f  最大 %.2f   (有號平均 %+.2f)'
          % (pct(ay, 0.5), pct(ay, 0.95), max(ay), sum(yaw_err) / len(yaw_err)))
    for lo, hi in ((0, 1), (1, 2), (2, 5), (5, 10), (10, 180)):
        n = sum(1 for v in ay if lo <= v < hi)
        print('    %3d ~ %3d deg : %6d 筆 (%5.1f %%)' % (lo, hi, n, 100.0 * n / len(ay)))
    print('base_footprint 側向偏差 (m)：中位 %.3f  p95 %.3f  最大 %.3f'
          % (pct(base_lat, 0.5), pct(base_lat, 0.95), max(base_lat)))
    print('四角最大側向偏移 (m)：中位 %.3f  p95 %.3f  最大 %.3f   (門檻 lateral_half_extent %.3f)'
          % (pct(corner_max, 0.5), pct(corner_max, 0.95), max(corner_max), limit))
    n_over = sum(1 for v in corner_max if v > limit)
    print('超過 %.3f 的取樣點 %d 筆 (%.1f %%)，合併成 %d 次事件，涉及 %d 段'
          % (limit, n_over, 100.0 * n_over / len(corner_max), len(events),
             len(set(e[0] for e in events))))
    for lbl, n, mx in sorted(events, key=lambda e: -e[2])[:15]:
        print('    %-14s %4d 筆  最大 %.3f m' % (lbl, n, mx))


if __name__ == '__main__':
    main()
