#!/usr/bin/env python3
"""規劃路徑有沒有穿過未觀測區（報告 29.3，只量測）。

用法:
    python3 test/tools/plan_vs_unknown.py <地圖.yaml> <run 目錄>

run 目錄要有 run_plan.csv（coverage_run.py 錄的 /f2c_path）與 demo.log（取佇列）。
對「周邊環繞折線」與「每一條割草線」：
  - 中心線落在 .pgm 未知格（灰階 205）上的長度
  - 刀盤範圍（± 0.25 m）與未知格重疊的面積，並分成
    「落在 >= 0.09 m² 的未知區塊」（map_to_boundary 會當成洞的大小）與
    「落在更小的未知碎點」
"""
import csv, math, os, re, sys
import numpy as np
import cv2
import yaml

RE_Q = re.compile(r'📋 佇列 (\d+)/\d+ \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) 終點 \(([-\d.]+), ([-\d.]+)\)')


def main():
    ymlp, run = sys.argv[1], sys.argv[2]
    y = yaml.safe_load(open(ymlp))
    img = np.flipud(cv2.imread(os.path.join(os.path.dirname(ymlp), y['image']), cv2.IMREAD_UNCHANGED))
    res = y['resolution']; ox, oy = y['origin'][0], y['origin'][1]
    unk = (img == 205).astype(np.uint8)
    n, lab, st, _c = cv2.connectedComponentsWithStats(unk, 8)
    big = np.isin(lab, [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] * res * res >= 0.09])
    small = (unk > 0) & ~big

    queue = {}
    for line in open(run + '/demo.log', encoding='utf-8', errors='replace'):
        m = RE_Q.search(line)
        if m:
            if m.group(1) == '1':
                queue = {}
            queue[m.group(2)] = tuple(float(m.group(k)) for k in range(3, 7))
    plan = [(float(r['x']), float(r['y'])) for r in csv.DictReader(l for l in open(run + '/run_plan.csv') if not l.startswith('#'))]
    per = next((plan[:i + 1] for i in range(1, len(plan)) if plan[i] == plan[0]), [])

    def px(x, y_):
        return int(round((x - ox) / res)), int(round((y_ - oy) / res))

    def measure(pieces):
        line_len = 0.0
        mask = np.zeros_like(unk)
        for pts in pieces:
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                L = math.hypot(x2 - x1, y2 - y1)
                k = max(2, int(L / 0.01) + 1)
                for j in range(k - 1):
                    u, v = px(x1 + (x2 - x1) * j / (k - 1), y1 + (y2 - y1) * j / (k - 1))
                    if 0 <= v < unk.shape[0] and 0 <= u < unk.shape[1] and unk[v, u]:
                        line_len += L / (k - 1)
            if len(pts) >= 2:
                cv2.polylines(mask, [np.array([px(*p) for p in pts], np.int32)], False, 1,
                              int(round(0.5 / res)))
        mb = mask.astype(bool)
        return line_len, (mb & big).sum() * res * res, (mb & small).sum() * res * res

    pieces, cur = [], []
    for p in per:
        if cur and math.hypot(p[0] - cur[-1][0], p[1] - cur[-1][1]) > 0.5:
            pieces.append(cur); cur = []
        cur.append(p)
    if cur:
        pieces.append(cur)
    print('%s  on  %s（未知格 %d，其中 >= 0.09 m² 的區塊 %.2f m²、碎點 %.2f m²）'
          % (run, os.path.basename(ymlp), int(unk.sum()), big.sum() * res * res, small.sum() * res * res))
    ll, ab, as_ = measure(pieces)
    print('周邊環繞：中心線在未知格上 %.2f m；刀盤範圍與未知區塊重疊 %.3f m²、與碎點重疊 %.3f m²' % (ll, ab, as_))
    tot = [0.0, 0.0, 0.0]; worst = []
    for lbl, (sx, sy, ex, ey) in queue.items():
        if not lbl.startswith('割草線'):
            continue
        ll, ab, as_ = measure([[(sx, sy), (ex, ey)]])
        tot[0] += ll; tot[1] += ab; tot[2] += as_
        worst.append((ll, ab, lbl))
    print('割草線 %d 條：中心線在未知格上合計 %.2f m；刀盤範圍與未知區塊重疊 %.3f m²、與碎點重疊 %.3f m²'
          % (sum(1 for l in queue if l.startswith('割草線')), tot[0], tot[1], tot[2]))
    for ll, ab, lbl in sorted(worst, reverse=True)[:5]:
        if ll > 0 or ab > 0:
            print('   %-14s 中心線在未知格上 %.2f m，刀盤範圍與未知區塊重疊 %.3f m²' % (lbl, ll, ab))


if __name__ == '__main__':
    main()
