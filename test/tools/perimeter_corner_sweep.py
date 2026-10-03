# -*- coding: utf-8 -*-
"""周邊環繞轉角偵測的門檻掃描 (階段 38 決定 3-B)。離線、只用 manager 的 perimeter_corner_cuts()。

用法：python3 perimeter_corner_sweep.py <perimeter_xy.csv>   (perimeter_probe.sh 產生)
對每個 (累積轉角門檻, 弧長窗口)：切成幾段、最短 / 最長段、< 0.5 m 的極短段數 (過度切段)、
以及「切完之後段落內仍有一個車身長 (1.13 m) 內累積轉角 > 60° 的地方」的段數 (漏切)。
漏切的判準固定用 60° / 1.13 m，與被掃的門檻無關 —— 否則每個組合都會說自己沒漏。
"""
import csv, math, sys
from mowerbot_action.manager import perimeter_corner_cuts
from mowerbot_description import vehicle_geometry
g = vehicle_geometry.load()
xy = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(sys.argv[1]))]


def seglen(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))


def max_window_turn(pts, window):
    head = [math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]) for i in range(len(pts) - 1)]
    pos, s = [], 0.0
    for i in range(len(pts) - 1):
        pos.append(s); s += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
    turn = [0.0] + [math.atan2(math.sin(head[i] - head[i - 1]), math.cos(head[i] - head[i - 1]))
                    for i in range(1, len(head))]
    best = 0.0
    for j in range(len(turn)):
        acc = sum(turn[k] for k in range(j, len(turn)) if pos[k] - pos[j] < window)
        best = max(best, abs(acc))
    return math.degrees(best)


CHECK_ANGLE, CHECK_WINDOW = 60.0, g.body_length_total
print('環繞 %d 個航點，全長 %.2f m；漏切判準：段內 %.2f m 弧長內累積轉角 > %.0f°' % (
    len(xy), seglen(xy), CHECK_WINDOW, CHECK_ANGLE))
print('%-22s %4s %8s %8s %8s %6s  %s' % ('門檻 / 窗口', '段數', '最短 m', '最長 m', '<0.5m', '漏切', '各段長度'))
combos = [(a, w) for a in (45.0, 60.0, 75.0) for w in (0.8, 1.13, 1.5)]
combos.append((math.degrees(g.perimeter_corner_angle), g.perimeter_corner_window))
for a, w in combos:
    cuts = perimeter_corner_cuts(xy, w, math.radians(a))
    segs, start = [], 0
    for c in cuts + [len(xy) - 1]:
        seg = xy[start:c + 1]
        if len(seg) >= 3:
            segs.append(seg)
        start = c
    lens = [seglen(s_) for s_ in segs]
    miss = sum(1 for s_ in segs if max_window_turn(s_, CHECK_WINDOW) > CHECK_ANGLE)
    tag = '  <- 推導值' if (a, w) == combos[-1] else ''
    print('%5.1f° / %4.2f m        %4d %8.2f %8.2f %8d %6d  %s%s' % (
        a, w, len(segs), min(lens), max(lens), sum(1 for l in lens if l < 0.5), miss,
        ' '.join('%.1f' % l for l in lens), tag))
