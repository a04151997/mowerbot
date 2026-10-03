# -*- coding: utf-8 -*-
"""由一趟跑測的紀錄重畫俯視動畫 MP4 (階段 38 後續 6)。

用法：python3 test/tools/replay_video.py <run 目錄> <輸出.mp4> [--speed 20] [--fps 20] [--world <world 檔>]

畫面內容全部來自那一趟模擬的紀錄，不是重新模擬：
  牆 / 障礙物 = 世界檔的方塊；灰線 = manager 最後一輪佇列的割草線 (規劃)；
  綠色 = 割草線與周邊環繞段落中刀盤 (blade_link) 實際掃過的面積 (寬 blade_width；approach / 對正不算)；藍框 = 車身 footprint (base_link 位姿)；
  左上 = 模擬時間、目前段落、已完成段數。
--speed 是相對模擬時間的倍率 (寫進畫面與同名 .txt)。
"""
import argparse, csv, math, os, re, sys

import cv2
import numpy as np

from mowerbot_description import vehicle_geometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_budget import world_boxes  # noqa: E402
from footprint_collision import rect  # noqa: E402

RE_Q = re.compile(r'📋 佇列 (\d+)/(\d+) \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) 終點 \(([-\d.]+), ([-\d.]+)\)')

ap = argparse.ArgumentParser()
ap.add_argument('run'); ap.add_argument('out')
ap.add_argument('--speed', type=float, default=20.0)
ap.add_argument('--fps', type=float, default=20.0)
ap.add_argument('--world', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                                                'src/mowerbot_bringup/worlds/demo_lawn.world'))
a = ap.parse_args()
g = vehicle_geometry.load()
rows = [r for r in csv.DictReader(l for l in open(os.path.join(a.run, 'run_traj.csv')) if not l.startswith('#'))]
queue = {}
for ln in open(os.path.join(a.run, 'manager.log'), encoding='utf-8', errors='replace'):
    m = RE_Q.search(ln)
    if m:
        if int(m.group(1)) == 1:
            queue = {}
        queue[m.group(3)] = tuple(float(m.group(k)) for k in range(4, 8))
boxes = [(n, rect(x, y, yaw, sx, sy)) for n, x, y, yaw, sx, sy in world_boxes(a.world)
         if not n.startswith('ground') and sx * sy < 400]
allx = [p[0] for _, b in boxes for p in b]; ally = [p[1] for _, b in boxes for p in b]
X0, X1, Y0, Y1 = min(allx) - 0.5, max(allx) + 0.5, min(ally) - 0.5, max(ally) + 0.5
S = 70.0                                       # px / m
W, H = int((X1 - X0) * S) // 2 * 2, int((Y1 - Y0) * S) // 2 * 2 + 60


def px(x, y):
    return int((x - X0) * S), int((Y1 - y) * S) + 60


base = np.full((H, W, 3), (40, 90, 40), np.uint8)
for _, b in boxes:
    cv2.fillPoly(base, [np.array([px(*p) for p in b], np.int32)], (60, 60, 60))
for lbl, (sx, sy, ex, ey) in queue.items():
    if lbl.startswith('割草線'):
        cv2.line(base, px(sx, sy), px(ex, ey), (150, 150, 150), 1, cv2.LINE_AA)
mown = np.zeros((H, W), np.uint8)
bw = max(1, int(g.blade_width * S))
vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (W, H))
fp = [tuple(c) for c in g.footprint]
t_first = float(rows[0]['t']); dt_frame = a.speed / a.fps
next_t = t_first; prev = None; done_lbl = []; last_lbl = ''
for r in rows:
    t = float(r['t']); bx, by = float(r['blade_x']), float(r['blade_y'])
    lbl = r['label']
    if lbl != last_lbl and last_lbl.startswith('割草線'):
        done_lbl.append(last_lbl)
    last_lbl = lbl or last_lbl
    if prev is not None and float(r['odom_vx']) > 0.05 and lbl.startswith(('割草線', '周邊環繞')):
        cv2.line(mown, px(*prev), px(bx, by), 255, bw)
    prev = (bx, by)
    if t < next_t:
        continue
    next_t += dt_frame
    fr = base.copy()
    fr[mown > 0] = (90, 200, 90)
    x, y, yaw = float(r['x']), float(r['y']), float(r['yaw'])
    c, s = math.cos(yaw), math.sin(yaw)
    poly = [(x + c * u - s * v, y + s * u + c * v) for u, v in fp]
    cv2.polylines(fr, [np.array([px(*p) for p in poly], np.int32)], True, (255, 160, 40), 2, cv2.LINE_AA)
    cv2.circle(fr, px(bx, by), 3, (0, 0, 255), -1)
    fr[:60] = (25, 25, 25)
    cv2.putText(fr, 'sim t = %5.0f s   x%.0f   segment: %s' % (t - t_first, a.speed,
                lbl.replace('割草線', 'swath').replace('周邊環繞', 'perimeter').replace('對正 → ', 'align>').replace('approach', 'approach') or '-'),
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(fr, 'swaths finished: %d   (replay rendered from logged Gazebo trajectory)' % len(set(done_lbl)),
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    vw.write(fr)
for _ in range(int(a.fps * 3)):
    vw.write(fr)
vw.release()
sim = float(rows[-1]['t']) - t_first
msg = '模擬 %.0f s，影片 %.0f s，%.0f 倍速 (由紀錄重畫，不是螢幕錄影)' % (sim, sim / a.speed + 3, a.speed)
print(msg)
open(a.out.rsplit('.', 1)[0] + '.txt', 'w').write(msg + '\n')
