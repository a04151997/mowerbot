#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""覆蓋率拆帳：A 草坪總面積 / B 外圈 / C 割草線 / D 重疊 / E 未覆蓋(E1+E2+E3)。

用法:
    python3 test/tools/coverage_budget.py <prefix>
    (需要 <prefix>_traj.csv 與 <prefix>_map.npz，由 coverage_run.py 產生，
     並用 --log <mower_control log> 提供 manager 的佇列與跳過紀錄)

=========================== 量法（可重現） ===========================
A  草坪總面積
   來源是 **世界檔的幾何**，不是 SLAM 地圖：把 demo_lawn.world 裡所有
   box 模型光柵化成佔據格，取「包含原點的連通自由區域」。
   用世界檔而不是 SLAM 地圖，是因為 A 是分母，不該隨建圖品質浮動。

B  外圈 pass 實際覆蓋面積
   軌跡上 current_label 以「周邊環繞」開頭的取樣點連成折線，
   往外膨脹刀盤半徑 0.25 m（刀盤寬 0.50 m）。
   **是實際軌跡，不是規劃路徑。**

C  割草線實際覆蓋面積
   同上，label 以「割草線」開頭。
   **扣掉跑道段**：跑道上刀盤是關的（設計如此），所以把投影落在
   割草起點 A 之前的取樣點去掉。A 由 manager 的
   「🛬 割草線 i/N 加跑道：... 起點 (x,y) -> A (x,y)」取得；
   沒有跑道的割草線，A 就是該段起點。

D  重疊 = B ∩ C

E  未覆蓋 = A − (B + C − D)，並拆成三類：
   E1 邊界安全帶：離草坪真實邊界 < 0.19 m 的未覆蓋格。
      0.19 = (車體半寬 0.34 + 邊界腐蝕 0.10) − 刀盤半徑 0.25。
      也就是「車心最近只能到 0.44 m，刀盤再往外 0.25 m」之後仍然到不了的帶狀區。
      **物理上不可能割到**，不是控制或規劃的問題。
   E3 被跳過的段落：未覆蓋格中，落在「被跳過段落的規劃路徑 ± 0.25 m」內的部分。
      被跳過的段落與其起訖點由 manager 的 📋 佇列 與 ⏭️ 跳過此段 log 取得。
   E2 其餘：割草線之間的縫隙，成因是循跡誤差。

   三類互斥，依 E1 -> E3 -> E2 的順序歸屬（先扣掉物理不可能的，
   再扣掉根本沒去割的，剩下的才算循跡誤差）。

光柵解析度 0.01 m。
=====================================================================
"""
import argparse, csv, math, os, re, sys
import numpy as np
import cv2
import xml.etree.ElementTree as ET

CELL = 0.01
BLADE_HALF = 0.25          # 刀盤寬 0.50 m
BODY_HALF = 0.34           # 車體半寬 (footprint 0.95 x 0.68)
ERODE = 0.10               # map_to_boundary 的邊界腐蝕量
E1_BAND = BODY_HALF + ERODE - BLADE_HALF      # 0.19 m

RE_QUEUE = re.compile(
    r'📋 佇列 (\d+)/(\d+) \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) '
    r'終點 \(([-\d.]+), ([-\d.]+)\) 長度 ([\d.]+) m')
RE_SKIP = re.compile(r'⏭️ 跳過此段 (.+?) \(起點 ([-\d.]+), ([-\d.]+)\)')
RE_LEADIN = re.compile(
    r'🛬 (割草線 \d+/\d+) 加跑道：長度 ([\d.]+) m，起點 \(([-\d.]+), ([-\d.]+)\)'
    r' -> A \(([-\d.]+), ([-\d.]+)\)')


def world_boxes(path):
    root = ET.parse(path).getroot()
    out = []
    for m in root.iter('model'):
        pose = m.findtext('pose')
        if not pose:
            continue
        p = [float(v) for v in pose.split()]
        size = None
        for b in m.iter('box'):
            s = b.findtext('size')
            if s:
                size = [float(v) for v in s.split()]
                break
        if size:
            out.append((m.get('name'), p[0], p[1], p[5], size[0], size[1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--log', required=True, help='mower_control 的 log 檔')
    ap.add_argument('--world', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..',
        'src/mowerbot_bringup/worlds/demo_lawn.world'))
    args = ap.parse_args()

    # ---- A：從世界檔建出草坪 ----
    boxes = world_boxes(args.world)
    xs = [b[1] for b in boxes]; ys = [b[2] for b in boxes]
    pad = 1.0
    x0, x1 = min(xs)-pad, max(xs)+pad
    y0, y1 = min(ys)-pad, max(ys)+pad
    W = int(round((x1-x0)/CELL)); H = int(round((y1-y0)/CELL))
    occ = np.zeros((H, W), np.uint8)
    def to_px(x, y):
        return (int(round((x-x0)/CELL)), int(round((y-y0)/CELL)))
    for _n, cx, cy, yaw, sx, sy in boxes:
        hx, hy = sx/2.0, sy/2.0
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.array([to_px(cx+c*dx-s*dy, cy+s*dx+c*dy)
                        for dx, dy in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]], np.int32)
        cv2.fillConvexPoly(occ, pts, 1)
    free = (occ == 0).astype(np.uint8)
    num, labels = cv2.connectedComponents(free, 4)
    lawn = (labels == labels[to_px(0.0, 0.0)[1], to_px(0.0, 0.0)[0]])
    A = int(lawn.sum()) * CELL * CELL

    # 草坪內每一格離真實邊界(牆/障礙物)的距離
    dist_wall = cv2.distanceTransform((occ == 0).astype(np.uint8),
                                      cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * CELL

    # ---- 軌跡 ----
    rows = list(csv.DictReader(open(args.prefix + '_traj.csv')))
    traj = [(float(r['t']), float(r['x']), float(r['y']), r['label']) for r in rows]

    txt = open(args.log, encoding='utf-8', errors='replace').read()
    lead_a = {m[0]: (float(m[4]), float(m[5])) for m in RE_LEADIN.findall(txt)}
    # 一份 log 可能含多輪規劃（失敗後重新切 mode 1 會再規劃一次）。
    # 只取「最後一輪」：佇列是連號印出來的，i == 1 就是新的一輪開始。
    queue = {}
    for i, _n, lbl, sx, sy, ex, ey, ln in RE_QUEUE.findall(txt):
        if int(i) == 1:
            queue = {}
        queue[lbl] = (float(sx), float(sy), float(ex), float(ey), float(ln))
    skipped = [m[0] for m in RE_SKIP.findall(txt)]

    def mask_from_points(points):
        m = np.zeros((H, W), np.uint8)
        if len(points) < 2:
            return m.astype(bool)
        poly = np.array([to_px(px, py) for px, py in points], np.int32)
        cv2.polylines(m, [poly], False, 1, int(round(2*BLADE_HALF/CELL)))
        return m.astype(bool)

    def runs(pred):
        cur, out = [], []
        for t, x, y, lbl in traj:
            if pred(lbl):
                cur.append((x, y))
            elif cur:
                out.append(cur); cur = []
        if cur:
            out.append(cur)
        return out

    B = np.zeros((H, W), bool)
    for seg in runs(lambda l: l.startswith('周邊環繞')):
        B |= mask_from_points(seg)

    C = np.zeros((H, W), bool)
    cur_lbl, cur = None, []
    def flush(lbl, pts):
        if not lbl or len(pts) < 2:
            return
        a = lead_a.get(lbl)
        if a is None and lbl in queue:
            a = (queue[lbl][0], queue[lbl][1])
        if a is not None and lbl in queue:
            sx, sy, ex, ey, _ln = queue[lbl]
            dx, dy = ex-sx, ey-sy
            nn = math.hypot(dx, dy)
            if nn > 1e-6:
                ux, uy = dx/nn, dy/nn
                s_a = (a[0]-sx)*ux + (a[1]-sy)*uy
                pts = [p for p in pts if (p[0]-sx)*ux + (p[1]-sy)*uy >= s_a - 0.05]
        if len(pts) >= 2:
            C[:] |= mask_from_points(pts)
    for t, x, y, lbl in traj:
        if lbl.startswith('割草線'):
            if lbl != cur_lbl:
                flush(cur_lbl, cur); cur_lbl, cur = lbl, []
            cur.append((x, y))
        else:
            flush(cur_lbl, cur); cur_lbl, cur = None, []
    flush(cur_lbl, cur)

    B &= lawn; C &= lawn
    D = B & C
    covered = B | C
    uncovered = lawn & ~covered

    E3m = np.zeros((H, W), bool)
    for lbl in set(skipped):
        # approach 段不割草（刀盤關著），跳過它不會造成「沒割到」
        if lbl.startswith('approach'):
            continue
        if lbl in queue:
            sx, sy, ex, ey, _ln = queue[lbl]
            E3m |= mask_from_points([(sx, sy), (ex, ey)])
    E1m = uncovered & (dist_wall < E1_BAND)
    E3m = uncovered & E3m & ~E1m
    E2m = uncovered & ~E1m & ~E3m
    # E2 再拆兩半：靠近邊界那一圈**規劃上就沒有涵蓋**，不是循跡誤差。
    #   外圈刀盤最外緣  = headland 0.70 + tool/2 0.15 − 刀盤半徑 0.25 = 離邊界 0.60 m
    #   割草線刀盤最外緣 = headland 0.70 − 刀盤半徑 0.25          = 離邊界 0.45 m
    #   邊界本身又比真實自由區域內縮 0.10 m
    # -> 離真實牆面 < 0.45 + 0.10 = 0.55 m 的地方，規劃上就不會被割到。
    NEVER_PLANNED = 0.45 + ERODE
    E2a = E2m & (dist_wall < NEVER_PLANNED)
    E2b = E2m & ~E2a

    def m2(mask):
        return int(mask.sum()) * CELL * CELL
    print('=' * 70)
    print('覆蓋率拆帳  (demo_lawn, headland=0.70, 刀盤 0.50 m, 光柵 %.2f m)' % CELL)
    print('=' * 70)
    print('A  草坪總面積 (世界檔幾何)            %8.2f m²' % A)
    print('B  外圈 pass 實際覆蓋                 %8.2f m²  (%5.1f%% of A)' % (m2(B), 100*m2(B)/A))
    print('C  割草線實際覆蓋 (已扣跑道段)        %8.2f m²  (%5.1f%% of A)' % (m2(C), 100*m2(C)/A))
    print('D  重疊 B∩C                          %8.2f m²' % m2(D))
    print('   B + C − D = 已覆蓋                 %8.2f m²  (%5.1f%% of A)'
          % (m2(covered), 100*m2(covered)/A))
    print('E  未覆蓋 = A − (B+C−D)               %8.2f m²  (%5.1f%% of A)'
          % (m2(uncovered), 100*m2(uncovered)/A))
    print('-' * 70)
    n_skip_mow = len([l for l in set(skipped) if not l.startswith('approach')])
    for name, mask, desc in (
            ('E1 ', E1m, '邊界安全帶 (< %.2f m，物理上割不到)' % E1_BAND),
            ('E2a', E2a, '邊界環帶 (< %.2f m，規劃就沒涵蓋)' % NEVER_PLANNED),
            ('E2b', E2b, '割草線之間的縫隙 (循跡誤差)'),
            ('E3 ', E3m, '被跳過的割草段落 (%d 段)' % n_skip_mow)):
        a = m2(mask)
        print('   %s %-34s %8.2f m²  (%5.1f%% of A)' % (name, desc, a, 100*a/A))
        n_lab, lab, stats, cent = cv2.connectedComponentsWithStats(
            mask.astype(np.uint8), 8)
        blobs = sorted(((stats[i, cv2.CC_STAT_AREA]*CELL*CELL,
                         x0+cent[i][0]*CELL, y0+cent[i][1]*CELL)
                        for i in range(1, n_lab)), reverse=True)[:3]
        for ar, bx, by in blobs:
            if ar >= 0.05:
                print('        最大區塊 %.2f m² 於 (%.2f, %.2f)' % (ar, bx, by))
    print('-' * 70)
    print('   E2 合計 (E2a + E2b)                %8.2f m²  (%5.1f%% of A)'
          % (m2(E2m), 100*m2(E2m)/A))
    print('   三類合計                           %8.2f m²' % (m2(E1m)+m2(E2m)+m2(E3m)))
    print('   覆蓋率 = (B+C−D)/A                  %7.2f %%' % (100*m2(covered)/A))
    print('   扣掉 E1 之後的覆蓋率                %7.2f %%'
          % (100*m2(covered)/(A-m2(E1m))))
    print('   扣掉 E1+E2a (規劃上可割的部分) 的覆蓋率 %6.2f %%'
          % (100*m2(covered)/(A-m2(E1m)-m2(E2a))))
    if skipped:
        print('   被跳過的段落：%s' % ', '.join(sorted(set(skipped))))


if __name__ == '__main__':
    main()
