#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""未覆蓋面積的離線分析 (不重跑模擬，只讀已經存好的 CSV)。

用法:
    python3 test/tools/coverage_analysis.py <log 目錄> [<log 目錄> ...]
    python3 test/tools/coverage_analysis.py --figure <log 目錄>   # 額外輸出 PNG

為什麼要這支程式:
    smoke_test.py 的 N3 只印「指令航點離軌跡超過 0.25 m 的比例」，
    那個指標的分母是航點數，會隨重疊率從 328 漲到 900 多，
    跨重疊率比較會失真。這裡改成算「實際沒被刀盤掃過的面積」，
    分母固定是面積，而且同時用兩種分母各算一次:

      (A) 整個邊界多邊形   —— 含地頭。地頭是設計上就不割的迴轉區，
                              算進去會讓未覆蓋比例灌水。
      (B) 扣掉地頭的作業區 —— 這才是「應該要割到」的區域，是主要指標。

    另外用連通區域分析把未覆蓋區塊拆開，分成「行末迴轉區」與
    「割草線之間的條狀縫隙」兩類，才知道漏割是掉頭造成的還是線距造成的。
"""

import csv
import json
import math
import os
import sys

import cv2
import numpy as np

# --- 與 smoke_test.py 一致的常數，改這裡之前先確認那邊也一樣 ---
BLADE_WIDTH = 0.5           # 實際刀盤寬 (車體物理屬性)
BLADE_HALF = BLADE_WIDTH / 2.0
TEST_BOUNDARY_CENTER = (-1.5, -1.5)
TEST_BOUNDARY_HALF = 2.5    # 5 m x 5 m 測試邊界
HEADLAND_WIDTH = 0.5        # f2c_server 的 headland_width 預設值
TURNING_RADIUS = 1.0        # 迴轉半徑，同時當作「行末區」的寬度
CELL = 0.01                 # 光柵化解析度，公尺


def load_xy(path, skip_cols=0):
    with open(path) as fh:
        rows = list(csv.reader(fh))[1:]
    return [(float(r[-2]), float(r[-1])) for r in rows]


def split_swaths(pts):
    """與 mower_manager.split_path_into_swaths 相同的切法：
    相鄰段方向反轉 (夾角 >= 90 度) 就切開，只有 2 點的橫向連接段丟掉。"""
    cuts, prev = [], None
    for i in range(len(pts) - 1):
        dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
        n = math.hypot(dx, dy)
        if n < 1e-9:
            continue
        cur = (dx / n, dy / n)
        if prev is not None and prev[0] * cur[0] + prev[1] * cur[1] <= 1e-9:
            cuts.append(i)
        prev = cur
    out, start = [], 0
    for c in cuts + [len(pts) - 1]:
        seg = pts[start:c + 1]
        if len(seg) >= 3:
            out.append(seg)
        start = c
    return out


def swath_frame(swaths):
    """取割草線的主方向，回傳 (沿線單位向量 u, 垂直單位向量 v)。"""
    best, blen = None, -1.0
    for sw in swaths:
        d = math.dist(sw[0], sw[-1])
        if d > blen:
            blen, best = d, sw
    dx, dy = best[-1][0] - best[0][0], best[-1][1] - best[0][1]
    n = math.hypot(dx, dy)
    u = (dx / n, dy / n)
    return u, (-u[1], u[0])


def analyse(logdir, want_figure=False, figure_path=None):
    path_csv = os.path.join(logdir, 'n3_path.csv')
    traj_csv = os.path.join(logdir, 'n3_traj.csv')
    pts = load_xy(path_csv)
    traj = load_xy(traj_csv)

    bcx, bcy = TEST_BOUNDARY_CENTER
    hb = TEST_BOUNDARY_HALF
    hm = hb - HEADLAND_WIDTH

    # 光柵化範圍：整個邊界多邊形，四周留一點邊
    pad = 0.3
    x0, y0 = bcx - hb - pad, bcy - hb - pad
    x1, y1 = bcx + hb + pad, bcy + hb + pad
    W = int(round((x1 - x0) / CELL))
    H = int(round((y1 - y0) / CELL))

    def to_px(x, y):
        return (int(round((x - x0) / CELL)), int(round((y - y0) / CELL)))

    # 軌跡畫成 1 px 的線，再用精確歐氏距離變換算每一格離軌跡多遠
    line = np.full((H, W), 255, np.uint8)
    poly = np.array([to_px(px, py) for px, py in traj], np.int32)
    cv2.polylines(line, [poly], False, 0, 1, cv2.LINE_8)
    dist_px = cv2.distanceTransform(line, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    covered = (dist_px * CELL) <= BLADE_HALF

    def rect_mask(half):
        m = np.zeros((H, W), np.uint8)
        p0, p1 = to_px(bcx - half, bcy - half), to_px(bcx + half, bcy + half)
        cv2.rectangle(m, p0, p1, 1, -1)
        return m.astype(bool)

    region = {'boundary': rect_mask(hb), 'mainland': rect_mask(hm)}

    res = {'logdir': os.path.basename(logdir.rstrip('/'))}
    for name, mask in region.items():
        total = int(mask.sum())
        unc = int((mask & ~covered).sum())
        res[name] = {
            'area_m2': total * CELL * CELL,
            'uncut_m2': unc * CELL * CELL,
            'uncut_pct': 100.0 * unc / total if total else float('nan'),
        }

    # --- 連通區域分析：以作業區 (扣地頭) 為準 ---
    unc_mask = ((region['mainland'] & ~covered)).astype(np.uint8)
    n_lab, labels, stats, cents = cv2.connectedComponentsWithStats(unc_mask, 8)

    swaths = split_swaths(pts)
    u, v = swath_frame(swaths)
    all_sw = [p for sw in swaths for p in sw]
    us = [p[0] * u[0] + p[1] * u[1] for p in all_sw]
    umin, umax = min(us), max(us)

    blobs = []
    for i in range(1, n_lab):
        area = stats[i, cv2.CC_STAT_AREA] * CELL * CELL
        cx = x0 + cents[i][0] * CELL
        cy = y0 + cents[i][1] * CELL
        cu = cx * u[0] + cy * u[1]
        # 行末迴轉區 = 質心落在割草線兩端各 TURNING_RADIUS 公尺的帶狀區內
        end = (cu < umin + TURNING_RADIUS) or (cu > umax - TURNING_RADIUS)
        blobs.append({'label': i, 'area_m2': area, 'center': (cx, cy),
                      'kind': 'row_end' if end else 'between_swaths'})

    blobs.sort(key=lambda b: -b['area_m2'])
    tot = sum(b['area_m2'] for b in blobs)
    end_a = sum(b['area_m2'] for b in blobs if b['kind'] == 'row_end')
    bet_a = tot - end_a
    res['components'] = {
        'count': len(blobs),
        'total_m2': tot,
        'row_end_m2': end_a,
        'between_swaths_m2': bet_a,
        'row_end_pct': 100.0 * end_a / tot if tot else 0.0,
        'between_swaths_pct': 100.0 * bet_a / tot if tot else 0.0,
        'top3': [{'area_m2': b['area_m2'],
                  'center': [round(b['center'][0], 3), round(b['center'][1], 3)],
                  'kind': b['kind']} for b in blobs[:3]],
    }
    res['swaths'] = len(swaths)

    if want_figure:
        _figure(figure_path, x0, y0, x1, y1, covered, region, unc_mask,
                labels, blobs, traj, pts, bcx, bcy, hb, hm, res)
    return res


def _figure(out_png, x0, y0, x1, y1, covered, region, unc_mask,
            labels, blobs, traj, pts, bcx, bcy, hb, hm, res):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    # 圖上有中文，指定系統裡有的 CJK 字型，否則會印成空心方框
    matplotlib.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'Droid Sans Fallback']
    matplotlib.rcParams['axes.unicode_minus'] = False

    fig, ax = plt.subplots(figsize=(9.0, 8.4), dpi=150)
    ext = [x0, x1, y0, y1]

    # 底圖：作業區內已割 = 淺綠，未割 = 紅
    rgb = np.ones(covered.shape + (3,), np.float32)
    main = region['mainland']
    head = region['boundary'] & ~region['mainland']
    rgb[head] = (0.93, 0.93, 0.93)                  # 地頭：灰
    rgb[main & covered] = (0.80, 0.91, 0.78)        # 已割：淺綠
    rgb[main & ~covered] = (0.86, 0.20, 0.18)       # 未割：紅
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')

    tx = [p[0] for p in traj]
    ty = [p[1] for p in traj]
    ax.plot(tx, ty, '-', color='#12406b', lw=0.8, alpha=0.9, label='實際軌跡 (odom)')
    ax.plot([p[0] for p in pts], [p[1] for p in pts], '.', color='#888888',
            ms=0.7, alpha=0.5, label='F2C 指令航點')

    ax.add_patch(Rectangle((bcx - hb, bcy - hb), 2 * hb, 2 * hb, fill=False,
                           ec='#333333', lw=1.4, label='邊界 (5x5 m)'))
    ax.add_patch(Rectangle((bcx - hm, bcy - hm), 2 * hm, 2 * hm, fill=False,
                           ec='#2f7f2f', lw=1.4, ls='--',
                           label='作業區 (扣地頭 4x4 m)'))

    for k, b in enumerate(blobs[:3]):
        ax.annotate('#%d  %.3f m²' % (k + 1, b['area_m2']),
                    xy=b['center'], xytext=(b['center'][0], b['center'][1] + 0.28),
                    color='black', fontsize=8, ha='center',
                    bbox=dict(fc='white', ec='#86322e', lw=0.7, alpha=0.9, pad=1.4),
                    arrowprops=dict(arrowstyle='->', color='#86322e', lw=0.8))

    c = res['components']
    ax.set_title('未覆蓋區域分布  (紅 = 刀盤沒掃到)\n'
                 '作業區未覆蓋 %.2f%%  ·  區塊 %d 個  ·  '
                 '行末迴轉區 %.1f%% / 線間縫隙 %.1f%%'
                 % (res['mainland']['uncut_pct'], c['count'],
                    c['row_end_pct'], c['between_swaths_pct']), fontsize=10)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=7.5, framealpha=0.92)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    want_fig = '--figure' in sys.argv
    out = []
    for d in args:
        png = None
        if want_fig:
            png = os.path.join('/home/a/Desktop/mowerbot/docs',
                               'coverage_map_%s.png' % os.path.basename(d.rstrip('/')))
        r = analyse(d, want_fig, png)
        if png:
            r['figure'] = png
        out.append(r)
        print('=' * 72)
        print('log 目錄: %s   (割草線 %d 條)' % (r['logdir'], r['swaths']))
        for k, label in (('boundary', '(A) 分母 = 整個邊界多邊形 (含地頭)'),
                         ('mainland', '(B) 分母 = 扣掉地頭的作業區')):
            b = r[k]
            print('  %-34s 面積 %6.2f m²   未覆蓋 %5.3f m²  = %5.2f %%'
                  % (label, b['area_m2'], b['uncut_m2'], b['uncut_pct']))
        c = r['components']
        print('  未覆蓋區塊總數: %d   總面積 %.3f m²' % (c['count'], c['total_m2']))
        print('  最大三塊:')
        for i, t in enumerate(c['top3']):
            print('     #%d  %.4f m²  中心 (%7.3f, %7.3f)  %s'
                  % (i + 1, t['area_m2'], t['center'][0], t['center'][1],
                     '行末迴轉區' if t['kind'] == 'row_end' else '線間條狀縫隙'))
        print('  分類 (面積佔比): 行末迴轉區 %.1f %%   線間條狀縫隙 %.1f %%'
              % (c['row_end_pct'], c['between_swaths_pct']))
        if png:
            print('  圖: %s' % png)
    print()
    print(json.dumps(out, ensure_ascii=False))


if __name__ == '__main__':
    main()
