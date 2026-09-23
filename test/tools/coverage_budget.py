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


def draw_heatmap(out_path, occ, lawn, covered, queue, skipped, traj,
                 x0, y0, cell, st):
    """二值覆蓋熱圖 + 規劃路徑疊圖。

    底圖三個色階：場外/牆體、草坪上未割到的、草坪上割到的。
    疊圖分兩種，因為兩種東西 log 裡留下的資訊不一樣：
      * 割草線是直線，log 的起點/終點就是完整的**規劃路徑**，直接畫。
      * 周邊環繞是沿著邊界多邊形走的折線，log 只有起點與終點，
        把它畫成直線會變成一條穿過場中央的假線。所以改畫**實際軌跡**，
        被跳過的那一段（沒有軌跡）只標出兩個端點。
    """
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    for fam in ('Noto Sans CJK TC', 'Noto Sans CJK JP', 'AR PL UMing TW'):
        matplotlib.rcParams['font.sans-serif'] = [fam] + \
            matplotlib.rcParams['font.sans-serif']
    matplotlib.rcParams['axes.unicode_minus'] = False

    H, W = lawn.shape
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = (70, 70, 78)                      # 場外 / 牆體
    img[lawn] = (232, 120, 120)                # 草坪未覆蓋
    img[lawn & covered] = (250, 250, 250)      # 草坪已覆蓋
    ext = [x0, x0 + W * cell, y0, y0 + H * cell]

    fig, ax = plt.subplots(figsize=(10, 9.5), dpi=140)
    ax.imshow(img, origin='lower', extent=ext, interpolation='nearest')
    # 周邊環繞的實際軌跡
    cur = []
    for _t, px, py, lbl in traj:
        if lbl.startswith('周邊環繞'):
            cur.append((px, py))
        elif cur:
            ax.plot([p[0] for p in cur], [p[1] for p in cur],
                    color='#ff7f0e', lw=1.6, zorder=4)
            cur = []
    if cur:
        ax.plot([p[0] for p in cur], [p[1] for p in cur],
                color='#ff7f0e', lw=1.6, zorder=4)
    # 規劃割草線
    for lbl, (sx, sy, ex, ey, _ln) in queue.items():
        if not lbl.startswith('割草線'):
            continue
        c = '#d62728' if lbl in skipped else '#1f77b4'
        ax.plot([sx, ex], [sy, ey], color=c,
                lw=2.2 if lbl in skipped else 1.0, zorder=5 if lbl in skipped else 3)
    # 被跳過的周邊環繞：只有端點
    for lbl in sorted(skipped):
        if lbl.startswith('周邊環繞') and lbl in queue:
            sx, sy, ex, ey, _ln = queue[lbl]
            ax.plot([sx, ex], [sy, ey], 'x', color='#d62728', ms=11, mew=2.5,
                    zorder=6)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_title('覆蓋熱圖：白=已割到、紅=草坪上沒割到\n'
                 '草坪 %.1f m²，已覆蓋 %.1f m² (%.1f %%)，'
                 '未覆蓋 %.1f m² (E1 %.1f / E2a %.1f / E2b %.1f)'
                 % (st['A'], st['cov'], 100 * st['cov'] / st['A'],
                    st['E'], st['E1'], st['E2a'], st['E2b']), fontsize=11)
    ax.legend(handles=[
        Patch(facecolor='#fafafa', edgecolor='#999', label='已割到'),
        Patch(facecolor='#e87878', edgecolor='#999', label='草坪上沒割到'),
        Patch(facecolor='#46464e', edgecolor='#999', label='牆體 / 場外'),
        Line2D([], [], color='#1f77b4', lw=1.2, label='規劃割草線'),
        Line2D([], [], color='#ff7f0e', lw=1.6, label='周邊環繞實際軌跡'),
        Line2D([], [], color='#d62728', marker='x', ls='none', ms=9, mew=2.5,
               label='被跳過段落的端點'),
    ], loc='upper left', bbox_to_anchor=(1.01, 1.0), fontsize=9, frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('prefix')
    ap.add_argument('--log', required=True, help='mower_control 的 log 檔')
    ap.add_argument('--heatmap', default=None,
                    help='另外輸出一張覆蓋熱圖 PNG 到這個路徑')
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
    # 一份 log 可能含多輪規劃（失敗後重新切 mode 1 會再規劃一次）。
    # 只取「最後一輪」：佇列是連號印出來的，i == 1 就是新的一輪開始。
    #
    # 跑道 (lead-in) 的 A 點也必須跟著一起重置。跑道是在規劃階段算的，
    # 所以同一條割草線的 🛬 訊息會在每一輪各出現一次，而且每一輪的結果
    # 不一樣（邊界變了、可行的跑道長度就變了）。用 findall 掃全檔會讓
    # 「這一輪沒有跑道」的割草線沿用上一輪的 A 點，C 被多切掉一段，
    # 未覆蓋面積因而虛增 —— demo_lawn 那次有 8 條割草線中招，
    # 其中割草線 38/39 整條被切光。
    queue = {}
    lead_a, pending = {}, {}
    for line in txt.splitlines():
        m = RE_LEADIN.search(line)
        if m:
            pending[m.group(1)] = (float(m.group(5)), float(m.group(6)))
        q = RE_QUEUE.search(line)
        if q:
            if int(q.group(1)) == 1:
                queue = {}
                lead_a, pending = pending, {}
            queue[q.group(3)] = (float(q.group(4)), float(q.group(5)),
                                 float(q.group(6)), float(q.group(7)),
                                 float(q.group(8)))
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
    e3_unmeasurable = []
    for lbl in set(skipped):
        # approach 段不割草（刀盤關著），跳過它不會造成「沒割到」
        if lbl.startswith('approach'):
            continue
        if lbl in queue:
            sx, sy, ex, ey, ln = queue[lbl]
            # log 只留下起點與終點，這裡用「起點到終點的直線」當成規劃路徑。
            # 割草線本來就是直線，弦長 == 段長，這個近似是精確的；
            # 但周邊環繞是沿著邊界多邊形走的折線，弦長會遠小於段長
            # （demo_lawn 的周邊環繞 4/5：段長 5.21 m，弦長只有 1.70 m）。
            # 這種段落用直線量出來的 E3 是錯的位置、錯的面積，不能當 0 讀。
            chord = math.hypot(ex - sx, ey - sy)
            if ln > chord + 0.20:
                e3_unmeasurable.append((lbl, ln, chord))
                continue
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
    unmeasurable = {l for l, _n, _c in e3_unmeasurable}
    n_skip_mow = len([l for l in set(skipped)
                      if not l.startswith('approach') and l not in unmeasurable])
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
    for lbl, ln, chord in e3_unmeasurable:
        print('   ⚠️ %s 被跳過，但它是折線 (段長 %.2f m、弦長 %.2f m)，'
              'log 裡沒有中間航點，這一段的 E3 量不出來，沒有計入上面的 E3。'
              % (lbl, ln, chord))

    if args.heatmap:
        draw_heatmap(args.heatmap, occ, lawn, covered, queue, set(skipped), traj,
                     x0, y0, CELL,
                     dict(A=A, cov=m2(covered), E=m2(uncovered), E1=m2(E1m),
                          E2a=m2(E2a), E2b=m2(E2b)))
        print('\n🖼️  熱圖已寫入 %s' % args.heatmap)


if __name__ == '__main__':
    main()
