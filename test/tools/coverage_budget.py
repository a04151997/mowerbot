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

E  未覆蓋 = A − (B + C − D)。階段 29 起分成五類（量法見報告 29.2）：
   規劃涵蓋 = 計畫裡每一段的刀盤範圍（± 0.25 m）。計畫取自 <prefix>_plan.csv
   （coverage_run.py 錄的 /f2c_path）：周邊環繞 = 路徑開頭到收尾航點那一段折線，
   割草線 = 佇列的直線（扣掉跑道）。

   E1  邊界安全帶：離真實邊界 < 0.19 m 的草坪。只由幾何決定。
       0.19 = (車體半寬 0.34 + 邊界腐蝕 0.10) − 刀盤半徑 0.25，物理上割不到。
   E2a 規劃就沒涵蓋：E1 以外、不在任何一段規劃涵蓋裡的草坪。**只由計畫決定**，
       同一個計畫下每趟都一樣。軌跡偏出計畫時可能割到其中一部分（「計畫外被割到」），
       另外列出；E 的分解用扣掉這部分之後的量。
       （階段 28 以前 E2a = 未覆蓋 ∩ 離牆 < 0.55 m：「未覆蓋」把軌跡混了進來，
        0.55 m 也只是用 headland 算出來的近似邊線，所以同一個計畫五趟差了 1.38 m²。）
   E3  規劃涵蓋裡沒割到、屬於「輪到了但沒割」的段落：⏭️ 跳過、
       階段 26 的「超出局部代價地圖範圍」跳過、送出後失敗而沒有 ⏭️ 的那一段。
   E4  規劃涵蓋裡沒割到、屬於「任務中止後從未送出」的段落。成因與 E3 完全不同。
   E2b 規劃涵蓋裡沒割到的其餘部分：循跡誤差。

   歸屬順序 E3 -> E4 -> E2b（相鄰割草線的刀盤範圍重疊，重疊處先算給沒割的段落）。

光柵解析度 0.01 m。
=====================================================================
"""
import argparse, csv, math, os, re, sys
import numpy as np
import cv2
import xml.etree.ElementTree as ET

CELL = 0.01


def _load_vehicle():
    """車輛幾何的單一來源：mowerbot_description/config/vehicle.yaml（安裝後的那一份）"""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory('mowerbot_description'),
                           'config', 'vehicle.yaml')) as fh:
        return yaml.safe_load(fh)


VEHICLE = _load_vehicle()
BLADE_HALF = VEHICLE['blade_width'] / 2.0          # 刀盤半徑 (目前 0.50 / 2)
BODY_HALF = VEHICLE['footprint_width'] / 2.0       # 車體半寬 = 內切半徑 (目前 0.68 / 2)
ERODE = 0.10               # map_to_boundary 的邊界腐蝕量
E1_BAND = BODY_HALF + ERODE - BLADE_HALF      # 0.19 m

RE_QUEUE = re.compile(
    r'📋 佇列 (\d+)/(\d+) \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) '
    r'終點 \(([-\d.]+), ([-\d.]+)\) 長度 ([\d.]+) m')
RE_SKIP = re.compile(r'⏭️ 跳過此段 (.+?) \(起點 ([-\d.]+), ([-\d.]+)\)')
RE_SEND = re.compile(r'➡️ 送出任務 \[(.+?)\]')
RE_DONE = re.compile(r'✅ (.+?) 完成')
RE_FAILED = re.compile(r'❌ \[(.+?)\] 失敗')
# 階段 26 的「超出局部代價地圖範圍」：沒有送給 Nav2，直接跳過，訊息格式與 ⏭️ 不同
RE_OUT_OF_RANGE = re.compile(r'⚠️ \[(.+?)\] approach .*?超出局部代價地圖範圍.*?跳過此段')
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
    ap.add_argument('--plan', default=None,
                    help='規劃路徑 CSV（預設 <prefix>_plan.csv，coverage_run.py 產生）')
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
    plan = [(float(r['x']), float(r['y']))
             for r in csv.DictReader(open(args.plan or args.prefix + '_plan.csv'))]
    if not plan:
        sys.exit('計畫檔是空的：沒有收到 /f2c_path，E2a 無法由計畫決定')

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

    # ---- 規劃涵蓋範圍（階段 29）：只由計畫決定，與軌跡無關 ----
    # 周邊環繞是 /f2c_path 開頭那一段：f2c_server 最後補一個與第 0 點座標完全相同的
    # 收尾航點，所以「第一個與第 0 點相同的點」就是環繞的結尾。
    # 被內部障礙物擋掉的地方航點會跳一段（缺口），畫的時候不能把缺口連起來。
    per = []
    for i in range(1, len(plan)):
        if plan[i] == plan[0]:
            per = plan[:i + 1]
            break

    def poly_mask(pts):
        m = np.zeros((H, W), bool)
        piece = []
        for p in pts:
            if piece and math.hypot(p[0] - piece[-1][0], p[1] - piece[-1][1]) > 0.5:
                m |= mask_from_points(piece); piece = []
            piece.append(p)
        if piece:
            m |= mask_from_points(piece)
        return m

    def planned_mask(lbl):
        """單一段落的規劃涵蓋（刀盤寬）。割草線扣掉跑道（跑道上刀盤是關的）。"""
        sx, sy, ex, ey, _ln = queue[lbl]
        if lbl.startswith('割草線'):
            a = lead_a.get(lbl) or (sx, sy)
            return mask_from_points([a, (ex, ey)])
        if lbl.startswith('周邊環繞') and per:
            i0 = min(range(len(per)), key=lambda i: math.hypot(per[i][0] - sx, per[i][1] - sy))
            rest = range(i0 + 1, len(per))
            if not rest:
                return np.zeros((H, W), bool)
            i1 = min(rest, key=lambda j: math.hypot(per[j][0] - ex, per[j][1] - ey))
            return poly_mask(per[i0:i1 + 1])
        return np.zeros((H, W), bool)

    seg_masks = {lbl: planned_mask(lbl) for lbl in queue if not lbl.startswith('approach')}
    planned = np.zeros((H, W), bool)
    for m_ in seg_masks.values():
        planned |= m_

    # ---- 段落狀態（只看最後一輪規劃之後的 log）----
    #   E3：跳過（⏭️）、超出局部代價地圖範圍而跳過（階段 26）、送出後失敗而沒有 ⏭️ 的
    #       （任務中止那一段）—— 「有輪到它，但沒割」
    #   E4：任務中止之後從來沒有送出的段落 —— 「根本沒輪到它」，成因與 E3 完全不同
    last_round = txt[txt.rfind('📋 佇列 1/'):] if '📋 佇列 1/' in txt else txt
    sent = set(RE_SEND.findall(last_round))
    done = set(RE_DONE.findall(last_round))
    failed = set(RE_FAILED.findall(last_round))
    skipped_q = set(m[0] for m in RE_SKIP.findall(last_round))
    out_of_range = set(RE_OUT_OF_RANGE.findall(last_round))
    e3_set = {l for l in seg_masks
              if l in skipped_q or l in out_of_range or (l in failed and l not in done)}
    e4_set = {l for l in seg_masks if l not in sent and l not in e3_set and l not in done}
    skipped = sorted(e3_set)

    E1m = lawn & (dist_wall < E1_BAND)                 # 物理上割不到（只由幾何決定）
    E2a = lawn & ~E1m & ~planned                       # 規劃就沒涵蓋（只由計畫決定）
    P = lawn & ~E1m & planned                          # 規劃上要割的部分
    E3any = np.zeros((H, W), bool)
    for l in e3_set:
        E3any |= seg_masks[l]
    E4any = np.zeros((H, W), bool)
    for l in e4_set:
        E4any |= seg_masks[l]
    unc_P = uncovered & P
    E3m = unc_P & E3any
    E4m = unc_P & E4any & ~E3m
    E2b = unc_P & ~E3m & ~E4m
    E1_unc = E1m & uncovered
    E2a_unc = E2a & uncovered
    E2a_off = E2a & covered                            # 計畫外被割到（軌跡偏出計畫）

    def m2(mask):
        return int(mask.sum()) * CELL * CELL
    wname = os.path.splitext(os.path.basename(args.world))[0]
    print('=' * 70)
    print('覆蓋率拆帳  (%s, 刀盤 0.50 m, 光柵 %.2f m, 階段 29 版)' % (wname, CELL))
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
    print('只由幾何 / 計畫決定的兩塊（同一個計畫下每趟都一樣）：')
    print('   E1  邊界安全帶 (< %.2f m，物理上割不到) %8.2f m²  (其中被割到 %.2f m²)'
          % (E1_BAND, m2(E1m), m2(E1m & covered)))
    print('   E2a 規劃就沒涵蓋 (計畫的刀盤範圍之外) %8.2f m²  (其中計畫外被割到 %.2f m²)'
          % (m2(E2a), m2(E2a_off)))
    print('規劃上要割的部分 P = %.2f m²，其中沒割到的：' % m2(P))
    for name, mask, desc in (
            ('E2b', E2b, '割草線之間的縫隙 (循跡誤差)'),
            ('E3 ', E3m, '被跳過 / 失敗的段落 (%d 段)' % len(e3_set)),
            ('E4 ', E4m, '任務中止後從未送出 (%d 段)' % len(e4_set))):
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
    tot = m2(E1_unc) + m2(E2a_unc) + m2(E2b) + m2(E3m) + m2(E4m)
    print('   未覆蓋的分解：E1 %.2f + E2a %.2f (扣掉計畫外被割到) + E2b %.2f + E3 %.2f + E4 %.2f'
          ' = %.2f m²（E = %.2f）'
          % (m2(E1_unc), m2(E2a_unc), m2(E2b), m2(E3m), m2(E4m), tot, m2(uncovered)))
    print('   覆蓋率 = (B+C−D)/A                  %7.2f %%' % (100*m2(covered)/A))
    print('   扣掉 E1 之後的覆蓋率                %7.2f %%'
          % (100*m2(covered)/(A-m2(E1m))))
    print('   規劃可割範圍的覆蓋率 = 已覆蓋∩P / P  %7.2f %%'
          % (100*m2(covered & P)/m2(P)))
    if e3_set:
        print('   E3 段落：%s' % ', '.join(sorted(e3_set)))
    if e4_set:
        print('   E4 段落：%s' % ', '.join(sorted(e4_set, key=lambda l: (l.split()[0], int(l.split()[1].split('/')[0])))))
    if not per:
        print('   ⚠️ 計畫裡找不到周邊環繞的收尾航點，周邊環繞的規劃涵蓋沒有算進去')

    if args.heatmap:
        draw_heatmap(args.heatmap, occ, lawn, covered, queue, set(skipped), traj,
                     x0, y0, CELL,
                     dict(A=A, cov=m2(covered), E=m2(uncovered), E1=m2(E1_unc),
                          E2a=m2(E2a), E2b=m2(E2b)))
        print('\n🖼️  熱圖已寫入 %s' % args.heatmap)


if __name__ == '__main__':
    main()
