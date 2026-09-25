#!/usr/bin/env python3
"""單一段落失敗的診斷（報告 29.4，只量測、不修）。

用法:
    python3 test/tools/segment_failure.py <run 目錄> '<label>' [--world <世界檔>] [--pgm <地圖.yaml>]

run 目錄是 ab_run.sh 的輸出（run_traj.csv、manager.log、demo.log）。印出：
  - 這一段的送出 / 結束時間、結果、歷時
  - 這段時間內 controller_server / DWB 的所有 WARN / ERROR 訊息（demo.log）
  - 前一段是什麼、怎麼結束的、送出這一段時車子在哪裡
  - 失敗當下的位置、朝向（軌跡沒有朝向：用最後 0.20 m 的移動方向）、
    /odom 速度與 /cmd_vel 指令、車子從什麼時候開始不動
  - 該段起點與終點的淨空：離世界檔的牆（真實幾何）、離存檔地圖的佔據格與未知格
"""
import argparse, csv, math, os, re, sys
import numpy as np

RE_TS = re.compile(r'\[(\d+\.\d+)\] \[mower_manager\]: (.*)$')
RE_Q = re.compile(r'📋 佇列 \d+/\d+ \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) 終點 \(([-\d.]+), ([-\d.]+)\)')
RE_ANY_TS = re.compile(r'\[(WARN|ERROR)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$')


def world_dist_fn(world):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from coverage_budget import world_boxes
    boxes = world_boxes(world)

    def dist(x, y):
        best = 1e9
        for _n, cx, cy, yaw, sx, sy in boxes:
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx, ly = c * (x - cx) - s * (y - cy), s * (x - cx) + c * (y - cy)
            dx, dy = max(abs(lx) - sx / 2, 0), max(abs(ly) - sy / 2, 0)
            best = min(best, math.hypot(dx, dy))
        return best
    return dist


def map_dist_fn(yaml_path):
    import yaml, cv2
    y = yaml.safe_load(open(yaml_path))
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), y['image']), cv2.IMREAD_UNCHANGED)
    img = np.flipud(img)
    res = y['resolution']; ox, oy = y['origin'][0], y['origin'][1]
    occ = np.argwhere(img == 0); unk = np.argwhere(img == 205)

    def dist(x, y_, cells):
        if len(cells) == 0:
            return float('inf')
        d = np.hypot(ox + cells[:, 1] * res - x, oy + cells[:, 0] * res - y_)
        return float(d.min())
    return lambda x, y_: (dist(x, y_, occ), dist(x, y_, unk))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run'); ap.add_argument('label')
    ap.add_argument('--world', default=None); ap.add_argument('--pgm', default=None)
    a = ap.parse_args()

    ev, queue = [], {}
    for line in open(a.run + '/manager.log', encoding='utf-8', errors='replace'):
        m = RE_TS.search(line)
        if m:
            ev.append((float(m.group(1)), m.group(2)))
            q = RE_Q.search(m.group(2))
            if q:
                queue[q.group(1)] = tuple(float(q.group(k)) for k in range(2, 6))
    i_send = next((i for i, (t, s) in enumerate(ev) if ('送出任務 [%s]' % a.label) in s), None)
    if i_send is None:
        print('%s：沒有送出過 %s' % (a.run, a.label)); return
    t_send = ev[i_send][0]
    i_end = next((i for i in range(i_send, len(ev))
                  if ('✅ %s 完成' % a.label) in ev[i][1] or ('❌ [%s]' % a.label) in ev[i][1]), None)
    t_end, end_msg = ev[i_end] if i_end is not None else (None, '(沒有結束訊息)')
    # 前一段：往回找上一個「完成 / 失敗」
    prev = next(((t, s) for t, s in reversed(ev[:i_send])
                 if re.search(r'✅ .+ 完成|❌ \[', s)), (None, '(沒有)'))

    rows = [(float(r['t']), float(r['x']), float(r['y']), r['label'],
             float(r['odom_vx']), float(r['odom_wz']), float(r['cmd_vx']), float(r['cmd_wz']))
            for r in csv.DictReader(open(a.run + '/run_traj.csv'))]

    def at(t):
        return min(rows, key=lambda r: abs(r[0] - t))

    def heading_before(t):
        pts = [r for r in rows if r[0] <= t]
        j = len(pts) - 1
        while j > 0 and math.hypot(pts[-1][1] - pts[j][1], pts[-1][2] - pts[j][2]) < 0.20:
            j -= 1
        dx, dy = pts[-1][1] - pts[j][1], pts[-1][2] - pts[j][2]
        return math.degrees(math.atan2(dy, dx)) if math.hypot(dx, dy) > 0.05 else float('nan')

    print('=' * 70)
    print('%s  %s' % (os.path.basename(a.run.rstrip('/')), a.label))
    if a.label in queue:
        sx, sy, ex, ey = queue[a.label]
        print('規劃：起點 (%.2f, %.2f) -> 終點 (%.2f, %.2f)，方向 %.0f°'
              % (sx, sy, ex, ey, math.degrees(math.atan2(ey - sy, ex - sx))))
    print('送出 %.1f，結束 %s，歷時 %s' % (
        t_send, '%.1f' % t_end if t_end else '-', '%.1f s' % (t_end - t_send) if t_end else '-'))
    print('結束訊息：%s' % end_msg)
    print('前一段：%s' % prev[1])
    r0 = at(t_send)
    print('送出時車子在 (%.2f, %.2f)，移動方向 %.0f°，odom v=%.2f w=%.2f'
          % (r0[1], r0[2], heading_before(t_send), r0[4], r0[5]))
    if t_end:
        r1 = at(t_end)
        print('結束時車子在 (%.2f, %.2f)，最後 0.20 m 的移動方向 %.0f°，odom v=%.2f w=%.2f，cmd v=%.2f w=%.2f'
              % (r1[1], r1[2], heading_before(t_end), r1[4], r1[5], r1[6], r1[7]))
        seg = [r for r in rows if t_send <= r[0] <= t_end]
        # 從什麼時候開始不動：之後位置變化都 < 0.05 m
        still = None
        for r in reversed(seg):
            if math.hypot(r[1] - r1[1], r[2] - r1[2]) > 0.05:
                break
            still = r[0]
        if still:
            print('最後 %.1f s 車子移動 < 0.05 m（從送出後 %.1f s 起），這段時間 cmd v 平均 %.2f、w 平均 %.2f'
                  % (t_end - still, still - t_send,
                     np.mean([r[6] for r in seg if r[0] >= still]),
                     np.mean([r[7] for r in seg if r[0] >= still])))
        if a.label in queue:
            sx, sy, ex, ey = queue[a.label]
            L = math.hypot(ex - sx, ey - sy); ux, uy = (ex - sx) / L, (ey - sy) / L
            s_ = (r1[1] - sx) * ux + (r1[2] - sy) * uy
            d_ = -(r1[1] - sx) * uy + (r1[2] - sy) * ux
            print('結束位置在規劃直線上：沿線 %.2f m（全長 %.2f m），橫偏 %+.2f m' % (s_, L, d_))
        # controller 訊息
        msgs = {}
        for line in open(a.run + '/demo.log', encoding='utf-8', errors='replace'):
            m = RE_ANY_TS.search(line)
            if m and t_send - 0.2 <= float(m.group(2)) <= t_end + 0.2 and \
                    ('controller' in m.group(3) or 'DWB' in m.group(3) or 'costmap' in m.group(3)):
                k = '[%s] [%s] %s' % (m.group(1), m.group(3), m.group(4)[:120])
                msgs.setdefault(k, []).append(float(m.group(2)))
        print('這段時間 controller / DWB / costmap 的 WARN / ERROR：%s' % ('' if msgs else '(沒有)'))
        for k, ts in sorted(msgs.items(), key=lambda kv: kv[1][0]):
            print('   %3d 次  首次 +%.1f s  %s' % (len(ts), ts[0] - t_send, k))
    if a.label in queue and (a.world or a.pgm):
        sx, sy, ex, ey = queue[a.label]
        for name, (px, py) in (('起點', (sx, sy)), ('終點', (ex, ey))):
            parts = []
            if a.world:
                parts.append('離世界檔的牆 %.2f m' % world_dist_fn(a.world)(px, py))
            if a.pgm:
                do, du = map_dist_fn(a.pgm)(px, py)
                parts.append('離存檔地圖的佔據格 %.2f m、未知格 %.2f m' % (do, du))
            # 門檻由 vehicle.yaml 的 footprint 算（外接 / 內切半徑），同 coverage_budget 的讀法
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from coverage_budget import VEHICLE
            half_l, half_w = VEHICLE['footprint_length'] / 2.0, VEHICLE['footprint_width'] / 2.0
            print('%s淨空：%s（原地掉頭需 %.2f m，直線通過需 %.2f m）'
                  % (name, '，'.join(parts), math.hypot(half_l, half_w), half_w))


if __name__ == '__main__':
    main()
