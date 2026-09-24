#!/usr/bin/env python3
"""掉頭外擺的分布（報告 28 節【2】，只量測）。

用法:
    python3 test/tools/swath_overshoot.py <run 目錄> [<run 目錄> ...] [--csv 輸出.csv]

每個 run 目錄要有 ab_run.sh 產生的 run_traj.csv 與 manager.log。

對每一條割草線（軌跡上 label == 割草線 i/N 的第一段連續取樣）：
  規劃直線  = manager 最後一輪「📋 佇列」的起點 S -> 終點 E
  沿線距離 s = (p - S)·u，橫向偏差 d = (p - S)·n（u 為 S->E 單位向量，n 為其左法向量）
  外擺方向  以「進線當下車子在直線的哪一側」為準：往另一側偏為正
             （掉頭從上一條那一側進來，衝過頭就跑到另一側）
  最大橫偏  = 該段所有取樣中，往外擺方向的最大 d（沒有往外擺就取 0）
  收回距離  = 最大橫偏出現之後，第一個 |d| < 0.10 m 的取樣點的 s（沿線、從規劃起點起算）；
             到該段結束都沒有降到 0.10 m 以下記為「未收回」
  穩定距離  = 從這一點起到該段結束 |d| 都 < 0.10 m 的最小 s（收回之後可能反向越過 0.10 m）
  反向越線  = 收回之後往進線那一側的最大偏差
  掉頭角度  = 進線前最後 0.20 m 路徑的移動方向與 u 的夾角（軌跡沒有朝向，用位移方向代替。
             用路徑長而不用時間窗：上一段結尾車子幾乎停下，0.6 s 內位移常不到 0.05 m）
  進線橫偏  = 進線當下車子離規劃直線的距離
  前一段    = 軌跡上緊接在這一段之前的 label 與它的規劃長度（approach 用實際走的路徑長）

軌跡是 /odom 座標（coverage_run.py 的量法），佇列是 map 座標；存圖跑的
定位模式 map_start_pose = (0,0,0)，兩者差距就是定位修正量，這裡沒有扣。
"""
import csv, math, re, statistics, sys
from collections import OrderedDict

RE_TS = re.compile(r'\[(\d+\.\d+)\] \[mower_manager\]: (.*)$')
RE_Q = re.compile(r'📋 佇列 (\d+)/\d+ \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) '
                  r'終點 \(([-\d.]+), ([-\d.]+)\) 長度 ([\d.]+) m')
RE_OK = re.compile(r'✅ (.+?) 完成')
RE_FAIL = re.compile(r'❌ \[(.+?)\] 失敗')
RE_LEADIN = re.compile(r'🛬 (割草線 \d+/\d+) 加跑道')
RECOVER = 0.10


def load(run):
    queue, result, leadin = OrderedDict(), {}, set()
    pend = set()
    for line in open(run + '/manager.log', encoding='utf-8', errors='replace'):
        m = RE_TS.search(line)
        if not m:
            continue
        msg = m.group(2)
        mi = RE_LEADIN.search(msg)
        if mi:
            pend.add(mi.group(1))
        q = RE_Q.search(msg)
        if q:
            if int(q.group(1)) == 1:          # 新的一輪規劃
                queue.clear(); leadin, pend = pend, set()
            queue[q.group(2)] = tuple(float(q.group(k)) for k in range(3, 8))
        mo = RE_OK.search(msg)
        if mo and mo.group(1) not in result:
            result[mo.group(1)] = '完成'
        mf = RE_FAIL.search(msg)
        if mf and mf.group(1) not in result:
            result[mf.group(1)] = '失敗'
    traj = [(float(r['t']), float(r['x']), float(r['y']), r['label'])
            for r in csv.DictReader(open(run + '/run_traj.csv'))]
    # 依 label 切成連續段，保留順序
    segs = []
    for t, x, y, lb in traj:
        if segs and segs[-1][0] == lb:
            segs[-1][1].append((t, x, y))
        else:
            segs.append((lb, [(t, x, y)]))
    return queue, result, leadin, segs


def pathlen(pts):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(pts, pts[1:]))


def analyse(run):
    queue, result, leadin, segs = load(run)
    out, seen = [], set()
    for k, (lb, pts) in enumerate(segs):
        if not lb.startswith('割草線') or lb in seen or lb not in queue or len(pts) < 5:
            continue
        seen.add(lb)
        sx, sy, ex, ey, ln = queue[lb]
        L = math.hypot(ex - sx, ey - sy)
        ux, uy = (ex - sx) / L, (ey - sy) / L
        nx, ny = -uy, ux
        s = [(p[1] - sx) * ux + (p[2] - sy) * uy for p in pts]
        d = [(p[1] - sx) * nx + (p[2] - sy) * ny for p in pts]
        side = 1.0 if d[0] >= 0 else -1.0          # 進線時在哪一側
        out_d = [-side * v for v in d]             # 往另一側為正
        i_pk = max(range(len(out_d)), key=lambda i: out_d[i])
        pk = max(out_d[i_pk], 0.0)
        rec = None
        for i in range(i_pk, len(d)):
            if abs(d[i]) < RECOVER:
                rec = s[i]
                break
        settle = None
        for i in range(len(d) - 1, -1, -1):
            if abs(d[i]) >= RECOVER:
                settle = s[i + 1] if i + 1 < len(d) else None
                break
        else:
            settle = s[0]
        under = max([side * d[i] for i in range(i_pk, len(d))] + [0.0])
        # 進線前最後 0.20 m 路徑的移動方向
        prev = segs[k - 1] if k > 0 else None
        ang = None
        if prev:
            q = prev[1]
            j = len(q) - 1
            while j > 0 and math.hypot(q[-1][1] - q[j][1], q[-1][2] - q[j][2]) < 0.20:
                j -= 1
            dx, dy = q[-1][1] - q[j][1], q[-1][2] - q[j][2]
            if math.hypot(dx, dy) >= 0.10:
                c = (dx * ux + dy * uy) / math.hypot(dx, dy)
                ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        if prev:
            plb = prev[0]
            plen = queue[plb][4] if plb in queue else pathlen(prev[1])
        else:
            plb, plen = '-', None
        out.append(dict(
            run=run.rstrip('/').split('/')[-1], label=lb, result=result.get(lb, '?'),
            leadin=lb in leadin, sx=sx, sy=sy, heading=math.degrees(math.atan2(uy, ux)),
            peak=pk, peak_s=s[i_pk], recover_s=rec, settle_s=settle, under=under, turn=ang,
            entry_off=abs(d[0]), prev=plb.split(' ')[0] if plb != '-' else '-',
            prev_len=plen, length=L))
    return out


def pct(v, q):
    v = sorted(v)
    if not v:
        return float('nan')
    k = (len(v) - 1) * q
    f = int(math.floor(k)); c = min(f + 1, len(v) - 1)
    return v[f] + (v[c] - v[f]) * (k - f)


def spearman(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    if len(pairs) < 5:
        return float('nan'), len(pairs)
    def rank(v):
        o = sorted(range(len(v)), key=lambda i: v[i]); r = [0.0] * len(v)
        i = 0
        while i < len(o):
            j = i
            while j + 1 < len(o) and v[o[j + 1]] == v[o[i]]:
                j += 1
            for k in range(i, j + 1):
                r[o[k]] = (i + j) / 2.0
            i = j + 1
        return r
    ra, rb = rank([p[0] for p in pairs]), rank([p[1] for p in pairs])
    ma, mb = statistics.mean(ra), statistics.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = math.sqrt(sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb))
    return (num / den if den else float('nan')), len(pairs)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    csv_out = sys.argv[sys.argv.index('--csv') + 1] if '--csv' in sys.argv else None
    if csv_out in args:
        args.remove(csv_out)
    rows = []
    for r in args:
        rows += analyse(r)
    if csv_out:
        with open(csv_out, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
    ok = [r for r in rows if r['result'] == '完成']
    print('割草線樣本：%d 條（%d 趟），其中完成 %d 條；以下統計只算完成的'
          % (len(rows), len(args), len(ok)))
    pk = [r['peak'] for r in ok]
    rec = [r['recover_s'] for r in ok if r['recover_s'] is not None]
    n_norec = sum(1 for r in ok if r['recover_s'] is None)
    print('')
    print('                      n    中位數    p95     最大')
    print('最大橫偏 (m)       %4d   %6.3f  %6.3f  %6.3f' % (len(pk), pct(pk, .5), pct(pk, .95), max(pk)))
    print('收回到 <0.10 m 的沿線距離 (m) %4d   %6.3f  %6.3f  %6.3f   （另 %d 條到結束都沒收回）'
          % (len(rec), pct(rec, .5), pct(rec, .95), max(rec) if rec else float('nan'), n_norec))
    st = [r['settle_s'] for r in ok if r['settle_s'] is not None]
    print('穩定在 <0.10 m 的沿線距離 (m) %4d   %6.3f  %6.3f  %6.3f   （另 %d 條到結束都沒穩定）'
          % (len(st), pct(st, .5), pct(st, .95), max(st) if st else float('nan'), len(ok) - len(st)))
    un = [r['under'] for r in ok]
    print('收回後反向越線 (m)          %4d   %6.3f  %6.3f  %6.3f' % (len(un), pct(un, .5), pct(un, .95), max(un)))
    fr = [r['recover_s'] / r['length'] for r in ok if r['recover_s'] is not None]
    print('收回距離 / 割草線長度          %4d   %6.3f  %6.3f  %6.3f' % (len(fr), pct(fr, .5), pct(fr, .95), max(fr) if fr else float('nan')))
    ln = [r['length'] for r in ok]
    print('割草線長度 (m)              %4d   %6.3f  %6.3f  %6.3f' % (len(ln), pct(ln, .5), pct(ln, .95), max(ln)))
    ps = [r['peak_s'] for r in ok]
    print('最大橫偏出現的沿線位置 (m) %4d   %6.3f  %6.3f  %6.3f' % (len(ps), pct(ps, .5), pct(ps, .95), max(ps)))
    print('')
    print('與最大橫偏的 Spearman 相關（只算完成的）：')
    for name, key in (('掉頭角度', 'turn'), ('進線橫偏', 'entry_off'),
                      ('前一段長度', 'prev_len'), ('起點 x', 'sx'), ('起點 y', 'sy'),
                      ('割草線方向', 'heading'), ('割草線長度', 'length')):
        rho, n = spearman([r[key] for r in ok], pk)
        print('  %-10s rho = %+.2f  (n=%d)' % (name, rho, n))
    print('')
    print('依前一段的種類分組（完成的）：')
    for kind in sorted({r['prev'] for r in ok}):
        g = [r['peak'] for r in ok if r['prev'] == kind]
        print('  前一段 = %-8s n=%3d  最大橫偏 中位數 %.3f  最大 %.3f' % (kind, len(g), pct(g, .5), max(g)))
    for flag in (True, False):
        g = [r['peak'] for r in ok if r['leadin'] == flag]
        if g:
            print('  %s跑道        n=%3d  最大橫偏 中位數 %.3f  最大 %.3f'
                  % ('有' if flag else '無', len(g), pct(g, .5), max(g)))
    for hd in sorted({round(r['heading']) for r in ok}):
        g = [r for r in ok if round(r['heading']) == hd]
        gp = [r['peak'] for r in g]
        gr = [r['recover_s'] for r in g if r['recover_s'] is not None]
        print('  方向 %+4d°    n=%3d  最大橫偏 中位數 %.3f  最大 %.3f  收回距離 中位數 %.2f'
              % (hd, len(g), pct(gp, .5), max(gp), pct(gr, .5)))
    tb = [(r['turn'], r['peak']) for r in ok if r['turn'] is not None]
    print('')
    print('依掉頭角度分組（完成的）：')
    for lo, hi in ((0, 30), (30, 90), (90, 150), (150, 181)):
        g = [p for a, p in tb if lo <= a < hi]
        if g:
            print('  %3d ~ %3d°  n=%3d  最大橫偏 中位數 %.3f  p95 %.3f  最大 %.3f'
                  % (lo, hi, len(g), pct(g, .5), pct(g, .95), max(g)))
    bad = [r for r in rows if r['result'] != '完成']
    if bad:
        print('')
        print('沒有完成的割草線（不列入統計）：%s'
              % ', '.join('%s %s(%s)' % (r['run'], r['label'], r['result']) for r in bad))


if __name__ == '__main__':
    main()
