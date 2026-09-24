#!/usr/bin/env python3
"""每一段結束時離終點多遠（報告 22.1 節的量法，25.4 節用它做前後對照）。

用法:
    python3 test/tools/segment_endpoint.py <prefix>_traj.csv <mower_manager log>

<prefix>_traj.csv 由 coverage_run.py 產生；manager log 是 ~/.ros/log/python3_*.log
裡有 [mower_manager] 的那一個（ab_run.sh 會複製成 <輸出目錄>/manager.log）。
前後對照請看 ab_run.sh 開頭的兩條規則（同一張地圖、對照組 checkout 舊 commit）。

ABORTED 的段落也列在「全部段落」表裡，但那只是車子沒去，
有意義的是「只算 SUCCEEDED」的表。

段落邊界：manager log 的「➡️ 送出任務」與下一個「✅ 完成 / ❌ 失敗」。
預定終點：佇列段落取「📋 佇列」的終點；approach 取它之後第一個
非 approach 段落的起點。結束位置：軌跡中時間最接近結果那一行的取樣點。
"""
import bisect, csv, math, re, statistics, sys
from collections import Counter

traj_path, log_path = sys.argv[1], sys.argv[2]
T, X, Y = [], [], []
for r in csv.DictReader(open(traj_path)):
    T.append(float(r['t'])); X.append(float(r['x'])); Y.append(float(r['y']))

RE_TS = re.compile(r'\[(\d+\.\d+)\] \[mower_manager\]: (.*)$')
RE_Q = re.compile(r'📋 佇列 \d+/\d+ \[(.+?)\] 起點 \(([-\d.]+), ([-\d.]+)\) '
                  r'終點 \(([-\d.]+), ([-\d.]+)\)')
RE_SEND = re.compile(r'➡️ 送出任務 \[(.+?)\]')
RE_OK = re.compile(r'✅ (.+?) 完成')
RE_FAIL = re.compile(r'❌ \[(.+?)\] 失敗 \(status=(\w+)\)')

queue, events, guard, summary = {}, [], [], None
for line in open(log_path, encoding='utf-8', errors='replace'):
    m = RE_TS.search(line)
    if not m:
        continue
    t, msg = float(m.group(1)), m.group(2)
    q = RE_Q.search(msg)
    if q:
        queue[q.group(1)] = tuple(map(float, q.groups()[1:]))
        continue
    s = RE_SEND.search(msg)
    if s:
        events.append(('send', t, s.group(1))); continue
    f = RE_FAIL.search(msg)
    if f:
        events.append(('res', t, f.group(1), f.group(2))); continue
    o = RE_OK.search(msg)
    if o and 'Nav2 已接受' not in msg:
        events.append(('res', t, o.group(1), 'SUCCEEDED')); continue
    if '未改善距離' in msg or '達到上限' in msg:
        guard.append(msg)
    if '🏁' in msg:
        summary = msg

segs = []                      # (label, t_send, t_end, status)
cur = None
for e in events:
    if e[0] == 'send':
        cur = (e[2], e[1])
    elif cur is not None:
        segs.append((cur[0], cur[1], e[1], e[3]))
        cur = None


def idx(t):
    i = bisect.bisect_left(T, t)
    if i >= len(T):
        return len(T) - 1
    if i > 0 and abs(T[i-1] - t) < abs(T[i] - t):
        return i - 1
    return i


def kind(label):
    if label == 'approach':
        return 'approach'
    return label.split()[0]


rows = []
for k, (label, t0, t1, st) in enumerate(segs):
    if label == 'approach':
        nxt = next((s[0] for s in segs[k+1:] if s[0] != 'approach'), None)
        if nxt is None or nxt not in queue:
            continue
        gx, gy = queue[nxt][0], queue[nxt][1]
    else:
        if label not in queue:
            continue
        gx, gy = queue[label][2], queue[label][3]
    i0, i1 = idx(t0), idx(t1)
    end = math.hypot(X[i1] - gx, Y[i1] - gy)
    near = min(math.hypot(X[i] - gx, Y[i] - gy) for i in range(i0, i1 + 1))
    rows.append((kind(label), label, st, end, near))


def p95(v):
    v = sorted(v)
    return v[min(len(v) - 1, int(math.ceil(0.95 * len(v))) - 1)]


def table(title, rs):
    print('\n' + title)
    print('%-8s %4s %8s %8s %8s %8s %8s %9s %9s' % (
        '類別', 'n', '中位數', 'p95', '最大', '>0.10m', '>0.30m', '最近中位', '平均飄移'))
    for c in ('全部', '割草線', 'approach', '周邊環繞'):
        v = [r for r in rs if c == '全部' or r[0] == c]
        if not v:
            continue
        e = [r[3] for r in v]
        print('%-8s %4d %8.3f %8.3f %8.3f %7.1f%% %7.1f%% %9.3f %9.3f' % (
            c, len(v), statistics.median(e), p95(e), max(e),
            100 * sum(x > 0.10 for x in e) / len(e),
            100 * sum(x > 0.30 for x in e) / len(e),
            statistics.median(r[4] for r in v),
            statistics.mean(r[3] - r[4] for r in v)))


table('== 全部段落', rows)
table('== 只算 SUCCEEDED', [r for r in rows if r[2] == 'SUCCEEDED'])
print('\n> 0.30 m 的段落:')
for r in rows:
    if r[3] > 0.30:
        print('   %-14s %-10s 結束 %.3f m  最近 %.3f m' % (r[1], r[2], r[3], r[4]))

print('\n狀態分布:', dict(Counter((r[0], r[2]) for r in rows)))
labels = [s[0] for s in segs]
chains, n = Counter(), 0
for lab in labels + ['']:
    if lab == 'approach':
        n += 1
    elif n:
        chains[n] += 1; n = 0
print('送出段數 %d，approach 送出 %d 次；連續 approach 長度分布 %s'
      % (len(segs), labels.count('approach'),
         ', '.join('%d×%d' % (k, v) for k, v in sorted(chains.items()))))
print('守門觸發 %d 次' % len(guard))
for g in guard:
    print('   ' + g)
print('摘要:', summary)
