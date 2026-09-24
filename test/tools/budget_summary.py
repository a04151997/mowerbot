#!/usr/bin/env python3
"""多趟 coverage_budget.py 輸出的彙總（報告 28 節【1】【3】）。

用法:
    python3 test/tools/budget_summary.py <run 目錄> [<run 目錄> ...]

每個 run 目錄要有 budget.txt（coverage_budget.py 的輸出）與 coverage_run.log。
印出每趟的 A/B/C/D/E1/E2a/E2b/E3 面積與三個覆蓋率，以及中位數與全距 (min~max)。
"""
import re, statistics, sys

KEYS = [('A', r'^A\s+草坪總面積.*?([\d.]+) m²'),
        ('B', r'^B\s+外圈.*?([\d.]+) m²'),
        ('C', r'^C\s+割草線.*?([\d.]+) m²'),
        ('D', r'^D\s+重疊.*?([\d.]+) m²'),
        ('E', r'^E\s+未覆蓋.*?([\d.]+) m²'),
        ('E1', r'^\s+E1\s.*?([\d.]+) m²'),
        ('E2a', r'^\s+E2a\s.*?([\d.]+) m²'),
        ('E2b', r'^\s+E2b\s.*?([\d.]+) m²'),
        ('E3', r'^\s+E3\s.*?([\d.]+) m²'),
        ('覆蓋率', r'覆蓋率 = \(B\+C−D\)/A\s+([\d.]+) %'),
        ('扣E1', r'扣掉 E1 之後的覆蓋率\s+([\d.]+) %'),
        ('可割', r'規劃上可割的部分\) 的覆蓋率\s+([\d.]+) %')]
RE_END = re.compile(r'任務結束：state=(\d+) 完成 (\d+)/(\d+) 跳過 (\d+)，耗時 (\d+) s')


def main():
    rows = []
    for d in sys.argv[1:]:
        try:
            txt = open(d + '/budget.txt', encoding='utf-8').read()
        except OSError:
            print('%s：沒有 budget.txt' % d)
            continue
        r = {'run': d.rstrip('/').split('/')[-1]}
        for k, pat in KEYS:
            m = re.search(pat, txt, re.M)
            r[k] = float(m.group(1)) if m else None
        m = re.search(r'被跳過的段落：(.*)', txt)
        r['skipped'] = m.group(1).strip() if m else '-'
        try:
            m = RE_END.search(open(d + '/coverage_run.log', encoding='utf-8').read())
        except OSError:
            m = None
        r['end'] = ('state=%s 完成 %s/%s 跳過 %s %ss' % m.groups()) if m else '(沒有結束訊息)'
        rows.append(r)
    cols = [k for k, _ in KEYS]
    print('| 趟 | ' + ' | '.join(cols) + ' | 結局 |')
    print('|---' * (len(cols) + 2) + '|')
    for r in rows:
        print('| %s | ' % r['run'] + ' | '.join(
            '-' if r[k] is None else ('%.2f' % r[k]) for k in cols) + ' | %s |' % r['end'])
    print('')
    print('| 項目 | 中位數 | 全距 (min ~ max) | 全距寬 |')
    print('|---|---|---|---|')
    for k in cols:
        v = [r[k] for r in rows if r[k] is not None]
        if v:
            print('| %s | %.2f | %.2f ~ %.2f | %.2f |'
                  % (k, statistics.median(v), min(v), max(v), max(v) - min(v)))
    print('')
    for r in rows:
        print('%s 被跳過：%s' % (r['run'], r['skipped']))


if __name__ == '__main__':
    main()
