#!/usr/bin/env python3
"""間歇性失敗的實測發生率（報告 28 節【4】，只統計，不加任何重試）。

用法:
    python3 test/tools/flaky_census.py --suite <test/logs/日期目錄> ... [--demo <ab_run 輸出目錄> ...]

逐一掃 log，找三種已知的上游間歇性失敗，分母是「那個東西被啟動 / 呼叫了幾次」：

  7.12 Nav2 生命週期競態   每一份 *_navigation.log（套件）或 demo.log（run_demo.sh）
                           裡有沒有 "failed to send response to /controller_server/change_state"
  7.9  save_map 失敗       smoke_test.log / save.log 裡每一則 SaveMap_Response(result=N)，N != 0 算一次失敗
  7.13 gzserver 起不來     每一份 *_gazebo.log / demo.log 裡有沒有
                           "Service /spawn_entity unavailable" 或 "Spawn service failed"

另外列出每趟套件的 FAIL / SKIP 項目，讓「其他非預期失敗」可以逐項對照。
"""
import glob, os, re, sys

NAV_RACE = 'failed to send response to /controller_server/change_state'
GZ_FAIL = ('Service /spawn_entity unavailable', 'Spawn service failed')
RE_SAVE = re.compile(r'SaveMap_Response\(result=(\d+)\)')
RE_ITEM = re.compile(r'^\[(FAIL|SKIP)\] (\S+)\s+(.*?)(?:  ->  (.*))?$')


def read(p):
    try:
        return open(p, encoding='utf-8', errors='replace').read()
    except OSError:
        return ''


def main():
    suites, demos, cur = [], [], None
    for a in sys.argv[1:]:
        if a in ('--suite', '--demo'):
            cur = a
        elif cur == '--suite':
            suites.append(a.rstrip('/'))
        elif cur == '--demo':
            demos.append(a.rstrip('/'))
    tot = dict(nav=0, nav_bad=0, gz=0, gz_bad=0, save=0, save_bad=0)

    print('=== 完整套件 ===')
    for d in suites:
        navs = sorted(glob.glob(d + '/*_navigation.log'))
        gzs = sorted(glob.glob(d + '/*_gazebo.log'))
        nav_bad = [os.path.basename(f) for f in navs if NAV_RACE in read(f)]
        gz_bad = [os.path.basename(f) for f in gzs if any(s in read(f) for s in GZ_FAIL)]
        st = read(d + '/smoke_test.log')
        saves = [int(x) for x in RE_SAVE.findall(st)]
        items = [m.groups() for m in (RE_ITEM.match(l) for l in st.splitlines()) if m]
        total = re.search(r'總計\s+(\d+/\d+)', st)
        tot['nav'] += len(navs); tot['nav_bad'] += len(nav_bad)
        tot['gz'] += len(gzs); tot['gz_bad'] += len(gz_bad)
        tot['save'] += len(saves); tot['save_bad'] += sum(1 for x in saves if x != 0)
        print('%s  總計 %s' % (os.path.basename(d), total.group(1) if total else '(沒有總計：中途中止或超時)'))
        print('   Nav2 啟動 %d 次，競態 %d 次 %s' % (len(navs), len(nav_bad), nav_bad or ''))
        print('   Gazebo 啟動 %d 次，spawn 失敗 %d 次 %s' % (len(gzs), len(gz_bad), gz_bad or ''))
        print('   save_map 呼叫 %d 次，結果 %s' % (len(saves), saves))
        for kind, cid, name, obs in items:
            print('   [%s] %s %s  ->  %s' % (kind, cid, name[:50], (obs or '')[:120]))

    if demos:
        print('')
        print('=== run_demo.sh（ab_run.sh）===')
    for d in demos:
        dl = read(d + '/demo.log')
        nav_bad = NAV_RACE in dl
        gz_bad = any(s in dl for s in GZ_FAIL)
        saves = [int(x) for x in RE_SAVE.findall(read(d + '/save.log'))]
        tot['nav'] += 1; tot['nav_bad'] += int(nav_bad)
        tot['gz'] += 1; tot['gz_bad'] += int(gz_bad)
        tot['save'] += len(saves); tot['save_bad'] += sum(1 for x in saves if x != 0)
        print('%-40s Nav2 競態=%s  spawn 失敗=%s  save_map=%s'
              % (d.split('stage28/')[-1], nav_bad, gz_bad, saves or '-'))

    print('')
    print('=== 合計 ===')
    for name, a, b in (('7.12 Nav2 生命週期競態', 'nav_bad', 'nav'),
                       ('7.9  save_map 失敗', 'save_bad', 'save'),
                       ('7.13 gzserver 起不來', 'gz_bad', 'gz')):
        n, k = tot[b], tot[a]
        print('%-24s %d / %d 次  (%s)' % (name, k, n, '%.1f %%' % (100.0 * k / n) if n else '-'))


if __name__ == '__main__':
    main()
