"""停下來之後接著割 (mission_resume.py)：從哪裡接、存檔讀檔、禁區外框改過了沒。"""
import array
import json
import math
import os
import time

from mowerbot_action import mission_resume as mr


def _line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    a = math.atan2(y1 - y0, x1 - x0)
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, a) for i in range(n + 1)]


def _swath(label, x0, x1, y, lead=0.5):
    """跑道 lead m + 割草線 (x0 -> x1)，跟 manager.build_lead_in 一樣跑道的點不含 A 本身。"""
    d = 1.0 if x1 > x0 else -1.0
    run = _line(x0 - d * lead, y, x0, y)[:-1] if lead > 0 else []
    return mr.make_entry(label, run + _line(x0, y, x1, y), lead=len(run))


def _plan():
    es = [mr.make_entry('周邊環繞 1/2', _line(0, 0, 10, 0)),
          mr.make_entry('周邊環繞 2/2', _line(10, 0, 10, 6)),
          _swath('割草線 1/3', 1, 9, 1),
          _swath('割草線 2/3', 9, 1, 1.3),
          _swath('割草線 3/3', 1, 9, 1.6),
          mr.make_entry('沿邊 1/3', _line(2, 3, 2, 5)),
          mr.make_entry('沿邊 2/3', _line(2, 5, 8, 5), cont=True),
          mr.make_entry('沿邊 3/3', _line(8, 5, 8, 3), cont=True)]
    return es


def test_kind_of():
    assert mr.kind_of('周邊環繞 3/4') == 'perimeter'
    assert mr.kind_of('割草線 12/40') == 'swath'
    assert mr.kind_of('沿邊 2/9') == 'edge'
    assert mr.kind_of('沿邊退出') is None
    assert mr.kind_of('approach') is None
    assert mr.label_index('割草線 12/40') == (12, 40)
    assert mr.label_index('approach') == (1, 1)


def test_nothing_done_keeps_everything():
    es = _plan()
    rest, info = mr.resume_entries(es, 0, None)
    assert [e['label'] for e in rest] == [e['label'] for e in es]
    assert info['from'] == '周邊環繞 1/2' and info['cut_m'] == 0.0
    # 不改原本的 list
    rest[0]['pts'].pop()
    assert len(es[0]['pts']) == 101


def test_stop_in_the_middle_of_a_swath_trims_and_releads():
    es = _plan()
    # 割草線 2/3 從 x=9 往 x=1 割，停在 x=5 (割了 4 m)
    rest, info = mr.resume_entries(es, 3, (5.0, 1.32))
    first = rest[0]
    assert first['label'] == '割草線 2/3' and first['relead'] and first['lead'] == 0
    # 從停的地方往回 0.3 m 開始：x = 5.3
    assert abs(first['pts'][0][0] - 5.3) < 0.051 and first['pts'][-1][0] == 1.0
    assert abs(info['cut_m'] - 3.7) < 0.051
    assert [e['label'] for e in rest[1:]] == ['割草線 3/3', '沿邊 1/3', '沿邊 2/3', '沿邊 3/3']


def test_stop_in_the_lead_in_keeps_the_whole_swath():
    es = _plan()
    # 還在跑道上 (x=9.3，割草線從 9 開始)
    rest, info = mr.resume_entries(es, 3, (9.3, 1.3))
    assert rest[0]['pts'] == es[3]['pts'] and not rest[0].get('relead')
    assert rest[0]['lead'] == es[3]['lead'] > 0 and info['cut_m'] == 0.0
    # 剛進割草線不到 0.3 m：一樣整條重割 (不用為了 0.2 m 重接跑道)
    rest, _ = mr.resume_entries(es, 3, (8.8, 1.3))
    assert rest[0]['pts'] == es[3]['pts']


def test_car_far_from_the_segment_keeps_it_whole():
    es = _plan()
    # 停在轉場途中 (離割草線 2 m)：那一段還沒開始
    rest, info = mr.resume_entries(es, 3, (5.0, 3.3))
    assert rest[0]['pts'] == es[3]['pts'] and info['cut_m'] == 0.0


def test_almost_done_segment_counts_as_done():
    es = _plan()
    rest, info = mr.resume_entries(es, 3, (1.2, 1.3))
    assert info['dropped'] and rest[0]['label'] == '割草線 3/3'
    assert info['from'] == '割草線 3/3'


def test_perimeter_trims_without_lead():
    es = _plan()
    rest, info = mr.resume_entries(es, 0, (6.0, 0.05))
    assert rest[0]['label'] == '周邊環繞 1/2' and not rest[0].get('relead')
    assert abs(rest[0]['pts'][0][0] - 5.7) < 0.051 and abs(info['cut_m'] - 5.7) < 0.051


def test_stop_in_edge_loop_restarts_the_loop():
    es = _plan()
    rest, info = mr.resume_entries(es, 6, (5.0, 5.0))
    assert [e['label'] for e in rest] == ['沿邊 1/3', '沿邊 2/3', '沿邊 3/3']
    assert info['loop_back'] == 1 and rest[0]['cont'] is False
    assert rest[0]['pts'] == es[5]['pts']
    # 在那一圈的第一段：也從頭，不從中間切 (離邊太近，中間不能原地轉身)
    rest, info = mr.resume_entries(es, 5, (2.0, 4.0))
    assert rest[0]['pts'] == es[5]['pts'] and info['loop_back'] == 0 and info['cut_m'] == 0.0


def test_all_done():
    es = _plan()
    rest, info = mr.resume_entries(es, len(es), None)
    assert rest == [] and info['from'] is None


def test_save_load_round_trip(tmp_path):
    base = str(tmp_path)
    es = _plan()
    rings = [[(0, 0), (10, 0), (10, 6), (0, 6)]]
    plan = mr.new_plan('S1', es, rings, {'keepout': 'abc', 'lawn': ''})
    cur = mr.new_cursor(plan)
    cur.update(k=3, xy=[5.0, 1.32], succeeded=4,
               skipped=[['割草線 1/3', [1.0, 1.0], 'ABORTED']], state='stopped', why='按了停止')
    pp, cp = mr.paths('S1', base)
    mr.write_text(pp, json.dumps(plan))
    mr.write_text(cp, json.dumps(cur))
    got = mr.load('S1', base)
    assert got is not None
    p2, c2 = got
    assert p2['entries'] == plan['entries'] and c2['k'] == 3 and c2['xy'] == [5.0, 1.32]
    assert p2['rings'] == [[[0, 0], [10, 0], [10, 6], [0, 6]]]
    # 別張地圖的不拿
    assert mr.load('S2', base) is None
    # 進度檔是上一趟的 (路線檔換新了、進度檔還沒寫)：當作從頭
    plan2 = dict(plan, id='other')
    mr.write_text(pp, json.dumps(plan2))
    assert mr.load('S1', base)[1]['k'] == 0
    # 路線檔壞了
    with open(pp, 'w') as fh:
        fh.write('{"version": 1, "sess')
    assert mr.load('S1', base) is None
    mr.remove('S1', base)
    assert not os.path.exists(pp) and not os.path.exists(cp)
    mr.remove('S1', base)       # 沒有檔也不出錯


def test_summary_counts():
    es = _plan()
    plan = mr.new_plan('S1', es, [], {},
                       carry={'succeeded': 5, 'skipped': [['割草線 9/9', None, 'x']]})
    cur = mr.new_cursor(plan)
    assert cur['skipped'] == [['割草線 9/9', None, 'x']]
    cur.update(k=3, xy=[5.0, 1.32], succeeded=4, state='stopped', why='按了停止')
    s = mr.summary(plan, cur)
    assert s['available'] and s['next'] == '割草線 2/3' and s['left'] == 5
    assert s['swaths_left'] == 2 and s['edge_left'] == 3
    assert s['done'] == 9 and s['skipped'] == 1 and s['total'] == 15
    assert s['why'] == '按了停止' and abs(s['cut_m'] - 3.7) < 0.051
    # 剩下的長度：割草線 2/3 從 5.3 到 1 (4.3 m) + 割草線 3/3 跑道 0.5 + 8 m + 沿邊 2 + 6 + 2
    assert abs(s['left_m'] - (4.3 + 8.5 + 10.0)) < 0.2
    cur['k'] = len(es)
    assert not mr.summary(plan, cur)['available']
    # 進度檔寫著「在跑」= 沒有正常停下來 (關電)
    cur2 = mr.new_cursor(plan)
    assert mr.summary(plan, cur2)['why'] == '關電或程式中斷'


def test_mask_fingerprint():
    data = [0] * 100
    assert mr.mask_fingerprint(0.05, 0, 0, 10, 10, data) == ''
    # 開機時 keepout_server 先發的 1x1 空遮罩也是「沒有禁區」
    assert mr.mask_fingerprint(0.05, 0, 0, 1, 1, [0]) == ''
    data[37] = 100
    a = mr.mask_fingerprint(0.05, -3.2, 1.5, 10, 10, data)
    assert a and a == mr.mask_fingerprint(0.05, -3.2, 1.5, 10, 10, array.array('b', data))
    assert a == mr.mask_fingerprint(0.05, -3.2, 1.5, 10, 10, bytes(data))
    data[38] = 100
    assert mr.mask_fingerprint(0.05, -3.2, 1.5, 10, 10, data) != a
    assert mr.mask_fingerprint(0.05, -3.25, 1.5, 10, 10, data[:]) != a


def test_poly_fingerprint_and_changed():
    p = [(0.0, 0.0), (5.0, 0.0), (5.0, 4.0)]
    assert mr.poly_fingerprint([]) == ''
    assert mr.poly_fingerprint(p) == mr.poly_fingerprint([(0.001, 0.0), (5.0, 0.002), (5.0, 4.0)])
    assert mr.poly_fingerprint(p) != mr.poly_fingerprint(p[:2] + [(5.0, 4.5)])
    assert mr.changed({'keepout': 'a', 'lawn': 'b'}, {'keepout': 'a', 'lawn': 'b'}) == []
    assert mr.changed({'keepout': 'a', 'lawn': ''}, {'keepout': 'c', 'lawn': ''}) == ['禁區']
    assert mr.changed({}, {'keepout': '', 'lawn': 'x'}) == ['外框']
    assert mr.changed(None, None) == []


def test_prune_keeps_newest(tmp_path):
    base = str(tmp_path)
    for i in range(5):
        pp, cp = mr.paths('S%d' % i, base)
        mr.write_text(pp, '{}')
        mr.write_text(cp, '{}')
        t = time.time() - 100 + i
        os.utime(pp, (t, t))
    mr.prune(base, keep=2)
    left = sorted(os.listdir(base))
    assert left == ['S3.cur.json', 'S3.plan.json', 'S4.cur.json', 'S4.plan.json']
