"""map_session 的純函式：地圖的 session、起點的順序、上次的位置、存好的地圖清單 (2026-10-21)。"""
import math
import os

import pytest

from mowerbot_action import map_session


def touch(path, text='x', mtime=None):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    if mtime is not None:
        os.utime(path, (mtime, mtime))


def test_session_of_reads_the_session_file_or_falls_back_to_the_name(tmp_path):
    m = str(tmp_path / 'lab1')
    assert map_session.session_of(m) == 'map_lab1'
    touch(m + '.session', '20261018_101500_abc123\n')
    assert map_session.session_of(m) == '20261018_101500_abc123'
    touch(m + '.session', '  \n')
    assert map_session.session_of(m) == 'map_lab1'


def test_check_saved_map_lists_missing_files(tmp_path):
    m = str(tmp_path / 'lab1')
    assert map_session.check_saved_map(m) == ['posegraph', 'data']
    touch(m + '.posegraph')
    assert map_session.check_saved_map(m) == ['data']
    touch(m + '.data')
    assert map_session.check_saved_map(m) == []


def test_parse_start_pose():
    assert map_session.parse_start_pose('') is None
    assert map_session.parse_start_pose('  ') is None
    x, y, yaw = map_session.parse_start_pose('1.5,-2,90')
    assert (x, y) == (1.5, -2.0) and yaw == pytest.approx(math.pi / 2)
    assert map_session.parse_start_pose('1 2 -45')[2] == pytest.approx(-math.pi / 4)
    for bad in ('1,2', '1,2,3,4', 'a,b,c', 'nan,0,0'):
        with pytest.raises(ValueError):
            map_session.parse_start_pose(bad)


def test_last_pose_round_trip_and_broken_file(tmp_path):
    f = str(tmp_path / 'sub' / 'last.yaml')
    assert map_session.read_last_pose(f) is None
    map_session.write_last_pose(f, '20261018_101500_abc123', 1.23456, -2.5, 0.5, 'mapping')
    d = map_session.read_last_pose(f)
    assert d['session'] == '20261018_101500_abc123'
    assert (d['x'], d['y']) == (1.2346, -2.5) and d['yaw'] == pytest.approx(0.5)
    assert not os.path.exists(f + '.tmp')
    touch(f, 'session: [unclosed')
    assert map_session.read_last_pose(f) is None
    touch(f, 'just text')
    assert map_session.read_last_pose(f) is None


def test_all_digit_session_stays_a_string(tmp_path):
    f = str(tmp_path / 'last.yaml')
    map_session.write_last_pose(f, '20261018', 0.0, 0.0, 0.0, 'mapping')
    assert map_session.read_last_pose(f)['session'] == '20261018'


def test_start_pose_order(tmp_path):
    f = str(tmp_path / 'last.yaml')
    assert map_session.start_pose('s1', '', f)[:3] == (0.0, 0.0, 0.0)
    map_session.write_last_pose(f, 's1', 3.0, 4.0, 1.0, 'localization')
    x, y, yaw, why = map_session.start_pose('s1', '', f)
    assert (x, y, yaw) == (3.0, 4.0, 1.0) and '上次' in why
    # 別張地圖記的位置不用
    assert map_session.start_pose('s2', '', f)[:3] == (0.0, 0.0, 0.0)
    # car-start 指定的最優先
    x, y, yaw, why = map_session.start_pose('s1', '-1,2,180', f)
    assert (x, y) == (-1.0, 2.0) and yaw == pytest.approx(math.pi) and '指定' in why


def test_list_saved_maps(tmp_path, monkeypatch):
    maps = tmp_path / 'maps'
    maps.mkdir()
    ko, lawn = tmp_path / 'ko', tmp_path / 'lawn'
    ko.mkdir()
    lawn.mkdir()
    monkeypatch.setattr(map_session, 'KEEPOUT_SESSIONS_DIR', str(ko))
    monkeypatch.setattr(map_session, 'LAWN_DIR', str(lawn))
    for name, t in (('old', 1000), ('new', 3000), ('mid', 2000)):
        touch(str(maps / (name + '.posegraph')), mtime=t)
        touch(str(maps / (name + '.data')))
    touch(str(maps / 'broken.posegraph'), mtime=4000)       # 沒有 .data：不能用
    touch(str(maps / 'new.session'), 'sid_new')
    touch(str(ko / 'sid_new.yaml'))
    touch(str(lawn / 'map_mid.yaml'))
    got = map_session.list_saved_maps(str(maps))
    assert [m['name'] for m in got] == ['new', 'mid', 'old']
    assert got[0]['session'] == 'sid_new' and got[0]['keepout'] and not got[0]['lawn']
    assert got[1]['session'] == 'map_mid' and got[1]['lawn'] and not got[1]['keepout']
    assert got[0]['path'] == str(maps / 'new')
    assert [m['name'] for m in map_session.list_saved_maps(str(maps), limit=2)] == ['new', 'mid']
    assert map_session.list_saved_maps(str(tmp_path / 'nope')) == []
    assert map_session.list_saved_maps('') == []
