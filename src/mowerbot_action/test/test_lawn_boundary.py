"""lawn_boundary 的單元測試 (不需要 ROS，pytest 直接跑)。"""
import math

import numpy as np
import pytest

from mowerbot_action import lawn_boundary as lb


def square_track(x0, y0, side, step=0.1, gap=0.0):
    """沿正方形逆時針取樣的軌跡，最後一點離起點 gap 公尺 (模擬沒有剛好開回起點)。"""
    pts = []
    n = int(round(side / step))
    corners = [(x0, y0), (x0 + side, y0), (x0 + side, y0 + side), (x0, y0 + side)]
    for k in range(4):
        ax, ay = corners[k]
        bx, by = corners[(k + 1) % 4]
        for i in range(n):
            t = i / n
            pts.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    pts.append((x0, y0 + gap))
    return pts


def test_append_if_moved_skips_small_moves():
    track = []
    assert lb.append_if_moved(track, 0.0, 0.0)
    assert not lb.append_if_moved(track, 0.05, 0.0)
    assert lb.append_if_moved(track, 0.10, 0.0)
    assert track == [(0.0, 0.0), (0.10, 0.0)]


def test_square_closes_to_four_corners():
    res = lb.close_and_simplify(square_track(1.0, 2.0, 6.0, gap=0.3))
    poly = res['polygon']
    assert len(poly) == 4
    assert res['area'] == pytest.approx(36.0, rel=0.03)
    assert res['perimeter'] == pytest.approx(24.0, rel=0.03)
    assert res['gap'] == pytest.approx(0.3, abs=1e-6)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    assert min(xs) == pytest.approx(1.0, abs=0.06)
    assert max(xs) == pytest.approx(7.0, abs=0.06)
    assert min(ys) == pytest.approx(2.0, abs=0.06)
    assert max(ys) == pytest.approx(8.0, abs=0.06)


def test_noisy_track_simplifies_within_tolerance():
    rng = np.random.default_rng(0)
    track = [(x + rng.normal(0, 0.01), y + rng.normal(0, 0.01))
             for x, y in square_track(0.0, 0.0, 5.0)]
    res = lb.close_and_simplify(track)
    assert len(res['polygon']) <= 12
    assert res['area'] == pytest.approx(25.0, rel=0.05)


def test_open_track_is_rejected():
    with pytest.raises(ValueError, match='終點離起點'):
        lb.close_and_simplify(square_track(0.0, 0.0, 6.0, gap=1.5))


def test_too_few_points_is_rejected():
    with pytest.raises(ValueError, match='還不是一圈'):
        lb.close_and_simplify([(0.0, 0.0), (0.1, 0.0)])


def test_tiny_area_is_rejected():
    with pytest.raises(ValueError, match='面積'):
        lb.close_and_simplify(square_track(0.0, 0.0, 0.5, step=0.05))


def test_figure_eight_is_rejected():
    # (0,0)->(3,3)->(3,0)->(0,3)->(0,0) 是蝴蝶結 (8 字形)，兩條對角線在 (1.5, 1.5) 交叉
    corners = [(0.0, 0.0), (3.0, 3.0), (3.0, 0.0), (0.0, 3.0)]
    track = []
    for k in range(4):
        ax, ay = corners[k]
        bx, by = corners[(k + 1) % 4]
        n = int(math.hypot(bx - ax, by - ay) / 0.1)
        for i in range(n):
            t = i / n
            track.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    track.append((0.0, 0.05))
    with pytest.raises(ValueError, match='交叉'):
        lb.close_and_simplify(track, min_area=0.0)


def test_self_intersects_control_cases():
    assert not lb.self_intersects([(0, 0), (1, 0), (1, 1), (0, 1)])
    assert lb.self_intersects([(0, 0), (1, 1), (1, 0), (0, 1)])


def test_rasterize_matches_map_convention():
    # 地圖 200 x 200 格、0.05 m，原點 (-5, -5)：多邊形 (0,0)~(2,1) 應該落在
    # col 100~140、row 100~120 (row 對應 y、col 對應 x)
    poly = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    mask = lb.rasterize(poly, -5.0, -5.0, 0.05, 200, 200)
    rows, cols = np.nonzero(mask)
    assert cols.min() == 100 and cols.max() == 140
    assert rows.min() == 100 and rows.max() == 120
    assert mask[110, 120] == 255
    assert mask[110, 150] == 0      # x = 2.5 m，在外面
    assert mask[130, 120] == 0      # y = 1.5 m，在外面


def test_rasterize_empty_polygon_is_all_zero():
    assert not lb.rasterize([], 0.0, 0.0, 0.05, 10, 10).any()


def test_yaml_roundtrip(tmp_path):
    poly = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    path = tmp_path / 'lawn.yaml'
    lb.save_yaml(str(path), poly, 'map', stamp_sec=123.0)
    loaded, frame = lb.load_yaml(str(path))
    assert frame == 'map'
    assert loaded == poly
