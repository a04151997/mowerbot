"""swath_angle 的單元測試 (不需要 ROS，pytest 直接跑)。"""
import math

import pytest

from mowerbot_action import swath_angle as sa

SPACING = 0.3       # 刀盤 0.5 m、重疊 0.4
HEADLAND = 1.2      # vehicle_geometry.headland_width


def rect(w, h, deg, cx=3.0, cy=-2.0):
    """寬 w (沿 deg 方向)、高 h 的長方形，中心 (cx, cy)。"""
    pts = sa.rotate_xy([(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)],
                       math.radians(deg))
    return [(x + cx, y + cy) for x, y in pts]


def axis_diff(a_deg, b_deg):
    """兩個割草線方向差幾度 (方向不分正反，180° 一循環)。"""
    d = (a_deg - b_deg) % 180.0
    return min(d, 180.0 - d)


def test_parse_angle():
    assert sa.parse_angle('auto') is None
    assert sa.parse_angle(' AUTO ') is None
    assert sa.parse_angle('45') == pytest.approx(math.pi / 4)
    assert sa.parse_angle(90) == pytest.approx(math.pi / 2)
    assert sa.parse_angle('0') == 0.0
    for bad in ('abc', '', 'nan', 'inf'):
        with pytest.raises(ValueError):
            sa.parse_angle(bad)


def test_rotate_quat_adds_yaw():
    for yaw in (0.0, 0.7, math.pi, -2.0):
        for ang in (0.3, -1.1, math.pi / 4):
            q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
            x, y, z, w = sa.rotate_quat(*q, ang)
            got = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            assert axis_diff(math.degrees(got), math.degrees(yaw + ang)) < 1e-9
            assert x * x + y * y + z * z + w * w == pytest.approx(1.0)


@pytest.mark.parametrize('w,h,deg', [(20, 4, 0), (20, 4, 45), (20, 4, 30), (20, 6, -60),
                                     (40, 3.5, -33), (30, 4, 71), (12, 7, 100)])
def test_long_rectangle_follows_long_side(w, h, deg):
    a, info = sa.choose_angle([rect(w, h, deg)], SPACING, HEADLAND)
    assert axis_diff(math.degrees(a), deg) <= 1.0, info
    # 順著長邊：每條線一段，段數 = 線數
    assert info['segments'] == info['lines']


def test_axis_aligned_ties_go_to_zero_and_tall_goes_to_ninety():
    a, _ = sa.choose_angle([rect(20, 4, 0)], SPACING, HEADLAND)
    assert a == 0.0
    a, info = sa.choose_angle([rect(4, 20, 0)], SPACING, HEADLAND)
    assert math.degrees(a) == pytest.approx(90.0)
    assert info['segments'] < info['segments_at_0']


def test_rotating_boundary_by_minus_angle_aligns_it_with_x():
    """manager 把邊界轉 -angle 再交給 F2C (沿 x)：轉完之後再挑一次應該是 0°。"""
    ring = rect(25, 5, 37)
    a, _ = sa.choose_angle([ring], SPACING, HEADLAND)
    a2, _ = sa.choose_angle([sa.rotate_xy(ring, -a)], SPACING, HEADLAND)
    assert axis_diff(math.degrees(a2), 0.0) <= 1.0


def test_keepout_wall_decides_direction():
    """10 x 10 的正方形中間有一道南北向的禁區牆：沿 x 每條線被切成兩段，沿 y 只要繞開牆。"""
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    wall = [(4.5, 2), (5.5, 2), (5.5, 8), (4.5, 8)]
    a, info = sa.choose_angle([square, wall], SPACING, HEADLAND)
    assert axis_diff(math.degrees(a), 90.0) <= 1.0, info
    assert info['segments'] < info['segments_at_0']


def test_l_shape_picks_fewer_segments_than_diagonal():
    ring = [(0, 0), (20, 0), (20, 4), (4, 4), (4, 15), (0, 15)]
    a, info = sa.choose_angle([ring], SPACING, HEADLAND)
    field, _c, res = sa._rasterize([ring])
    diag = sa.score_angle(field, res, math.radians(45), SPACING, HEADLAND)[0]
    assert info['segments'] < diag
    assert axis_diff(math.degrees(a), 0.0) <= 1.0 or axis_diff(math.degrees(a), 90.0) <= 1.0


def test_too_small_after_headland_falls_back_to_zero():
    assert sa.choose_angle([rect(2, 2, 30)], SPACING, HEADLAND) == (0.0, None)
    assert sa.choose_angle([[(0, 0), (1, 0)]], SPACING, HEADLAND) == (0.0, None)
    assert sa.choose_angle([rect(20, 4, 30)], 0.0, HEADLAND) == (0.0, None)
