"""map_match：光達跟地圖對不對得上、在猜的位置附近找回真正的位置 (2026-10-21)。"""
import math

import numpy as np

from mowerbot_action import map_match

RES = 0.05


def room():
    """10 m x 8 m 的房間 (牆厚 0.10 m)，裡面一個 1 x 0.6 m 的箱子、一段 2 m 的隔牆 (不對稱，才找得到唯一解)。
    原點 (-1, -2)：地圖格子 row 0 在 origin 那側，跟 OccupancyGrid 一樣。"""
    ox, oy = -1.0, -2.0
    w, h = int(10 / RES), int(8 / RES)
    g = np.zeros((h, w), dtype=np.int8)
    g[:2, :] = 100
    g[-2:, :] = 100
    g[:, :2] = 100
    g[:, -2:] = 100

    def box(x0, y0, x1, y1):
        u0, u1 = int((x0 - ox) / RES), int((x1 - ox) / RES)
        v0, v1 = int((y0 - oy) / RES), int((y1 - oy) / RES)
        g[v0:v1, u0:u1] = 100

    box(4.0, 1.0, 5.0, 1.6)
    box(1.0, 3.0, 1.1, 5.0)
    return g, ox, oy


def fake_scan(g, ox, oy, pose, n=360, rmax=11.5):
    """從 pose (車子在地圖上的位置) 往四周打 n 條光，回傳車子座標系的點 (碰到牆的那一點)。"""
    x, y, yaw = pose
    pts = []
    for i in range(n):
        a = -math.pi + i * 2 * math.pi / n
        dx, dy = math.cos(yaw + a), math.sin(yaw + a)
        t = 0.1
        while t < rmax:
            u = int(math.floor((x + t * dx - ox) / RES))
            v = int(math.floor((y + t * dy - oy) / RES))
            if not (0 <= u < g.shape[1] and 0 <= v < g.shape[0]):
                break
            if g[v, u] >= 65:
                pts.append((t * math.cos(a), t * math.sin(a)))
                break
            t += 0.01
    return np.array(pts)


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def test_score_high_at_true_pose_and_low_when_off():
    g, ox, oy = room()
    field = map_match.MapField(g, RES, ox, oy)
    true = (2.0, 2.0, 0.3)
    pts = fake_scan(g, ox, oy, true)
    good, n = map_match.score(field, map_match.transform(pts, *true))
    assert n > 300
    assert good > 0.95
    off = map_match.transform(pts, true[0] + 0.8, true[1] - 0.5, true[2] + 0.25)
    bad, _ = map_match.score(field, off)
    assert bad < 0.4


def test_locate_recovers_pose_from_a_guess_1m_and_20deg_off():
    g, ox, oy = room()
    field = map_match.MapField(g, RES, ox, oy)
    true = (2.0, 2.0, 0.3)
    pts = fake_scan(g, ox, oy, true)
    r = map_match.locate(field, pts, (true[0] + 0.8, true[1] - 0.6, true[2] - math.radians(20)))
    assert math.hypot(r['x'] - true[0], r['y'] - true[1]) < 0.05
    assert angle_diff(r['yaw'], true[2]) < math.radians(1.5)
    assert r['score'] > 0.9
    assert 0.9 < r['moved'] < 1.1


def test_locate_keeps_a_good_guess():
    g, ox, oy = room()
    field = map_match.MapField(g, RES, ox, oy)
    true = (6.5, 4.0, -2.0)
    pts = fake_scan(g, ox, oy, true)
    r = map_match.locate(field, pts, true)
    assert r['moved'] < 0.05
    assert angle_diff(r['yaw'], true[2]) < math.radians(1.0)


def test_empty_scan_and_outside_map():
    g, ox, oy = room()
    field = map_match.MapField(g, RES, ox, oy)
    assert map_match.score(field, np.zeros((0, 2))) == (0.0, 0)
    r = map_match.locate(field, np.zeros((0, 2)), (1.0, 2.0, 0.5))
    assert r['n'] == 0 and r['score'] == 0.0 and (r['x'], r['y'], r['yaw']) == (1.0, 2.0, 0.5)
    # 地圖外面的點離牆「很遠」，不會算成對上
    d = field.distance_at(np.array([[-50.0, -50.0], [100.0, 3.0]]))
    assert (d > 5.0).all()


def test_scan_points_drops_invalid_and_far_returns():
    ranges = [1.0, float('inf'), float('nan'), 0.01, 39.5, 2.0]
    p = map_match.scan_points(ranges, 0.0, math.pi / 2, 0.15, 40.0)
    assert p.shape == (2, 2)
    assert np.allclose(p[0], [1.0, 0.0])
    a = 5 * math.pi / 2
    assert np.allclose(p[1], [2.0 * math.cos(a), 2.0 * math.sin(a)], atol=1e-9)
    many = map_match.scan_points([3.0] * 720, -math.pi, 2 * math.pi / 720, 0.15, 40.0)
    assert len(many) == map_match.MAX_POINTS
