"""obstacle_detour 的單元測試 (純計算，不需要 ROS)。

車身照實車：後輪軸前 0.975 m 是車頭、後面 0.25 m、半寬 0.42 m，側邊再加停止區的 0.05 m。
"""
import math

import numpy as np

from mowerbot_action import obstacle_detour as od
from mowerbot_action import route_plan

FRONT, REAR, HALF = 0.975, 0.25, 0.42


def line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]


def box_face(cx, cy, w=0.5, n=12):
    """光達看得到的箱子那一面 (垂直 x 軸的一條線)。"""
    return np.array([(cx, cy - w / 2 + w * k / (n - 1)) for k in range(n)])


def legs(cx, cy, gap=0.2, r=0.07, n=8):
    """人站著：兩條腿各一小圈點 (光達那個高度掃到的大腿)。"""
    pts = []
    for oy in (-gap / 2, gap / 2):
        for k in range(n):
            a = math.pi / 2 + math.pi * k / (n - 1)     # 朝車子那半圈
            pts.append((cx + r * math.cos(a), cy + oy + r * math.sin(a)))
    return np.array(pts)


def blocked(path, pose, pts, look=1.3):
    return od.blocked_ahead(path, pose, pts, FRONT, REAR, HALF, look=look)


def test_scan_to_map():
    pts = od.scan_to_map([1.0, float('inf'), 2.0, 0.01], -math.pi / 2, math.pi / 2, 0.05, 40.0,
                         (1.0, 2.0, math.pi / 2))
    # 光達朝 +y (yaw 90°)：第一束 -90° 是地圖的 0°、第三束 +90° 是地圖的 180°；inf 和太近的丟掉
    assert pts.shape == (2, 2)
    assert np.allclose(pts[0], (2.0, 2.0))
    assert np.allclose(pts[1], (-1.0, 2.0))


def test_box_on_the_line_is_seen_before_the_stop_zone():
    path = line(0, 0, 10, 0)
    gap, idx = blocked(path, (0.0, 0.0, 0.0), box_face(FRONT + 1.0, 0.0))
    assert gap is not None and abs(gap - 1.0) <= 0.1 + 1e-9
    assert len(idx) >= 3


def test_too_far_beside_and_behind_are_not_blocking():
    path = line(0, 0, 10, 0)
    assert blocked(path, (0.0, 0.0, 0.0), box_face(FRONT + 2.0, 0.0))[0] is None      # 超過 1.3 m
    assert blocked(path, (0.0, 0.0, 0.0), box_face(FRONT + 1.0, HALF + 0.05 + 0.3))[0] is None
    assert blocked(path, (2.0, 0.0, 0.0), box_face(0.5, 0.0))[0] is None                 # 已經開過去
    # 擦邊：箱子 (寬 0.5 m) 有 10 cm 壓進車身側邊 + 0.05 的範圍就算
    edge = HALF + 0.05 - 0.10 + 0.25
    assert blocked(path, (0.0, 0.0, 0.0), box_face(FRONT + 1.0, edge))[0] is not None


def test_person_legs_count():
    path = line(0, 0, 10, 0)
    gap, _idx = blocked(path, (0.0, 0.0, 0.0), legs(FRONT + 0.8, 0.1))
    assert gap is not None and gap <= 0.8 + 0.1


def test_few_noise_points_are_ignored():
    path = line(0, 0, 10, 0)
    two = np.array([(FRONT + 0.5, 0.0), (FRONT + 0.6, 0.0)])
    assert blocked(path, (0.0, 0.0, 0.0), two)[0] is None


def test_points_on_the_car_itself_are_ignored():
    path = line(0, 0, 10, 0)
    body = np.array([(0.3 + 0.05 * k, 0.3) for k in range(10)])   # 光達掃到自己的車架
    assert blocked(path, (0.0, 0.0, 0.0), body)[0] is None


def test_wall_after_the_end_of_the_swath_is_not_blocking():
    # 割草線終點離牆 1.6 m (光達停止區開著時的規則)：車子開到終點車頭離牆 0.625 m，不能當成擋路
    path = line(0, 0, 4, 0)
    wall = np.array([(4.0 + 1.6, -2.0 + 0.05 * k) for k in range(81)])
    for x in np.arange(0.0, 4.0, 0.25):
        assert blocked(path, (float(x), 0.0, 0.0), wall)[0] is None, x


def test_follows_the_curve_not_straight_ahead():
    # 沿邊的轉角：路線往左轉 90°，正前方 (不轉的話會撞到) 的牆不是擋路
    r = 2.0
    arc = [(r * math.sin(a), r - r * math.cos(a)) for a in np.linspace(0, math.pi / 2, 40)]
    path = ([(-2.0 + 0.1 * k, 0.0) for k in range(20)] + arc
            + [(r, r + 0.1 * k) for k in range(1, 30)])
    wall = np.array([(r + 1.0 + 0.4, -1.0 + 0.05 * k) for k in range(40)])   # 轉彎外側正前方
    assert blocked(path, (-0.5, 0.0, 0.0), wall)[0] is None
    # 同一個轉彎，箱子擺在彎道上就擋
    gap, _ = blocked(path, (-0.5, 0.0, 0.0), box_face(1.4, 0.6, w=0.4))
    assert gap is not None


def test_offset_from_the_line_is_also_checked():
    # 割草線開頭車子偏在線的左邊 0.3 m：箱子在線的左邊，照線走碰不到、照現在偏的位置會碰到
    path = line(0, 0, 10, 0)
    pts = box_face(FRONT + 1.0, 0.3 + HALF + 0.02, w=0.2)
    assert blocked(path, (0.0, 0.0, 0.0), pts)[0] is None
    assert blocked(path, (0.0, 0.3, 0.0), pts)[0] is not None


def test_cluster_joins_two_legs_not_the_wall():
    pts = np.vstack([legs(3.0, 0.0), np.array([(5.0, -2 + 0.05 * k) for k in range(80)])])
    seeds = np.arange(4)            # 第一條腿的幾個點
    c = od.cluster(pts, seeds)
    assert len(c) == len(legs(3.0, 0.0))
    assert c[:, 0].max() < 3.5


def test_solid_and_hole_keep_clearance():
    face = box_face(3.0, 0.0, w=0.5)
    sol = od.solid(face, (0.0, 0.0), depth=0.5)
    assert sol[:, 0].max() > 3.45              # 往後補了 0.5 m
    hole = od.hole_polygon(sol, 1.0)
    assert len(hole) >= 8
    hp = np.array(hole)
    # 多邊形每一條邊離原本的點都至少 1.0 m
    for i in range(len(hp)):
        a, b = hp[i], hp[(i + 1) % len(hp)]
        for t in np.linspace(0, 1, 11):
            q = a + t * (b - a)
            assert od.min_distance(sol, q) >= 1.0 - 1e-6


def test_rejoin_index():
    path = line(0, 0, 10, 0)
    i = od.rejoin_index(path, 4.0, lambda x, y: x >= 6.0)
    assert i is not None and abs(path[i][0] - 6.0) < 1e-9
    assert od.rejoin_index(path, 4.0, lambda x, y: x >= 9.8) is None      # 後面剩不到 0.5 m
    assert od.rejoin_index(path, 4.0, lambda x, y: False) is None


def test_detour_around_a_box_in_the_middle_of_the_lawn():
    """箱子擋在割草線中間：從車子停下來的位置繞到箱子後面同一條線上，整路離箱子 >= 1 m。"""
    outer = [(-1.2, -4.0), (14.0, -4.0), (14.0, 4.0), (-1.2, 4.0)]
    face = box_face(6.0, 0.0, w=0.5)
    sol = od.solid(face, (6.0 - 1.4, 0.0), depth=0.5)
    hole = od.hole_polygon(sol, 1.0)
    planner = route_plan.TransitPlanner([outer, hole], HALF + 0.05 + 0.05, 1.0616 + 0.05)
    path = line(0, 0, 12, 0)
    s_far = od.far_side(path, sol)
    i = od.rejoin_index(path, s_far, planner.turn_ok)
    assert i is not None
    rejoin = path[i]
    assert od.min_distance(sol, rejoin) >= 1.0 + 1.0616
    robot = (6.0 - FRONT - 1.1, 0.0)       # 車頭停在箱子前 1.1 m
    plan = planner.plan(robot, rejoin, 0.3)
    assert plan is not None and len(plan) >= 3
    for k in range(1, len(plan) - 1):       # 轉折點要掉得了頭
        assert planner.turn_ok(*plan[k])
    for a, b in zip(plan[:-1], plan[1:]):
        for t in np.linspace(0, 1, 50):
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if math.hypot(q[0] - robot[0], q[1] - robot[1]) < 0.3:
                continue        # 起點附近 (relax) 不檢查：車子本來就停在那裡
            assert od.min_distance(sol, q) >= 1.0 + HALF, q


def test_no_detour_when_the_lawn_is_too_narrow():
    outer = [(-1.2, -1.5), (14.0, -1.5), (14.0, 1.5), (-1.2, 1.5)]
    sol = od.solid(box_face(6.0, 0.0), (4.6, 0.0))
    hole = od.hole_polygon(sol, 1.0)
    planner = route_plan.TransitPlanner([outer, hole], HALF + 0.1, 1.0616 + 0.05)
    path = line(0, 0, 12, 0)
    i = od.rejoin_index(path, od.far_side(path, sol), planner.turn_ok)
    if i is not None:
        assert planner.plan((6.0 - FRONT - 1.1, 0.0), path[i], 0.3) is None


def test_segment_gap():
    sq = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    # 穿過去、從裡面出發都是 0
    assert od.segment_gap((-1.0, 0.5), (2.0, 0.5), sq) == 0.0
    assert od.segment_gap((0.5, 0.5), (3.0, 3.0), sq) == 0.0
    # 從旁邊經過：最近的那一邊的距離
    assert abs(od.segment_gap((-1.0, 1.3), (2.0, 1.3), sq) - 0.3) < 1e-6
    # 斜線經過角落
    g = od.segment_gap((3.0, 0.0), (0.0, 3.0), sq)
    assert abs(g - math.sqrt(0.5)) < 0.02
    # 只有一個點 / 沒有點
    assert abs(od.segment_gap((0.0, 1.0), (2.0, 1.0), [(1.0, 0.0)]) - 1.0) < 1e-6
    assert od.segment_gap((0.0, 0.0), (1.0, 0.0), []) == float('inf')
    # 11-03 模擬 (外框裡箱子沒繞成)：直線從箱子旁邊約 0.16 m 擦過去要看得出來
    sol = od.solid(box_face(6.3, 0.24), (4.0, 0.24))
    gap = od.segment_gap((4.9, 0.8), (9.0, 1.65), sol) - HALF
    assert gap < 1.0
