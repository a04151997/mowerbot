"""mission_time：路線幾何與預計時間 (2026-10-21)。"""
import math

import pytest

from mowerbot_action import mission_time as mt


def line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def test_back_and_forth_swaths():
    """弓字形：第二條在第一條旁邊 0.3 m、方向相反 -> 不插轉場、原地轉 180°。"""
    segs = mt.plan_geometry([(line(0, 0, 5, 0), False), (line(5, 0.3, 0, 0.3), False)],
                            (0.0, 0.0, 0.0))
    a = segs[0]
    assert a[1] == 0.0 and a[3] == pytest.approx(0.0, abs=1e-6)
    assert a[4] == pytest.approx(5.0, abs=1e-6) and a[5] == 0.0
    b = segs[1]
    assert b[1] == 0.0 and b[3] == pytest.approx(math.pi, abs=0.02)
    assert mt.route_meters(segs) == pytest.approx(10.0, abs=1e-6)


def test_approach_and_transit():
    """離下一段起點 >= 0.5 m：先開過去 (照 transit_plan 的折線)，轉場前、到了都原地對正。"""
    calls = []

    def plan(a, b):
        calls.append((a, b))
        return [a, (a[0], b[1]), b]

    segs = mt.plan_geometry([(line(3, 4, 3, 8), False)], (0.0, 0.0, 0.0), plan)
    s = segs[0]
    assert len(calls) == 1
    assert s[0] == pytest.approx(math.pi / 2, abs=1e-6)        # 轉向 +y
    assert s[1] == pytest.approx(7.0, abs=1e-6)                # 4 + 3
    assert s[2] == pytest.approx(math.pi / 2, abs=1e-6)        # 轉場中間轉一次
    assert s[3] == pytest.approx(math.pi / 2, abs=1e-6)        # 到了轉回 +y
    near = mt.plan_geometry([(line(0.3, 0, 3, 0), False)], (0.0, 0.0, 0.0))
    assert near[0][1] == 0.0                                    # 0.3 m：不插轉場


def test_edge_continuation_turns_while_moving():
    """沿邊「接著上一段」：不原地對正，轉角算在邊走邊轉。"""
    first = line(0, 0, 4, 0)
    second = line(4, 0, 4, 4)
    on = mt.plan_geometry([(first, False), (second, True)], (0.0, 0.0, 0.0))
    off = mt.plan_geometry([(first, False), (second, False)], (0.0, 0.0, 0.0))
    assert on[1][3] == 0.0 and on[1][5] == pytest.approx(math.pi / 2, abs=1e-6)
    assert off[1][3] == pytest.approx(math.pi / 2, abs=1e-6) and off[1][5] == 0.0


def test_turning_counts_corners_not_noise():
    sq = line(0, 0, 4, 0)[:-1] + line(4, 0, 4, 4)[:-1] + line(4, 4, 0, 4)
    assert mt.turning(sq) == pytest.approx(math.pi, abs=0.05)
    wobble = [(0.1 * i, 0.002 * (-1) ** i) for i in range(50)]
    assert mt.turning(wobble) == 0.0


def test_seconds_scale_with_speed():
    seg = [0.0, 0.0, 0.0, math.pi, 10.0, 0.0]
    t25, t50 = mt.seconds(seg, 0.25), mt.seconds(seg, 0.5)
    assert t25 > t50 > 0
    # 直線 10 m：0.5 時速度上限 0.35 m/s
    a_base, a_k, d_base = mt.CALIBRATION[-1][1]
    assert t50 == pytest.approx(a_base + math.pi / (a_k * 0.6)
                                + d_base + 10.0 / (mt.DRIVE_K * 0.35), rel=1e-9)
    assert mt.align_seconds(0.05, 0.5) == 0.0          # align_goal_checker 的容許以內不轉


def test_calibration_interpolates_and_clamps():
    lo, hi = mt.CALIBRATION[0][1], mt.CALIBRATION[-1][1]
    assert mt.calibration(0.10) == pytest.approx(lo)
    assert mt.calibration(1.00) == pytest.approx(hi)
    mid = mt.calibration((mt.CALIBRATION[0][0] + mt.CALIBRATION[-1][0]) / 2)
    assert mid == pytest.approx(tuple((a + b) / 2 for a, b in zip(lo, hi)))
    # 慢一點一定比較久
    seg = [0.0, 0.0, 0.0, math.pi / 2, 5.0, 0.5]
    times = [mt.seconds(seg, v) for v in (0.10, 0.15, 0.25, 0.35, 0.50, 1.0)]
    assert times == sorted(times, reverse=True)


def test_remaining():
    segs = [[0.0, 0.0, 0.0, 0.0, 5.0, 0.0], [0.0, 0.0, 0.0, math.pi, 5.0, 0.0]]
    total = mt.total_seconds(segs, 0.5)
    assert mt.remaining_seconds(segs, 0.5, None, 0.0) == pytest.approx(total)
    assert mt.remaining_seconds(segs, 0.5, 0, 0.0) == pytest.approx(total)
    assert mt.remaining_seconds(segs, 0.5, 1, 1e6) == 0.0
    assert mt.remaining_seconds(segs, 0.5, 0, 1e6) == pytest.approx(mt.seconds(segs[1], 0.5))
    assert mt.remaining_seconds(segs, 0.5, 2, 0.0) == 0.0
