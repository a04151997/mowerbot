#!/usr/bin/env python3
"""里程計數學的單元測試。

純 Python：不需要 ROS、不需要模擬器、不需要硬體，跑完不到一秒，結果是決定性的。
可以用 pytest 跑，也可以直接 python3 test_odometry.py。

測的是 mowerbot_bridge/odometry.py，也就是實車上唯一負責產生 /odom 的那段數學。
(f) 編碼器回繞與 (g) 讀取失敗是實車上一定會遇到、但模擬裡永遠不會出現的情況。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mowerbot_bridge.odometry import (              # noqa: E402
    DifferentialOdometry, body_to_wheel, normalize_angle)

# 車體實際物理量
WHEEL_RADIUS = 0.17
WHEEL_SEPARATION = 0.58
TICKS_PER_REV = 4096        # 測試用的假值（真值要查驅動板文件）

# 容許誤差：中點積分在這些理想輸入下應該非常準，
# 留 1 mm / 0.5 度的餘裕給浮點誤差就夠了。
TOL_M = 1e-3
TOL_RAD = math.radians(0.5)


def make_odom(**kwargs):
    params = dict(wheel_radius=WHEEL_RADIUS,
                  wheel_separation=WHEEL_SEPARATION,
                  ticks_per_rev=TICKS_PER_REV)
    params.update(kwargs)
    return DifferentialOdometry(**params)


def ticks_for_distance(metres, odom):
    """輪面走 metres 公尺對應幾個 tick"""
    return metres / (2.0 * math.pi * odom.effective_radius) * odom.ticks_per_rev


def drive(odom, left_m, right_m, steps=100, dt=0.01):
    """把左右輪各走 left_m / right_m 公尺分成 steps 拍餵進去。

    分多拍是必要的：一拍餵完等於假設車子瞬間轉完，
    中點積分在大角度下會失真，而實車是 30 Hz 連續取樣。
    """
    left_total = ticks_for_distance(left_m, odom)
    right_total = ticks_for_distance(right_m, odom)
    base_l, base_r = odom._prev if odom._prev else (0, 0)
    state = odom.update((base_l, base_r), dt)      # 建立基準
    for i in range(1, steps + 1):
        l = base_l + left_total * i / steps
        r = base_r + right_total * i / steps
        state = odom.update((int(round(l)), int(round(r))), dt)
    return state


# ----------------------------------------------------------------------
# (a) 直線前進 5 公尺
# ----------------------------------------------------------------------
def test_straight_5m():
    odom = make_odom()
    state = drive(odom, 5.0, 5.0)
    assert abs(state.x - 5.0) < TOL_M, 'x=%.6f' % state.x
    assert abs(state.y) < TOL_M, 'y=%.6f' % state.y
    assert abs(normalize_angle(state.theta)) < TOL_RAD, 'theta=%.6f' % state.theta


# ----------------------------------------------------------------------
# (b) 原地旋轉 360 度
# ----------------------------------------------------------------------
def test_rotate_360():
    odom = make_odom()
    # 原地轉一圈：兩輪等速反向，各走 pi * 有效輪距 / 2 ... 的兩倍即整圈
    arc = math.pi * odom.effective_separation       # 轉 360 度時單輪走的距離
    state = drive(odom, -arc, arc, steps=360)
    assert abs(state.x) < TOL_M, 'x=%.6f' % state.x
    assert abs(state.y) < TOL_M, 'y=%.6f' % state.y
    assert abs(state.theta - 2.0 * math.pi) < TOL_RAD, 'theta=%.6f' % state.theta
    # 取模之後應該回到 0
    assert abs(normalize_angle(state.theta)) < TOL_RAD


# ----------------------------------------------------------------------
# (c) 2m x 2m 正方形回到原點
# ----------------------------------------------------------------------
def test_square_2m():
    odom = make_odom()
    quarter = 0.5 * math.pi * odom.effective_separation * 0.5   # 轉 90 度單輪的距離
    for _ in range(4):
        drive(odom, 2.0, 2.0, steps=200)            # 直線 2 m
        drive(odom, -quarter, quarter, steps=90)    # 左轉 90 度
    state = odom.update(odom._prev, 0.01)
    err = math.hypot(state.x, state.y)
    assert err < 5e-3, '回到原點的誤差 %.6f m 太大' % err
    assert abs(normalize_angle(state.theta)) < TOL_RAD, 'theta=%.6f' % state.theta


# ----------------------------------------------------------------------
# (d) 後退、左轉、右轉的方向
# ----------------------------------------------------------------------
def test_backward():
    odom = make_odom()
    state = drive(odom, -2.0, -2.0)
    assert abs(state.x + 2.0) < TOL_M, 'x=%.6f' % state.x
    assert abs(state.y) < TOL_M
    assert abs(normalize_angle(state.theta)) < TOL_RAD


def test_turn_left_is_positive_theta():
    odom = make_odom()
    # 右輪比左輪快 -> 左轉 -> theta 增加（REP-103：逆時針為正）
    state = drive(odom, 1.0, 2.0)
    assert state.theta > 0.0, 'theta=%.6f 應該 > 0' % state.theta
    assert state.y > 0.0, 'y=%.6f 左轉之後應該往左偏' % state.y


def test_turn_right_is_negative_theta():
    odom = make_odom()
    state = drive(odom, 2.0, 1.0)
    assert state.theta < 0.0, 'theta=%.6f 應該 < 0' % state.theta
    assert state.y < 0.0, 'y=%.6f 右轉之後應該往右偏' % state.y


# ----------------------------------------------------------------------
# (e) 校正係數要正確地縮放結果
# ----------------------------------------------------------------------
def test_radius_correction_scales_distance():
    plain = make_odom()
    scaled = make_odom(radius_correction=1.10)
    # 餵「同一串 tick」給兩個里程計，走出來的距離應該差 1.10 倍
    n = int(ticks_for_distance(5.0, plain))
    for odom in (plain, scaled):
        odom.update((0, 0), 0.01)
        odom.update((n, n), 0.01)
    ratio = scaled.x / plain.x
    assert abs(ratio - 1.10) < 1e-6, '距離比例 %.6f 應該是 1.10' % ratio


def test_separation_correction_scales_rotation():
    plain = make_odom()
    scaled = make_odom(separation_correction=1.50)
    n = int(ticks_for_distance(1.0, plain))
    for odom in (plain, scaled):
        odom.update((0, 0), 0.01)
        odom.update((-n, n), 0.01)
    # 有效輪距變成 1.5 倍，同一串 tick 轉出來的角度只有 1/1.5
    ratio = scaled.theta / plain.theta
    assert abs(ratio - 1.0 / 1.5) < 1e-6, '角度比例 %.6f 應該是 %.6f' % (ratio, 1 / 1.5)


# ----------------------------------------------------------------------
# (f) 編碼器計數器回繞
# ----------------------------------------------------------------------
def test_encoder_wraparound():
    WRAP = 65536
    odom = make_odom(encoder_wrap=WRAP)
    # 從接近上限的地方開始，往前走一點點就回繞
    odom.update((WRAP - 10, WRAP - 10), 0.01)
    state = odom.update((5, 5), 0.01)               # 實際增量是 +15，不是 -65521
    expected = 15 * (2.0 * math.pi * odom.effective_radius) / odom.ticks_per_rev
    assert abs(state.x - expected) < 1e-9, \
        '回繞處理錯誤：x=%.6f，應該是 %.6f' % (state.x, expected)
    assert state.x < 0.01, '回繞被算成巨大跳躍：x=%.3f' % state.x


def test_encoder_wraparound_backwards():
    WRAP = 65536
    odom = make_odom(encoder_wrap=WRAP)
    odom.update((5, 5), 0.01)
    state = odom.update((WRAP - 10, WRAP - 10), 0.01)   # 實際增量是 -15
    expected = -15 * (2.0 * math.pi * odom.effective_radius) / odom.ticks_per_rev
    assert abs(state.x - expected) < 1e-9, \
        '往回回繞處理錯誤：x=%.6f，應該是 %.6f' % (state.x, expected)


def test_no_wrap_handling_when_disabled():
    # encoder_wrap = 0 時不應該自作聰明：驅動層保證回傳的是不會回繞的累計值
    odom = make_odom(encoder_wrap=0)
    odom.update((0, 0), 0.01)
    n = int(ticks_for_distance(1.0, odom))
    state = odom.update((n, n), 0.01)
    assert abs(state.x - 1.0) < TOL_M


# ----------------------------------------------------------------------
# (g) 編碼器讀取失敗
# ----------------------------------------------------------------------
def test_read_failure_keeps_pose():
    odom = make_odom()
    drive(odom, 1.0, 1.0)
    before = (odom.x, odom.y, odom.theta)
    state = odom.update(None, 0.01)
    assert state.ok is False
    assert (state.x, state.y, state.theta) == before, '讀取失敗時位姿不該變動'
    assert state.v == 0.0 and state.w == 0.0, '讀取失敗時速度要歸零，不要外插'
    assert odom.missed_reads == 1


def test_recover_after_read_failure():
    odom = make_odom()
    drive(odom, 1.0, 1.0)
    odom.update(None, 0.01)
    odom.update(None, 0.01)
    assert odom.missed_reads == 2
    # 恢復之後要能繼續累積，而且不能把「失聯期間」算成一次巨大位移
    n = int(ticks_for_distance(1.0, odom))
    base = odom._prev
    state = odom.update((base[0] + n, base[1] + n), 0.01)
    assert abs(state.x - 2.0) < TOL_M, 'x=%.6f 應該是 2.0' % state.x
    assert state.ok is True


# ----------------------------------------------------------------------
# 參數防呆
# ----------------------------------------------------------------------
def test_rejects_bad_ticks_per_rev():
    for bad in (0, -1, None):
        try:
            make_odom(ticks_per_rev=bad)
        except ValueError:
            continue
        raise AssertionError('ticks_per_rev=%r 應該被拒絕' % (bad,))


def test_first_reading_does_not_move():
    odom = make_odom()
    state = odom.update((12345, 67890), 0.01)
    assert (state.x, state.y, state.theta) == (0.0, 0.0, 0.0), \
        '第一次讀值只該建立基準，不該移動位姿'


# ----------------------------------------------------------------------
# 下行運動學（與上行共用同一組幾何量）
# ----------------------------------------------------------------------
def test_body_to_wheel_straight():
    left, right = body_to_wheel(1.0, 0.0, WHEEL_RADIUS, WHEEL_SEPARATION)
    assert abs(left - right) < 1e-12, '直線時兩輪角速度應該相同'
    assert abs(left - 1.0 / WHEEL_RADIUS) < 1e-12


def test_body_to_wheel_spin():
    left, right = body_to_wheel(0.0, 1.0, WHEEL_RADIUS, WHEEL_SEPARATION)
    assert right > 0 > left, '原地左轉時右輪正轉、左輪反轉'
    assert abs(left + right) < 1e-12, '原地旋轉時兩輪角速度大小相同'


# ----------------------------------------------------------------------
def main():
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print('  PASS  %s' % name)
        except AssertionError as exc:
            failed.append((name, exc))
            print('  FAIL  %s  ->  %s' % (name, exc))
    print('')
    print('%d / %d 通過' % (len(tests) - len(failed), len(tests)))
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
