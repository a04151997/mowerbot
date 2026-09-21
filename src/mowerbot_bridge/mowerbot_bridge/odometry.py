#!/usr/bin/env python3
"""差速里程計的純數學層。

刻意不 import rclpy：這一層要能在沒有 ROS、沒有模擬器、沒有硬體的情況下
用單元測試驗證（見 mowerbot_bridge/test/test_odometry.py）。
bridge_node.py 只負責把 ROS 的訊息轉進轉出，數學全部在這裡。

座標系與符號約定（與 REP-103 一致）：
  x 向前、y 向左、theta 逆時針為正。
  v > 0 前進；w > 0 左轉（原地左轉時右輪比左輪快）。
"""

import math


class OdomState(object):
    """一次更新之後的里程計狀態。

    ok = False 代表這次沒有拿到有效的編碼器讀值，位姿維持上一次的值
    （不是歸零，也不是外插）。
    """

    __slots__ = ('x', 'y', 'theta', 'v', 'w', 'ok')

    def __init__(self, x, y, theta, v, w, ok):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.ok = ok

    def __repr__(self):
        return ('OdomState(x=%.4f, y=%.4f, theta=%.4f, v=%.3f, w=%.3f, ok=%s)'
                % (self.x, self.y, self.theta, self.v, self.w, self.ok))


def normalize_angle(a):
    """把角度收斂到 (-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


class DifferentialOdometry(object):
    """用左右輪的編碼器計數積分出位姿。

    參數的意義與預設值見 bridge_node.py 的 ROS 參數說明，這裡只收數值。

    encoder_wrap：編碼器計數器的模數（例如 16 位元就是 65536）。
    0 代表「驅動層回傳的是不會回繞的累計值」（loopback 驅動就是這樣）。
    真實驅動板多半會回繞，位元寬度必須查板子的文件才知道 —— 在拿到板子資訊
    之前這個值不要亂猜，維持 0，並在 drivers/README.md 的清單裡列為待查項目。
    """

    def __init__(self, wheel_radius, wheel_separation, ticks_per_rev,
                 radius_correction=1.0, separation_correction=1.0,
                 encoder_wrap=0, invert_left=False, invert_right=False):
        if ticks_per_rev is None or ticks_per_rev <= 0:
            raise ValueError('ticks_per_rev 必須是正數，收到 %r' % (ticks_per_rev,))
        if wheel_radius <= 0 or wheel_separation <= 0:
            raise ValueError('wheel_radius 與 wheel_separation 必須是正數')

        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.ticks_per_rev = float(ticks_per_rev)
        self.radius_correction = float(radius_correction)
        self.separation_correction = float(separation_correction)
        self.encoder_wrap = int(encoder_wrap)
        self.invert_left = bool(invert_left)
        self.invert_right = bool(invert_right)

        # 每個 tick 對應的輪面位移（公尺）。校正係數直接乘在半徑上。
        self._metres_per_tick = (2.0 * math.pi * self.effective_radius
                                 / self.ticks_per_rev)

        self.reset()

    # ---- 有效幾何量 --------------------------------------------------
    @property
    def effective_radius(self):
        return self.wheel_radius * self.radius_correction

    @property
    def effective_separation(self):
        return self.wheel_separation * self.separation_correction

    # ---- 狀態 --------------------------------------------------------
    def reset(self, x=0.0, y=0.0, theta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v = 0.0
        self.w = 0.0
        self._prev = None          # 上一次的 (left_ticks, right_ticks)
        self.missed_reads = 0      # 累計讀取失敗次數，給上層做健康度判斷

    # ---- 核心 --------------------------------------------------------
    def _delta(self, now, prev):
        """算出 tick 增量，必要時處理計數器回繞。

        回繞的判斷方式：如果增量的絕對值超過模數的一半，那一定是繞過去了
        （正常取樣週期內輪子不可能轉超過半圈的計數）。
        encoder_wrap = 0 時不做任何處理。
        """
        d = now - prev
        if self.encoder_wrap > 0:
            half = self.encoder_wrap // 2
            if d > half:
                d -= self.encoder_wrap
            elif d < -half:
                d += self.encoder_wrap
        return d

    def update(self, reading, dt):
        """餵一次編碼器讀值，回傳更新後的 OdomState。

        reading 是 (left_ticks, right_ticks)，或 None 代表這次讀取失敗。
        第一次呼叫只記錄基準值，不會移動位姿。
        """
        if reading is None:
            # 讀不到就維持上一個位姿，速度歸零（不要外插，那會讓 SLAM 收到假資料）
            self.missed_reads += 1
            self.v = 0.0
            self.w = 0.0
            return OdomState(self.x, self.y, self.theta, 0.0, 0.0, False)

        left, right = int(reading[0]), int(reading[1])
        if self._prev is None:
            self._prev = (left, right)
            return OdomState(self.x, self.y, self.theta, 0.0, 0.0, True)

        d_left_ticks = self._delta(left, self._prev[0])
        d_right_ticks = self._delta(right, self._prev[1])
        self._prev = (left, right)

        if self.invert_left:
            d_left_ticks = -d_left_ticks
        if self.invert_right:
            d_right_ticks = -d_right_ticks

        d_left = d_left_ticks * self._metres_per_tick
        d_right = d_right_ticks * self._metres_per_tick

        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.effective_separation

        # 中點積分：用這一段的中間朝向去投影位移，直線與圓弧都夠準。
        mid = self.theta + 0.5 * d_theta
        self.x += d_center * math.cos(mid)
        self.y += d_center * math.sin(mid)
        self.theta += d_theta

        if dt and dt > 0.0:
            self.v = d_center / dt
            self.w = d_theta / dt
        else:
            self.v = 0.0
            self.w = 0.0

        return OdomState(self.x, self.y, self.theta, self.v, self.w, True)


def body_to_wheel(v, w, wheel_radius, wheel_separation,
                  radius_correction=1.0, separation_correction=1.0):
    """差速運動學（下行）：車體速度 -> 左右輪角速度 rad/s。

        v_left  = (v - w * L / 2) / r
        v_right = (v + w * L / 2) / r

    校正係數與里程計用同一組：下行與上行用不同的幾何量的話，
    車子走出來的軌跡與里程計算出來的軌跡會系統性地對不起來。
    """
    r = wheel_radius * radius_correction
    half_l = 0.5 * wheel_separation * separation_correction
    return ((v - w * half_l) / r, (v + w * half_l) / r)
