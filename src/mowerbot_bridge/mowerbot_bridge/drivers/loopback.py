#!/usr/bin/env python3
"""假驅動：沒有硬體時用來測試整條 bridge_node 的邏輯。

行為：把收到的速度指令按時間積分成「完美的」編碼器計數。
沒有打滑、沒有延遲、沒有雜訊、不會回繞 —— 它證明的是
「bridge_node 的資料流與數學是通的」，不是「車子會這樣動」。

實車的里程計一定比這個差，差多少要靠 test/tools/calibrate_odometry.py 實測。
"""

import math
import time

from .base import MotorDriver


class LoopbackDriver(MotorDriver):

    def __init__(self, ticks_per_rev, clock=None, **_unused):
        """clock 可以注入（單元測試用），預設是單調時鐘。

        bridge_node 對所有驅動傳同一組參數（輪半徑、輪距、序列埠...），
        假驅動用不到的就忽略。"""
        if ticks_per_rev is None or ticks_per_rev <= 0:
            raise ValueError('ticks_per_rev 必須是正數')
        self.ticks_per_rev = float(ticks_per_rev)
        self._clock = clock or time.monotonic
        self._connected = False
        self._left_cmd = 0.0        # rad/s
        self._right_cmd = 0.0
        self._left_ticks = 0.0      # 用浮點累計，回報時才取整，避免低速被截斷成 0
        self._right_ticks = 0.0
        self._last = None

    # ---- 連線 --------------------------------------------------------
    def connect(self):
        self._connected = True
        self._last = self._clock()

    def disconnect(self):
        self._connected = False

    # ---- 下行 --------------------------------------------------------
    def set_wheel_velocities(self, left_rad_s, right_rad_s):
        self._integrate()
        self._left_cmd = float(left_rad_s)
        self._right_cmd = float(right_rad_s)

    def stop(self):
        self._integrate()
        self._left_cmd = 0.0
        self._right_cmd = 0.0

    # ---- 上行 --------------------------------------------------------
    def read_encoders(self):
        if not self._connected:
            return None
        self._integrate()
        return (int(self._left_ticks), int(self._right_ticks))

    def read_status(self):
        # 假驅動沒有真實感測值，回傳固定的合理值方便上層測試欄位有沒有接對。
        return {
            'battery_voltage': 24.0,
            'battery_current': 0.0,
            'is_overheated': False,
        }

    # ---- 內部 --------------------------------------------------------
    def _integrate(self):
        """把上次到現在的指令速度積分進編碼器計數"""
        now = self._clock()
        if self._last is None:
            self._last = now
            return
        dt = now - self._last
        self._last = now
        if dt <= 0.0:
            return
        k = self.ticks_per_rev / (2.0 * math.pi)
        self._left_ticks += self._left_cmd * dt * k
        self._right_ticks += self._right_cmd * dt * k
