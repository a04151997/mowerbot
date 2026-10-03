#!/usr/bin/env python3
"""輪趣（Wheeltec）C30D 下位機驅動（階段 36）。

協定見 drivers/README.md 的「輪趣 C30D」一節 —— 那份協定是從 wheeltec_robot_ros2
的鏡像讀來的，**未經實機確認**。這個驅動目前只對著 test/tools/fake_c30d.py 驗證過。

【暫時的接法：README 的方案 (a)，等現場確認之後再決定】
MotorDriver 介面是輪速層級，C30D 是車體速度層級、而且不回報編碼器 tick。
這裡用最直接的方式接上，bridge_node 的里程計路徑一行不動：
  下行  左右輪角速度 -> 用同一組有效輪半徑/輪距換回 (vx, wz) -> 送出
  上行  下位機回報的 (vx, wz) -> 換成左右輪角速度 -> 積分成「等效 tick」回報
換算是線性的、來回無損，但要知道兩件事：
  - 回報的 tick 是算出來的，不是編碼器原始值；ticks_per_rev 對這個驅動只是
    等效 tick 的解析度，沒有物理意義。
  - bridge_node 的 wheel_radius_correction / wheel_separation_correction 對這個
    驅動來說下行與上行會互相抵消（驅動用的就是乘過校正的有效值），等於不起作用。
    C30D 把 vx/wz 換成輪速用的幾何是寫在韌體裡的（README 的 E1 E2）。
"""

import math
import struct
import time

from .base import MotorDriver, DriverError

HEAD = 0x7B
TAIL = 0x7D
DOWN_LEN = 11
UP_LEN = 24


def _bcc(data):
    x = 0
    for b in data:
        x ^= b
    return x


def build_downlink(vx_mm_s, wz_mrad_s):
    """組 11 byte 下行封包：7B 00 00 vx vy wz BCC 7D，int16 大端，vy 固定 0"""
    try:
        body = struct.pack('>BBBhhh', HEAD, 0, 0, vx_mm_s, 0, wz_mrad_s)
    except struct.error as exc:
        raise DriverError('速度超出 int16 範圍：vx=%r mm/s, wz=%r mrad/s (%s)'
                          % (vx_mm_s, wz_mrad_s, exc))
    return body + bytes([_bcc(body), TAIL])


def parse_uplink(frame):
    """解析一個已經確認框頭、框尾、BCC 都正確的 24 byte 上行封包"""
    (_head, flag_stop, vx, vy, wz, _ax, _ay, _az, _gx, _gy, _gz,
     voltage_mv) = struct.unpack('>BBhhhhhhhhhH', bytes(frame[:22]))
    return {'flag_stop': flag_stop, 'vx': vx, 'vy': vy, 'wz': wz,
            'voltage_mv': voltage_mv}


class WheeltecDriver(MotorDriver):

    # 超過這段時間沒有收到任何一個有效上行封包，read_encoders() 就回傳 None。
    # 上行是 20 Hz（50 ms 一包），0.2 s = 連續掉 4 包。
    UPLINK_TIMEOUT = 0.2

    def __init__(self, ticks_per_rev, wheel_radius, wheel_separation,
                 serial_port, serial_baud=115200, clock=None):
        """wheel_radius / wheel_separation 要傳「乘過校正係數的有效值」，
        與 bridge_node 下行的 body_to_wheel、上行的 DifferentialOdometry 同一組，
        換算才會來回無損。clock 可以注入（單元測試用）。"""
        if ticks_per_rev is None or ticks_per_rev <= 0:
            raise ValueError('ticks_per_rev 必須是正數')
        if wheel_radius <= 0 or wheel_separation <= 0:
            raise ValueError('wheel_radius 與 wheel_separation 必須是正數')
        self.ticks_per_rev = float(ticks_per_rev)
        self.wheel_radius = float(wheel_radius)
        self.wheel_separation = float(wheel_separation)
        self.serial_port = str(serial_port)
        self.serial_baud = int(serial_baud)
        self._clock = clock or time.monotonic

        self._ser = None
        self._rxbuf = bytearray()
        self._left_ticks = 0.0       # 浮點累計，回報時才取整
        self._right_ticks = 0.0
        self._last_packet_time = None
        self._last_status = {}
        self.good_frames = 0
        self.bad_frames = 0

    # ---- 連線 --------------------------------------------------------
    def connect(self):
        try:
            import serial
        except ImportError as exc:
            raise DriverError('缺少 pyserial（sudo apt install python3-serial）: %s'
                              % exc)
        try:
            self._ser = serial.Serial(self.serial_port, self.serial_baud,
                                      timeout=0, write_timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            raise DriverError('打不開序列埠 %s @ %d: %s'
                              % (self.serial_port, self.serial_baud, exc))
        # 打開之前堆在緩衝區裡的上行封包是舊的，丟掉
        self._ser.reset_input_buffer()

    def disconnect(self):
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    # ---- 下行 --------------------------------------------------------
    def set_wheel_velocities(self, left_rad_s, right_rad_s):
        r = self.wheel_radius
        vx = r * (left_rad_s + right_rad_s) * 0.5
        wz = r * (right_rad_s - left_rad_s) / self.wheel_separation
        # round 而不是 int：0.5 / r * r 在浮點下可能是 0.49999...，截斷會變 499
        self._write(build_downlink(int(round(vx * 1000.0)),
                                   int(round(wz * 1000.0))))

    def stop(self):
        self._write(build_downlink(0, 0))

    def _write(self, pkt):
        if self._ser is None:
            raise DriverError('序列埠沒有打開')
        try:
            self._ser.write(pkt)
        except Exception as exc:
            raise DriverError('寫入序列埠失敗: %s' % exc)

    # ---- 上行 --------------------------------------------------------
    def read_encoders(self):
        if self._ser is None:
            return None
        try:
            data = self._ser.read(4096)
        except Exception:
            return None
        now = self._clock()
        if data:
            self._rxbuf.extend(data)
            for pkt in self._extract_frames():
                self._integrate(pkt, now)
        if (self._last_packet_time is None
                or now - self._last_packet_time > self.UPLINK_TIMEOUT):
            return None
        return (int(self._left_ticks), int(self._right_ticks))

    def read_status(self):
        return dict(self._last_status)

    # ---- 內部 --------------------------------------------------------
    def _extract_frames(self):
        """從緩衝區切出所有框頭、框尾、BCC 都正確的封包；錯的丟掉一個位元組重新找"""
        buf = self._rxbuf
        out = []
        while True:
            i = buf.find(bytes([HEAD]))
            if i < 0:
                del buf[:]
                break
            if i > 0:
                del buf[:i]
            if len(buf) < UP_LEN:
                break
            if buf[UP_LEN - 1] == TAIL and _bcc(buf[:22]) == buf[22]:
                out.append(parse_uplink(buf[:UP_LEN]))
                del buf[:UP_LEN]
                self.good_frames += 1
            else:
                del buf[:1]
                self.bad_frames += 1
        return out

    def _integrate(self, pkt, now):
        """把一個有效上行封包的車體速度換成輪速，積分成等效 tick。

        積分區間是「上一個有效封包到這一個」的時間。中間斷線超過
        UPLINK_TIMEOUT 的話不積分（不知道那段時間車子怎麼動，不外插），
        只把這一包當成新的起點。
        """
        prev = self._last_packet_time
        self._last_packet_time = now
        self._last_status = {'battery_voltage': pkt['voltage_mv'] / 1000.0,
                             'flag_stop': pkt['flag_stop']}
        if prev is None:
            return
        dt = now - prev
        if dt <= 0.0 or dt > self.UPLINK_TIMEOUT:
            return
        vx = pkt['vx'] / 1000.0
        wz = pkt['wz'] / 1000.0
        half_l = 0.5 * self.wheel_separation
        left = (vx - wz * half_l) / self.wheel_radius
        right = (vx + wz * half_l) / self.wheel_radius
        k = self.ticks_per_rev / (2.0 * math.pi)
        self._left_ticks += left * dt * k
        self._right_ticks += right * dt * k
