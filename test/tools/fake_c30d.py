#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假的輪趣 C30D 下位機（階段 36，Phase R 用）。

建立一個 pty，對 slave 端（驅動程式打開的那一端）表現得像 C30D：
  - 持續以 20 Hz 送出上行封包（24 bytes：速度、IMU、電壓、XOR）
  - 記錄收到的所有下行封包（11 bytes），可存成 CSV 供驗證
協定照 src/mowerbot_bridge/mowerbot_bridge/drivers/README.md 的「輪趣 C30D」一節，
那份協定本身**未經實機確認**；這個假裝置證明的是「我們的軟體照那份協定講話」，
不是「C30D 真的這樣回」。

模式（可以在執行中用 set_mode() 切換，模擬線鬆了、干擾、板子重開）：
  normal   正常回應
  silent   不回任何封包（測連線中斷處理）
  corrupt  封包內容正常但 XOR 錯誤（測校驗）
  slow     延遲回應：每 slow_period 秒才送一包（測逾時）
  garbage  隨機位元組（測不會誤解析）

上行回報的速度：
  預設「回聲」—— 回報最後一個有效下行封包的 vx / wz（等於一台完美的車）。
  set_reported_velocity(vx_mm_s, wz_mrad_s) 可以改成回報固定值（R8 用），
  傳 None 恢復回聲。

單獨執行（手動測試用）：
    python3 test/tools/fake_c30d.py --mode normal --csv /tmp/downlink.csv
會印出 pty 的路徑，Ctrl-C 結束時寫出 CSV。
"""

import argparse
import csv
import os
import random
import select
import struct
import threading
import time
import tty

MODES = ('normal', 'silent', 'corrupt', 'slow', 'garbage')

HEAD = 0x7B
TAIL = 0x7D
DOWN_LEN = 11
UP_LEN = 24

# 上行 IMU / 電壓的換算（與 README 相同），假裝置送靜止水平的讀值
ACCEL_LSB_PER_MS2 = 1671.84      # ±2 g 量程
GRAVITY = 9.80665


def bcc(data):
    """逐位元組 XOR"""
    x = 0
    for b in data:
        x ^= b
    return x


def build_uplink(vx_mm_s, vy_mm_s, wz_mrad_s, voltage_mv, flag_stop=0,
                 corrupt=False):
    """組一個 24 byte 的上行封包。corrupt=True 時 BCC 故意算錯（其餘內容正常）"""
    az = int(round(GRAVITY * ACCEL_LSB_PER_MS2))
    body = struct.pack('>BBhhhhhhhhhH', HEAD, flag_stop,
                       int(vx_mm_s), int(vy_mm_s), int(wz_mrad_s),
                       0, 0, az,          # 加速度 x/y/z
                       0, 0, 0,           # 角速度 x/y/z
                       int(voltage_mv))
    check = bcc(body)
    if corrupt:
        check ^= 0xFF
    return body + bytes([check, TAIL])


def parse_downlink(frame):
    """解析 11 byte 下行封包，回傳 dict；框頭/框尾/BCC 的檢查結果放在欄位裡"""
    head, flag, reserved, vx, vy, wz, check, tail = struct.unpack(
        '>BBBhhhBB', bytes(frame))
    return {
        'head_ok': head == HEAD,
        'tail_ok': tail == TAIL,
        'bcc_ok': bcc(frame[:9]) == check,
        'flag': flag,
        'reserved': reserved,
        'vx': vx,
        'vy': vy,
        'wz': wz,
    }


class FakeC30D(object):

    def __init__(self, mode='normal', rate=20.0, voltage_mv=24000,
                 slow_period=1.0, seed=36):
        if mode not in MODES:
            raise ValueError('未知的模式 %r，可用：%s' % (mode, ', '.join(MODES)))
        self.mode = mode
        self.rate = float(rate)
        self.voltage_mv = int(voltage_mv)
        self.slow_period = float(slow_period)
        self._rng = random.Random(seed)       # 固定種子：garbage 的內容可重現

        self._lock = threading.Lock()
        self._override = None                  # (vx_mm_s, wz_mrad_s) 或 None = 回聲
        self._echo = (0, 0)

        # 下行紀錄
        self.downlink = []        # dict：t、raw(hex)、len_ok 與 parse_downlink 的欄位
        self.rx_bytes = 0         # 從 slave 端收到的原始位元組總數（read_only 測試用）
        self._rxbuf = bytearray()

        # 上行紀錄（R8 用來算「實際送了多久的某個速度」）
        self.uplink = []          # (t, mode, vx_mm_s, wz_mrad_s)

        self.master_fd, self.slave_fd = os.openpty()
        # slave 端一定要設成 raw：pty 預設開著 ECHO，驅動打開之前寫進 master 的
        # 上行封包會被回聲回 master，假裝置就會把自己的上行當成「收到的下行」。
        tty.setraw(self.slave_fd)
        self.port = os.ttyname(self.slave_fd)
        # 驅動還沒打開 slave 之前沒有人讀，pty 緩衝區滿了寫入就會卡住整個執行緒。
        # 改成非阻塞寫、滿了就丟（真的下位機也不會等上位機）。
        os.set_blocking(self.master_fd, False)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='fake_c30d',
                                        daemon=True)

    # ---- 控制 --------------------------------------------------------
    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        for fd in (self.master_fd, self.slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def set_mode(self, mode):
        if mode not in MODES:
            raise ValueError('未知的模式 %r' % mode)
        with self._lock:
            self.mode = mode

    def set_reported_velocity(self, vx_mm_s, wz_mrad_s=0):
        """固定回報的速度；vx_mm_s=None 恢復回聲模式"""
        with self._lock:
            self._override = None if vx_mm_s is None else (int(vx_mm_s),
                                                           int(wz_mrad_s))

    # ---- 查詢 --------------------------------------------------------
    def downlink_since(self, t0, t1=None):
        with self._lock:
            return [d for d in self.downlink
                    if d['t'] >= t0 and (t1 is None or d['t'] <= t1)]

    def uplink_since(self, t0, t1=None):
        with self._lock:
            return [u for u in self.uplink
                    if u[0] >= t0 and (t1 is None or u[0] <= t1)]

    def write_csv(self, path):
        fields = ['t', 'raw', 'len_ok', 'head_ok', 'tail_ok', 'bcc_ok',
                  'flag', 'reserved', 'vx', 'vy', 'wz']
        with self._lock:
            rows = list(self.downlink)
        with open(path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, '') for k in fields})
        return len(rows)

    # ---- 內部 --------------------------------------------------------
    def _run(self):
        period = 1.0 / self.rate
        next_send = time.time()
        last_slow = 0.0
        while not self._stop.is_set():
            timeout = max(0.0, next_send - time.time())
            try:
                r, _w, _x = select.select([self.master_fd], [], [], timeout)
            except (OSError, ValueError):
                return
            if r:
                self._read_downlink()
            now = time.time()
            if now < next_send:
                continue
            next_send += period
            if next_send < now:              # 落後太多（例如被排程延遲）就重新對齊
                next_send = now + period
            with self._lock:
                mode = self.mode
                vx, wz = self._override if self._override else self._echo
            if mode == 'silent':
                continue
            if mode == 'slow':
                if now - last_slow < self.slow_period:
                    continue
                last_slow = now
            if mode == 'garbage':
                pkt = bytes(self._rng.getrandbits(8) for _ in range(UP_LEN))
            else:
                pkt = build_uplink(vx, 0, wz, self.voltage_mv,
                                   corrupt=(mode == 'corrupt'))
            try:
                os.write(self.master_fd, pkt)
            except BlockingIOError:
                continue                     # 沒有人在讀，緩衝區滿了：丟掉這一包
            except OSError:
                return
            with self._lock:
                self.uplink.append((now, mode, vx, wz))

    def _read_downlink(self):
        try:
            data = os.read(self.master_fd, 4096)
        except (BlockingIOError, OSError):
            return
        if not data:
            return
        now = time.time()
        with self._lock:
            self.rx_bytes += len(data)
            self._rxbuf.extend(data)
            buf = self._rxbuf
            while True:
                i = buf.find(bytes([HEAD]))
                if i < 0:
                    # 沒有框頭：全部是雜訊，也要記下來（len_ok=False），不能安靜丟掉
                    if buf:
                        self.downlink.append({'t': now, 'raw': buf.hex(),
                                              'len_ok': False})
                    del buf[:]
                    break
                if i > 0:
                    self.downlink.append({'t': now, 'raw': buf[:i].hex(),
                                          'len_ok': False})
                    del buf[:i]
                if len(buf) < DOWN_LEN:
                    break
                frame = bytes(buf[:DOWN_LEN])
                rec = parse_downlink(frame)
                rec.update({'t': now, 'raw': frame.hex(), 'len_ok': True})
                if rec['tail_ok'] and rec['bcc_ok']:
                    del buf[:DOWN_LEN]
                    self._echo = (rec['vx'], rec['wz'])
                else:
                    # 框不對：記下來，丟掉這個框頭位元組重新找（不整包丟，免得吃掉下一包）
                    del buf[:1]
                self.downlink.append(rec)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--mode', default='normal', choices=MODES)
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--slow-period', type=float, default=1.0)
    ap.add_argument('--csv', default='', help='結束時把下行封包寫到這個 CSV')
    ap.add_argument('--vx', type=int, default=None,
                    help='固定回報的 vx (mm/s)；不給 = 回聲下行指令')
    ap.add_argument('--wz', type=int, default=0, help='固定回報的 wz (mrad/s)')
    a = ap.parse_args()

    fake = FakeC30D(mode=a.mode, rate=a.rate, slow_period=a.slow_period)
    if a.vx is not None:
        fake.set_reported_velocity(a.vx, a.wz)
    fake.start()
    print('假 C30D 已啟動：pty = %s，模式 = %s（Ctrl-C 結束）' % (fake.port, a.mode),
          flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        fake.stop()
        if a.csv:
            n = fake.write_csv(a.csv)
            print('下行封包 %d 筆 -> %s' % (n, a.csv))
        print('收到原始位元組 %d' % fake.rx_bytes)


if __name__ == '__main__':
    main()
