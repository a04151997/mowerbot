#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""里程計校正工具（互動式）。

用途：量出 bridge_node 的兩個校正係數，以及驗證累積誤差。

    (a) 直線 5.0 m        -> wheel_radius_correction
    (b) 原地旋轉 360 度 x5 -> wheel_separation_correction
    (c) 2m x 2m 正方形     -> 驗證累積誤差（不產生係數，只回報）

流程：本腳本發指令讓車子動 -> 記錄 /odom 的起訖 -> 問你「實際量到多少」
      -> 算出校正係數 -> 印出應該設定的參數值與 ros2 param 指令。

用法:
    python3 test/tools/calibrate_odometry.py              # 完整互動式校正
    python3 test/tools/calibrate_odometry.py --auto       # 乾跑（見下）
    python3 test/tools/calibrate_odometry.py --only=b     # 只做其中一項
    python3 test/tools/calibrate_odometry.py --speed=0.15 # 放慢

--auto 乾跑模式：不問問題，直接把 /odom 自己報的值當成「實際量到的值」。
    這在 Gazebo 裡會算出係數 1.0 —— 那不代表車子準，只代表
    「開車、記錄、計算、輸出」這條流程本身沒有寫錯。
    真正的校正一定要用捲尺與地面標記，在實車上做。

【安全】實車第一次跑這支腳本之前，先確認 docs/hardware_bringup.md
        第 5 步的急停 / deadman / watchdog 都測過了。
        本腳本會讓 74 kg 的車子前進 5 公尺並原地旋轉 5 圈，
        周圍至少要留 2 公尺淨空，手要放在急停上。
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


class Calibrator(Node):

    def __init__(self, args):
        super().__init__('calibrate_odometry')
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.unwrapped_yaw = None      # 連續角度（不收斂到 -pi..pi，才數得出圈數）
        self._last_yaw = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)

    def _on_odom(self, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        if self._last_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            d = yaw - self._last_yaw
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            self.unwrapped_yaw += d
        self._last_yaw = yaw
        self.odom = msg

    # ---- 基本動作 ----------------------------------------------------
    def wait_for_odom(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout and self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def pose(self):
        p = self.odom.pose.pose.position
        return (p.x, p.y, self.unwrapped_yaw)

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self, settle=1.5):
        t0 = time.time()
        while time.time() - t0 < settle:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def turn_by(self, target, tol=math.radians(2.0), tries=4):
        """轉 target 弧度，停下來之後量誤差，不足就補轉（最多 tries 次）。

        為什麼需要：開環「轉到 /odom 說夠了就停」一定會過衝 ——
        指令停止到車子真的停下來之間還會再轉一段。實測在 0.5 rad/s 下
        每次過衝約 11 度，走完正方形四個角就累積 45 度，
        回原點誤差 1.48 m，那量到的是這支腳本的過衝，不是里程計的誤差。
        補轉之後 (c) 才真的在量「里程計的累積誤差」。

        注意這只是讓實驗本身成立的閉環，不是車子的控制器 ——
        實車的循跡精度由 Nav2 的 DWB 決定，與這裡無關。
        """
        sign = 1.0 if target >= 0 else -1.0
        start = self.pose()[2]
        for attempt in range(tries):
            remain = target - (self.pose()[2] - start)
            if abs(remain) <= tol:
                break
            w = self.args.turn_speed * (1.0 if remain > 0 else -1.0)
            # 補轉時放慢，否則又會過衝
            if attempt > 0:
                w *= 0.35
            edge = self.pose()[2]
            self.drive_until(
                0.0, w,
                lambda p, e=edge, r=remain: abs(p[2] - e) >= abs(r),
                limit=abs(remain) / max(abs(w), 1e-3) * 3.0 + 6.0)
        _ = sign
        return self.pose()[2] - start

    def drive_until(self, v, w, done, limit):
        """持續發指令直到 done(pose) 成立或超過 limit 秒"""
        t0 = time.time()
        while time.time() - t0 < limit:
            self.publish(v, w)
            rclpy.spin_once(self, timeout_sec=0.05)
            if done(self.pose()):
                break
        self.stop()
        return time.time() - t0

    # ---- 三項實驗 ----------------------------------------------------
    def run_straight(self):
        target = self.args.distance
        print('')
        print('=' * 70)
        print('(a) 直線 %.2f m  ->  wheel_radius_correction' % target)
        print('=' * 70)
        if not self.confirm('車子會以 %.2f m/s 前進約 %.1f 公尺'
                            % (self.args.speed, target)):
            return None
        start = self.pose()
        elapsed = self.drive_until(
            self.args.speed, 0.0,
            lambda p: math.hypot(p[0] - start[0], p[1] - start[1]) >= target,
            limit=target / max(self.args.speed, 1e-3) * 3.0 + 10.0)
        end = self.pose()
        reported = math.hypot(end[0] - start[0], end[1] - start[1])
        drift = abs(end[2] - start[2])
        print('  /odom 報告走了     : %.4f m  (耗時 %.1f s)' % (reported, elapsed))
        print('  過程中的朝向變化   : %.4f rad (%.2f 度)'
              % (drift, math.degrees(drift)))
        if drift > math.radians(5):
            print('  ⚠️ 直線行駛卻轉了超過 5 度，兩側輪速可能不對稱，'
                  '先處理這個再談校正')
        actual = self.ask_float('用捲尺量到的實際位移 (公尺)', reported)
        if actual is None or reported <= 0:
            return None
        factor = actual / reported
        print('  -> wheel_radius_correction = 目前值 x %.4f' % factor)
        return ('wheel_radius_correction', factor, reported, actual)

    def run_rotation(self):
        turns = self.args.turns
        target = 2.0 * math.pi * turns
        print('')
        print('=' * 70)
        print('(b) 原地旋轉 360 度 x %d  ->  wheel_separation_correction' % turns)
        print('=' * 70)
        print('  做法：在車體與地面各做一個對齊的記號，轉完之後看記號差多少。')
        if not self.confirm('車子會以 %.2f rad/s 原地旋轉 %d 圈'
                            % (self.args.turn_speed, turns)):
            return None
        start = self.pose()
        elapsed = self.drive_until(
            0.0, self.args.turn_speed,
            lambda p: abs(p[2] - start[2]) >= target,
            limit=target / max(self.args.turn_speed, 1e-3) * 3.0 + 15.0)
        end = self.pose()
        reported = abs(end[2] - start[2])
        pos_drift = math.hypot(end[0] - start[0], end[1] - start[1])
        print('  /odom 報告轉了     : %.4f rad = %.2f 度 = %.3f 圈 (耗時 %.1f s)'
              % (reported, math.degrees(reported), reported / (2 * math.pi), elapsed))
        print('  原地旋轉的位置漂移 : %.4f m' % pos_drift)
        actual_deg = self.ask_float(
            '實際轉了幾度 (%d 圈是 %d 度；轉不足就填實際值)'
            % (turns, int(360 * turns)), math.degrees(reported))
        if actual_deg is None or actual_deg <= 0:
            return None
        actual = math.radians(actual_deg)
        # theta_reported = delta / (L * c)  =>  c_true = c_old * theta_rep / theta_act
        factor = reported / actual
        print('  -> wheel_separation_correction = 目前值 x %.4f' % factor)
        if factor > 1.05:
            print('     (實際轉得比報告的少，代表有效輪距比幾何值大 —— '
                  'skid-steer 的正常現象)')
        return ('wheel_separation_correction', factor, reported, actual)

    def run_square(self):
        side = self.args.square_side
        print('')
        print('=' * 70)
        print('(c) %.1fm x %.1fm 正方形  ->  驗證累積誤差' % (side, side))
        print('=' * 70)
        if not self.confirm('車子會走一個 %.1f 公尺見方的正方形' % side):
            return None
        origin = self.pose()
        for i in range(4):
            leg_start = self.pose()
            self.drive_until(
                self.args.speed, 0.0,
                lambda p, s=leg_start: math.hypot(p[0] - s[0], p[1] - s[1]) >= side,
                limit=side / max(self.args.speed, 1e-3) * 3.0 + 10.0)
            turned = self.turn_by(math.pi / 2.0)
            p = self.pose()
            print('       轉彎實際轉了 %.2f 度 (目標 90 度，已補轉修正)'
                  % math.degrees(turned))
            print('  第 %d 邊完成，目前 /odom 位置 (%.3f, %.3f) 朝向 %.1f 度'
                  % (i + 1, p[0], p[1], math.degrees(p[2])))
        end = self.pose()
        err = math.hypot(end[0] - origin[0], end[1] - origin[1])
        heading_err = abs(end[2] - origin[2] - 2.0 * math.pi)
        print('  /odom 報告的回原點誤差 : %.4f m  (佔總行程 %.1f%%)'
              % (err, 100.0 * err / (4 * side)))
        print('  /odom 報告的朝向誤差   : %.4f rad (%.2f 度)'
              % (heading_err, math.degrees(heading_err)))
        actual = self.ask_float('車子實際離出發點多遠 (公尺)', err)
        return ('square_error', err, err, actual)

    # ---- 互動 --------------------------------------------------------
    def confirm(self, what):
        if self.args.auto:
            print('  [--auto] %s' % what)
            return True
        ans = input('  %s。周圍淨空了嗎？手放在急停上了嗎？ [y/N] ' % what)
        return ans.strip().lower() in ('y', 'yes')

    def ask_float(self, prompt, auto_value):
        if self.args.auto:
            print('  [--auto] %s = %.4f (直接採用 /odom 的值)' % (prompt, auto_value))
            return auto_value
        while True:
            raw = input('  %s: ' % prompt).strip()
            if raw == '':
                print('    (略過這一項)')
                return None
            try:
                return float(raw)
            except ValueError:
                print('    請輸入數字，或直接按 Enter 略過')


def main():
    ap = argparse.ArgumentParser(description='里程計校正工具')
    ap.add_argument('--auto', action='store_true',
                    help='乾跑：不問問題，把 /odom 的值當成實際值（驗證腳本邏輯用）')
    ap.add_argument('--only', default='abc', help='只做其中幾項，例如 --only=b')
    ap.add_argument('--speed', type=float, default=0.2, help='直線速度 m/s')
    ap.add_argument('--turn-speed', type=float, default=0.5, help='旋轉速度 rad/s')
    ap.add_argument('--distance', type=float, default=5.0, help='(a) 的距離 m')
    ap.add_argument('--turns', type=int, default=5, help='(b) 的圈數')
    ap.add_argument('--square-side', type=float, default=2.0, help='(c) 的邊長 m')
    args = ap.parse_args()

    rclpy.init()
    node = Calibrator(args)
    print('mowerbot 里程計校正')
    print('  模式: %s' % ('乾跑 (--auto)' if args.auto else '互動式'))
    print('  等待 /odom ...')
    if not node.wait_for_odom():
        print('  20 秒內沒有收到 /odom。確認 bridge_node（實車）或 Gazebo（模擬）在跑。')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    results = []
    try:
        if 'a' in args.only:
            r = node.run_straight()
            if r:
                results.append(r)
        if 'b' in args.only:
            r = node.run_rotation()
            if r:
                results.append(r)
        if 'c' in args.only:
            r = node.run_square()
            if r:
                results.append(r)
    except KeyboardInterrupt:
        print('\n  中斷，送零速度')
    finally:
        node.stop()

    print('')
    print('=' * 70)
    print('校正結果')
    print('=' * 70)
    if not results:
        print('  沒有完成任何一項。')
    for name, factor, reported, actual in results:
        if name == 'square_error':
            print('  正方形回原點誤差 : /odom 報告 %.4f m，實際量到 %s'
                  % (reported, ('%.4f m' % actual) if actual is not None else '(未填)'))
            continue
        print('  %-28s 建議乘上 %.4f   (/odom 報告 %.4f，實際 %.4f)'
              % (name, factor, reported, actual))
    corrections = [(n, f) for n, f, _r, _a in results if n != 'square_error']
    if corrections:
        print('')
        print('  套用方式（擇一）：')
        print('  1) 立即套用到執行中的節點（重開就沒了，適合反覆試）:')
        for name, factor in corrections:
            print('       ros2 param set /mower_bridge %s %.4f' % (name, factor))
        print('  2) 寫進 bringup_real.launch.py 的 bridge_node 參數（永久）:')
        for name, factor in corrections:
            print("       '%s': %.4f," % (name, factor))
        print('')
        print('  注意：上面的數字是「乘上目前值」。如果節點目前的參數不是 1.0，')
        print('  要自己乘進去（新值 = 目前值 x 建議係數）。')
    print('')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
