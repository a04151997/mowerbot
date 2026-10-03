# -*- coding: utf-8 -*-
"""對正階段實驗的量測端 (階段 38 決定 1-A)。只送對正 goal，不跑割草線。

用法：python3 align_probe.py <次數> <轉角 deg> <輸出 jsonl>
每一次：讀目前朝向 θ0，目標朝向 = θ0 + 轉角，送 FollowPath
  path = 一個位姿 (目前 x, y, 目標朝向)，controller_id=AlignController，goal_checker_id=align_goal_checker
這個實驗沒有 manager 幫忙轉發，所以這裡自己把 /cmd_vel_nav 轉到 /cmd_vel (測試夾具)。
記錄：收斂時間 (送出 -> 結果)、結果狀態、結果當下與 1 s 後的朝向誤差、
      朝向誤差過零次數 (衝過頭)、最後 1 s 的 wz 指令變號次數 (震盪)、整段平移位移。
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus

N = int(sys.argv[1]); TURN = math.radians(float(sys.argv[2])); OUT = sys.argv[3]
rclpy.init(); n = Node('align_probe')
st = {'odom': None, 'cmds': []}
pub = n.create_publisher(Twist, '/cmd_vel', 10)


def on_odom(m):
    q = m.pose.pose.orientation
    st['odom'] = (time.time(), m.pose.pose.position.x, m.pose.pose.position.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    if 'trace' in st:
        st['trace'].append(st['odom'])


def on_cmd(m):
    pub.publish(m)
    st['cmds'].append((time.time(), m.linear.x, m.angular.z))


n.create_subscription(Odometry, '/odom', on_odom, 50)
n.create_subscription(Twist, '/cmd_vel_nav', on_cmd, 50)
ac = ActionClient(n, FollowPath, 'follow_path')
if not ac.wait_for_server(timeout_sec=60):
    print('follow_path 沒上線'); sys.exit(1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def spin(sec):
    t = time.time()
    while time.time() - t < sec:
        rclpy.spin_once(n, timeout_sec=0.02)


spin(2.0)
with open(OUT, 'a') as fh:
    for k in range(N):
        spin(1.0)
        t0, x0, y0, th0 = st['odom']
        target = wrap(th0 + TURN)
        p = PoseStamped(); p.header.frame_id = 'odom'
        p.pose.position.x, p.pose.position.y = x0, y0
        p.pose.orientation.z, p.pose.orientation.w = math.sin(target / 2), math.cos(target / 2)
        path = Path(); path.header.frame_id = 'odom'; path.poses = [p]
        g = FollowPath.Goal(); g.path = path
        g.controller_id = 'AlignController'; g.goal_checker_id = 'align_goal_checker'
        st['trace'] = []; st['cmds'] = []
        ts = time.time()
        fut = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
        gh = fut.result()
        if gh is None or not gh.accepted:
            rec = {'k': k, 'status': 'REJECTED'}
        else:
            rf = gh.get_result_async()
            rclpy.spin_until_future_complete(n, rf, timeout_sec=30)
            td = time.time()
            status = rf.result().status if rf.result() else -1
            err_done = wrap(st['odom'][3] - target)
            spin(1.0)
            err_1s = wrap(st['odom'][3] - target)
            after = [c for c in st['cmds'] if c[0] > td]
            o_after = [o for o in st['trace'] if o[0] > td]
            rate_after = (abs(wrap(o_after[-1][3] - o_after[0][3])) / max(o_after[-1][0] - o_after[0][0], 1e-6)
                          if len(o_after) > 1 else None)
            tr = list(st['trace'])
            errs = [wrap(o[3] - target) for o in tr]
            cross = sum(1 for a, b in zip(errs, errs[1:]) if a * b < 0 and abs(a) < 0.5)
            late = [c[2] for c in st['cmds'] if td - 1.0 <= c[0] <= td and abs(c[2]) > 1e-3]
            flips = sum(1 for a, b in zip(late, late[1:]) if a * b < 0)
            move = max(math.hypot(o[1] - x0, o[2] - y0) for o in tr) if tr else float('nan')
            # 卡住診斷：最後 2 s 的 wz 指令與實際角速度、第一次進入 0.1 rad 的時間
            c2 = [c for c in st['cmds'] if td - 2.0 <= c[0] <= td]
            wz_cmd_last2 = (sum(abs(c[2]) for c in c2) / len(c2)) if c2 else 0.0
            vx_cmd_last2 = (sum(abs(c[1]) for c in c2) / len(c2)) if c2 else 0.0
            o2 = [o for o in tr if td - 2.0 <= o[0] <= td]
            rate_last2 = (abs(wrap(o2[-1][3] - o2[0][3])) / max(o2[-1][0] - o2[0][0], 1e-6)) if len(o2) > 1 else 0.0
            t01 = next((o[0] - ts for o, e in zip(tr, errs) if abs(e) < 0.1), None)
            # 震盪：第一次過零之後，誤差的最大絕對值 (幅度) 與過零頻率
            i0 = next((i for i in range(1, len(errs)) if errs[i - 1] * errs[i] < 0 and abs(errs[i]) < 0.5), None)
            osc_amp = max(abs(e) for e in errs[i0:]) if i0 is not None else 0.0
            osc_dur = (tr[-1][0] - tr[i0][0]) if i0 is not None else 0.0
            osc_freq = (cross / 2.0) / osc_dur if osc_dur > 0.2 else 0.0
            rec = {'k': k, 'status': {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                                       GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(status, str(status)),
                   'time': round(td - ts, 3), 'turn_deg': round(math.degrees(TURN), 1),
                   'err_done_rad': round(err_done, 4), 'err_1s_rad': round(err_1s, 4),
                   'overshoot_crossings': cross, 'wz_sign_flips_last1s': flips,
                   'max_translation_m': round(move, 4),
                   'mean_abs_wz_cmd_last2s': round(wz_cmd_last2, 4),
                   'mean_abs_vx_cmd_last2s': round(vx_cmd_last2, 4),
                   'actual_yaw_rate_last2s': round(rate_last2, 4),
                   'time_to_err_0.1rad': round(t01, 2) if t01 is not None else None,
                   'osc_amp_after_first_cross_rad': round(osc_amp, 4),
                   'osc_freq_hz': round(osc_freq, 3),
                   # 結果之後 1 s：controller 有沒有送零、車子實際還在轉多快
                   'cmds_after_done': len(after),
                   'last_cmd_after_done': [round(after[-1][1], 3), round(after[-1][2], 3)] if after else None,
                   'yaw_rate_1s_after_done': round(rate_after, 4) if rate_after is not None else None}
        print(json.dumps(rec, ensure_ascii=False), flush=True)
        fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
        del st['trace']
rclpy.shutdown()
