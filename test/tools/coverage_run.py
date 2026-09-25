# -*- coding: utf-8 -*-
"""跑一次完整覆蓋任務，記錄「帶標籤的軌跡」與地圖，供覆蓋率拆帳使用。

搭配 test/tools/coverage_budget.py，量法見 docs/simulation_results.md 第 18 節。

用法（模擬要先用 ./run_demo.sh 起來）:
    python3 test/tools/coverage_run.py <prefix> [秒數上限]
    python3 test/tools/coverage_run.py --map-only          # 只建圖開車
    python3 test/tools/coverage_run.py <prefix> 1500 --no-drive
        # 不建圖開車（前後量測用；請透過 test/tools/ab_run.sh 執行）


標籤來自 /mission_status.current_label，所以每一個軌跡取樣點都知道
它當下屬於哪一段 (周邊環繞 / 割草線 / approach)。

<prefix>_traj.csv 的欄位：

    t, x, y, label, odom_vx, odom_wz, cmd_vx, cmd_wz, cmd_age, yaw

  yaw  /odom 的朝向 (rad)（階段 29 新增在最後一欄，原本 9 欄的位置與意義不變）。
       車子被擋住時 twist 會抖，把 odom_wz 積分回去得不到可靠的朝向，所以直接錄。

  odom_vx / odom_wz  /odom 的 twist，車子**實際**的線速度與角速度
  cmd_vx  / cmd_wz   /cmd_vel 最後一筆的 linear.x 與 angular.z，也就是**指令**
  cmd_age  這一筆 cmd 是多久以前收到的 (秒)。/cmd_vel 是 20 Hz、
           /odom 是 30 Hz，正常情況下 < 0.05 s；數值變大代表那段時間
           根本沒有人在下指令，這本身就是資訊。

  指令與實際分開錄，是因為「車子在原地轉圈」有兩種完全不同的成因：
  控制器一直在下轉向指令，或是指令是直線但車子被擋住走不動。
  只錄位置分不出來 —— 階段 23 的那次診斷就是卡在這裡。
  取樣時刻與 x / y / label 完全對齊：全部在同一個 /odom 回呼裡寫進同一列。

<prefix>_plan.csv（階段 29）：manager 發的 /f2c_path（F2C 規劃出來的完整路徑）的航點，
  欄位 x, y，依原順序。coverage_budget.py 用它算「規劃上涵蓋了哪裡」，
  讓 E2a 只由計畫決定、不受軌跡影響。多輪規劃時存最後一輪。
"""
import csv, math, sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, PolygonStamped
from mowerbot_interfaces.msg import MissionStatus
from mowerbot_interfaces.srv import SetDriveMode

_pos = [a for a in sys.argv[1:] if not a.startswith('--')]
OUT = _pos[0] if _pos else 'unused'
LIMIT = float(_pos[1]) if len(_pos) > 1 else 1500.0
# 前後量測（同一張地圖）用的兩個選項，流程見 test/tools/ab_run.sh：
#   --map-only  只做建圖開車，不切 mode 1、不存檔（之後由 save_map.sh 存圖）
#   --no-drive  不開車建圖：/map 是 map_server 發布的存檔 .pgm，
#               開車只會讓定位模式把即時掃描併進地圖，規劃就不可重現
MAP_ONLY = '--map-only' in sys.argv
NO_DRIVE = '--no-drive' in sys.argv
rclpy.init(); n = Node('coverage_run')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
odom, maps, bnds = [], [], []
state = {'label': '', 'state': 0, 'total': 0, 'done': 0, 'skipped': 0}
pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
# /cmd_vel 最後一筆：(收到的時刻, linear.x, angular.z)。
# 沒收到過就用 t=0，這樣 cmd_age 會是一個很大的數字，一眼看得出「沒有指令」。
cmd = {'t': 0.0, 'vx': 0.0, 'wz': 0.0}
n.create_subscription(
    Twist, '/cmd_vel',
    lambda m: cmd.update(t=time.time(), vx=m.linear.x, wz=m.angular.z), 50)
def on_odom(m):
    q = m.pose.pose.orientation
    pose['x'] = m.pose.pose.position.x
    pose['y'] = m.pose.pose.position.y
    pose['yaw'] = math.atan2(2*(q.w*q.z), 1-2*(q.z*q.z))
    now = time.time()
    odom.append((now, pose['x'], pose['y'], state['label'],
                 m.twist.twist.linear.x, m.twist.twist.angular.z,
                 cmd['vx'], cmd['wz'], now - cmd['t'] if cmd['t'] else 999.0,
                 pose['yaw']))
n.create_subscription(Odometry, '/odom', on_odom, 50)
n.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), qos)
plans = []
n.create_subscription(Path, '/f2c_path', lambda m: plans.append(
    [(p.pose.position.x, p.pose.position.y) for p in m.poses]), qos)
n.create_subscription(PolygonStamped, '/f2c_boundary',
                      lambda m: bnds.append([(p.x, p.y) for p in m.polygon.points]), 10)
def on_ms(m):
    state['label'] = m.current_label
    state['state'] = m.state
    state['total'] = m.total_segments
    state['done'] = m.completed_segments
    state['skipped'] = m.skipped_segments
n.create_subscription(MissionStatus, '/mission_status', on_ms, 10)
pub = n.create_publisher(Twist, '/cmd_vel_joy', 10)
cli = n.create_client(SetDriveMode, 'change_mower_mode'); cli.wait_for_service(timeout_sec=30.0)
def mode(m_):
    r = SetDriveMode.Request(); r.mode = m_
    f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
    return f.result().success if f.result() else None
def drive(v, w, sec):
    m_ = Twist(); m_.linear.x = v; m_.angular.z = w
    t = time.time()
    while time.time()-t < sec:
        pub.publish(m_); rclpy.spin_once(n, timeout_sec=0.05)

if NO_DRIVE:
    # map_server 只發一次 /map，map_to_boundary 也只算一次邊界；
    # 等 30 s 讓 manager 收到邊界再開始。
    mode(2)
    t0 = time.time()
    while time.time()-t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    m_ = maps[-1] if maps else None
    print('收到 /map %d 次，最後一張 %s；收到邊界 %d 次' % (
        len(maps), ('%dx%d' % (m_.info.width, m_.info.height)) if m_ else '-',
        len(bnds)))
else:
    print('切 mode 2，開車繞一圈建圖...'); mode(2)
    # 3 m 見方的小方塊就夠：雷達 12 m，在 12x12 的草坪中央就看得到所有牆面。
    # 走 6 m 會直接頂到牆，車子還會停在角落轉不了身。
    for _ in range(4):
        drive(0.45, 0.0, 6.5); drive(0.0, 0.6, 2.6); drive(0.0, 0.0, 0.6)
    drive(0.0, 0.0, 2.0)

    print('開回場中央 (用 odom 的實際朝向)...')
    for _ in range(80):
        rclpy.spin_once(n, timeout_sec=0.05)
        x, y, yaw = pose['x'], pose['y'], pose['yaw']
        if math.hypot(x, y) < 0.8:
            break
        want = math.atan2(-y, -x)
        err = math.atan2(math.sin(want-yaw), math.cos(want-yaw))
        if abs(err) > 0.20:
            drive(0.0, 0.6 if err > 0 else -0.6, 0.25)
        else:
            drive(0.45, 0.0, 0.5)
    drive(0.0, 0.0, 1.5)
    print('回到 (%.2f, %.2f)' % (pose['x'], pose['y']))

    t0 = time.time()
    while time.time()-t0 < 20 and not (maps and bnds): rclpy.spin_once(n, timeout_sec=0.2)
    print('地圖 %s，邊界 %s 頂點' % (bool(maps), len(bnds[-1]) if bnds else 0))

if MAP_ONLY:
    rclpy.shutdown()
    sys.exit(0)

del odom[:]
print('切 mode 1 開始覆蓋任務...'); mode(1)
t0 = time.time(); last = 0
while time.time()-t0 < LIMIT:
    rclpy.spin_once(n, timeout_sec=0.05)
    if state['state'] in (MissionStatus.STATE_DONE, MissionStatus.STATE_ABORTED) \
            and time.time()-t0 > 20:
        break
    if time.time()-t0-last > 60:
        last = time.time()-t0
        print('  t=%4.0fs state=%d 完成 %d/%d 跳過 %d  [%s]'
              % (last, state['state'], state['done'], state['total'],
                 state['skipped'], state['label']))
print('任務結束：state=%d 完成 %d/%d 跳過 %d，耗時 %.0f s'
      % (state['state'], state['done'], state['total'], state['skipped'], time.time()-t0))
mode(2)
with open(OUT + '_traj.csv', 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['t', 'x', 'y', 'label',
                'odom_vx', 'odom_wz', 'cmd_vx', 'cmd_wz', 'cmd_age', 'yaw'])
    for r in odom:
        w.writerow(['%.3f' % r[0], '%.4f' % r[1], '%.4f' % r[2], r[3],
                    '%.4f' % r[4], '%.4f' % r[5],
                    '%.4f' % r[6], '%.4f' % r[7], '%.3f' % r[8], '%.4f' % r[9]])
with open(OUT + '_plan.csv', 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(['x', 'y'])
    for px, py in (plans[-1] if plans else []):
        w.writerow(['%.4f' % px, '%.4f' % py])
print('已存 %s_plan.csv (收到 /f2c_path %d 次，最後一次 %d 個航點)'
      % (OUT, len(plans), len(plans[-1]) if plans else 0))
m = maps[-1]
g = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
np.savez(OUT + '_map.npz', grid=g, res=m.info.resolution,
         ox=m.info.origin.position.x, oy=m.info.origin.position.y)
print('已存 %s_traj.csv (%d 筆) 與 %s_map.npz' % (OUT, len(odom), OUT))
rclpy.shutdown()
