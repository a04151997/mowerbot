# -*- coding: utf-8 -*-
"""撞牆那一段的拆解 (階段 38，docs/stage38_collision_brief.md 的數據來源)。只讀 bag，不改任何東西。

用法：python3 test/tools/collision_brief.py <bag 目錄> [--wall-y -6.00] [--label 割草線 1/34]

1. 碰撞瞬間：第一個「車身角點 y <= 牆面」的 /odom 時刻 —— base_link 位姿、四角座標、
   哪一角先越界、cmd_vel 與 odom 實際速度、越界深度。
2. 碰撞前 3 秒：cmd_vel_nav 時間序列；每一筆 /evaluation：DWB 選中的軌跡與各 critic 分數，
   對照同一筆裡「最好的原地旋轉候選」(|vx| < 0.01) 的分數 —— 哪一個 critic 讓弧線勝出。
   DWB 的 total 越低越好；每個 critic 的貢獻 = raw_score x scale。
3. 同一時段 local_costmap 在車頭兩角、base_link 那一格的 cost。
"""
import argparse, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from mowerbot_description import vehicle_geometry

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--wall-y', type=float, default=-6.00)
ap.add_argument('--label', default='割草線 1/')
ap.add_argument('--window', type=float, default=3.0)
a = ap.parse_args()
g = vehicle_geometry.load()
CORNERS = {'左前': (g.front_extent, g.lateral_half_extent), '右前': (g.front_extent, -g.lateral_half_extent),
           '右後': (-g.rear_extent, -g.lateral_half_extent), '左後': (-g.rear_extent, g.lateral_half_extent)}

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
want = ('/odom', '/cmd_vel_nav', '/cmd_vel', '/evaluation', '/local_costmap/costmap', '/mission_status')
reader.set_filter(rosbag2_py.StorageFilter(topics=list(want)))
data = {k: [] for k in want}
while reader.has_next():
    topic, raw, t = reader.read_next()
    data[topic].append((t * 1e-9, deserialize_message(raw, get_message(types[topic]))))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def corners(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return {k: (x + c * cx - s * cy, y + s * cx + c * cy) for k, (cx, cy) in CORNERS.items()}


# 割草線 1 的時段
lab_t = [t for t, m in data['/mission_status'] if m.current_label.startswith(a.label)]
if not lab_t:
    sys.exit('bag 裡沒有 %s' % a.label)
t_sw0 = lab_t[0]
odom = [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation),
         m.twist.twist.linear.x, m.twist.twist.angular.z) for t, m in data['/odom'] if t >= t_sw0]
hit = None
for o in odom:
    cs = corners(o[1], o[2], o[3])
    first = min(cs.items(), key=lambda kv: kv[1][1])
    if first[1][1] <= a.wall_y + 0.005:
        hit = (o, cs, first)
        break
if hit is None:
    sys.exit('割草線開始後沒有任何角點碰到 y = %.2f' % a.wall_y)
(t_hit, x, y, th, vx, wz), cs, first = hit


def last_before(topic, t):
    xs = [(tt, m) for tt, m in data[topic] if tt <= t]
    return xs[-1] if xs else (None, None)


_, cmd = last_before('/cmd_vel_nav', t_hit)
print('=' * 78)
print(g.stamp())
print('=' * 78)
print('【1】碰撞瞬間 (第一個角點 y <= 牆面 %.2f + 0.005)：割草線開始後 %.2f s' % (a.wall_y, t_hit - t_sw0))
print('  base_link x %.3f  y %.3f  yaw %.3f rad (%.1f°)' % (x, y, th, math.degrees(th)))
for k, (cx, cy) in cs.items():
    print('  %s角 (%.3f, %.3f)%s' % (k, cx, cy, '   <- 最先越界' if k == first[0] else ''))
print('  cmd_vel_nav  vx %.3f  wz %.3f    odom 實際  vx %.3f  wz %.3f' % (cmd.linear.x, cmd.angular.z, vx, wz))
# 之後 15 s 內角點最深到哪裡 (模擬的接觸會把車擋住，越界深度就是穿透量)
deep = min(min(c[1] for c in corners(o[1], o[2], o[3]).values()) for o in odom if t_hit <= o[0] <= t_hit + 15)
print('  牆面 y = %.2f；之後 15 s 內角點最低 y = %.4f，越界 (穿透) 深度 = %.4f m' % (a.wall_y, deep, a.wall_y - deep))
stuck = [o for o in odom if t_hit <= o[0] <= t_hit + 15]
print('  之後 15 s：base_link 移動 %.3f m，平均 odom vx %.3f' % (
    math.hypot(stuck[-1][1] - stuck[0][1], stuck[-1][2] - stuck[0][2]), np.mean([o[4] for o in stuck])))

print('\n【2a】碰撞前 %.1f s 的 cmd_vel_nav (DWB 輸出) 與 odom' % a.window)
print('   t-碰撞    cmd vx   cmd wz  |  odom vx  odom wz  |  base x  base y  yaw°   最低角 y')
for t, m in data['/cmd_vel_nav']:
    if t_hit - a.window <= t <= t_hit + 0.3:
        _, om = last_before('/odom', t)
        ox, oy, oth = om.pose.pose.position.x, om.pose.pose.position.y, yaw_of(om.pose.pose.orientation)
        low = min(c[1] for c in corners(ox, oy, oth).values())
        print('  %+6.2f    %6.3f  %6.3f  |  %6.3f  %6.3f  | %6.2f  %6.2f  %6.1f  %7.3f'
              % (t - t_hit, m.linear.x, m.angular.z, om.twist.twist.linear.x, om.twist.twist.angular.z,
                 ox, oy, math.degrees(oth), low))

print('\n【2b】DWB 每一次評估：選中的軌跡 vs 最好的原地旋轉候選 (total 越低越好；分數 = raw x scale)')
critics = None
rows = []
for t, ev in data['/evaluation']:
    if not (t_hit - a.window <= t <= t_hit + 0.2) or not ev.twists:
        continue
    best = ev.twists[ev.best_index]
    valid = [tw for tw in ev.twists if tw.total >= 0]
    spins = [tw for tw in valid if abs(tw.traj.velocity.x) < 0.01 and abs(tw.traj.velocity.theta) > 0.05]
    spin = min(spins, key=lambda tw: tw.total) if spins else None
    if critics is None:
        critics = [s.name for s in best.scores]
    rows.append((t, best, spin, len(ev.twists), len(valid), len(spins)))
for t, best, spin, n, nv, ns in rows[::max(1, len(rows) // 12)]:
    def part(tw):
        return ' '.join('%s %.1f' % (s.name[:6], s.raw_score * s.scale) for s in tw.scores)
    print('  %+5.2f s  候選 %d / 有效 %d / 有效原地旋轉 %d' % (t - t_hit, n, nv, ns))
    print('     選中  vx %.2f wz %+.2f total %8.2f | %s'
          % (best.traj.velocity.x, best.traj.velocity.theta, best.total, part(best)))
    if spin:
        print('     原地  vx %.2f wz %+.2f total %8.2f | %s'
              % (spin.traj.velocity.x, spin.traj.velocity.theta, spin.total, part(spin)))
        diff = {s.name: (s.raw_score * s.scale) for s in spin.scores}
        for s in best.scores:
            diff[s.name] -= s.raw_score * s.scale
        top = sorted(diff.items(), key=lambda kv: -kv[1])[:3]
        print('     原地旋轉比選中的多出 (正值 = 原地吃虧)：%s'
              % ', '.join('%s %+.1f' % kv for kv in top))
    else:
        print('     (這一筆沒有有效的原地旋轉候選)')

# 彙總：整個視窗內原地旋轉輸在哪個 critic
agg = {}
cnt = 0
for t, best, spin, n, nv, ns in rows:
    if not spin:
        continue
    cnt += 1
    for s in spin.scores:
        agg[s.name] = agg.get(s.name, 0.0) + s.raw_score * s.scale
    for s in best.scores:
        agg[s.name] -= s.raw_score * s.scale
print('\n  視窗內 %d 筆評估，有原地旋轉候選的 %d 筆；平均「原地 - 選中」各 critic 分差：' % (len(rows), cnt))
for k, v in sorted(agg.items(), key=lambda kv: -kv[1]):
    print('     %-14s %+8.2f' % (k, v / max(cnt, 1)))
# 選中軌跡的末端車頭角預測位置
print('\n【2c】選中軌跡 (sim_time 2 s) 每一個預測位姿的最低角點 —— DWB 自己的預測會不會穿牆')
for t, best, spin, n, nv, ns in rows[::max(1, len(rows) // 8)]:
    lows = [(min(c[1] for c in corners(p.x, p.y, p.theta).values()), k) for k, p in enumerate(best.traj.poses)]
    low, k = min(lows)
    print('  %+5.2f s  選中 vx %.2f wz %+.2f：預測軌跡上最低角 y %.3f (第 %d/%d 個位姿，牆 %.2f) %s'
          % (t - t_hit, best.traj.velocity.x, best.traj.velocity.theta, low, k + 1, len(lows), a.wall_y,
             '-> 預測就已穿牆 %.3f m' % (a.wall_y - low) if low < a.wall_y else ''))
print('\n【3】local_costmap 在車頭兩角與 base_link 那一格的 cost (0 自由 / 99 inscribed / 100 致命，OccupancyGrid 尺度)')
for t, cm in data['/local_costmap/costmap']:
    if not (t_hit - a.window - 0.5 <= t <= t_hit + 1.0):
        continue
    _, om = last_before('/odom', t)
    ox, oy, oth = om.pose.pose.position.x, om.pose.pose.position.y, yaw_of(om.pose.pose.orientation)
    info = cm.info

    def cost(px, py):
        i = int((px - info.origin.position.x) / info.resolution)
        j = int((py - info.origin.position.y) / info.resolution)
        if 0 <= i < info.width and 0 <= j < info.height:
            return cm.data[j * info.width + i]
        return None
    cs3 = corners(ox, oy, oth)
    # 牆面在 costmap 裡的最大 cost (確認 costmap 有看到牆)
    wall = max((cost(xx, a.wall_y - 0.05) or -1) for xx in np.arange(ox - 1.5, ox + 1.5, 0.05))
    print('  %+5.2f s  base_link %s  左前 %s  右前 %s  | 牆面 (y=%.2f) 那一排最大 %s'
          % (t - t_hit, cost(ox, oy), cost(*cs3['左前']), cost(*cs3['右前']), a.wall_y - 0.05, wall))
