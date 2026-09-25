#!/usr/bin/env python3
"""Phase O 失敗段落的軌跡診斷（階段 27，只量測、不修任何東西）。

用法：
    # 1. 跑 --phases=O 的同時，在同一個 ROS_DOMAIN_ID (77) 錄 bag：
    #    ros2 bag record -a -x '/evaluation|/slam_map.*|/map_updates|/main_camera.*' -o obag --use-sim-time
    #    （相機影像要排除，否則一趟 Phase O 約 14 GB）
    # 2. 分析某一段（label 要與 manager log 的字樣相同）：
    python3 test/tools/o_failure_trace.py <bag 目錄> '割草線 6/15'

bag 用 --use-sim-time 錄，訊息時間是模擬時間；manager 的 log 是系統時間。
兩者用 bag 裡的 /rosout 對齊：找到 manager「送出任務 [label]」那一則 /rosout，
它在 bag 裡的時間就是該段開始的模擬時間，結束同理（「label 完成」或「[label] 失敗」）。

輸出：
  - 段落開始 / 結束時的 map 座標位置與朝向
  - 這一段收到的路徑 (/received_global_plan) 的起訖點
  - 每 0.5 s 一列：位置、朝向、離路徑直線的橫向偏移、/cmd_vel、離箱子的距離
  - 結束當下 local costmap 在車體 footprint 內、前方與左側的佔據情況
"""
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OBST = (-2.5, -0.75, 0.5)          # 與 smoke_test.py 的 O_OBSTACLE 相同


def _load_vehicle():
    """車輛幾何的單一來源：mowerbot_description/config/vehicle.yaml（安裝後的那一份）"""
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory('mowerbot_description'),
                           'config', 'vehicle.yaml')) as fh:
        return yaml.safe_load(fh)


_VEHICLE = _load_vehicle()
HALF_L = _VEHICLE['footprint_length'] / 2.0    # footprint（vehicle.yaml）
HALF_W = _VEHICLE['footprint_width'] / 2.0

TOPICS = ['/rosout', '/tf', '/odom', '/cmd_vel', '/received_global_plan',
          '/local_costmap/costmap', '/local_plan']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D 位姿合成 a∘b，位姿 = (x, y, yaw)"""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + math.cos(at) * bx - math.sin(at) * by,
            ay + math.sin(at) * bx + math.cos(at) * by, at + bt)


def box_dist(px, py):
    cx, cy, h = OBST
    return math.hypot(max(abs(px - cx) - h, 0.0), max(abs(py - cy) - h, 0.0))


def footprint_box_dist(x, y, th):
    """車體矩形到箱子的最短距離（在車體輪廓上取樣，重疊回 0）"""
    best = 1e9
    c, s = math.cos(th), math.sin(th)
    for i in range(41):
        for j in range(41):
            if 0 < i < 40 and 0 < j < 40:
                continue
            lx = -HALF_L + 2 * HALF_L * i / 40.0
            ly = -HALF_W + 2 * HALF_W * j / 40.0
            best = min(best, box_dist(x + c * lx - s * ly, y + s * lx + c * ly))
    return best


def main():
    bag, label = sys.argv[1], sys.argv[2]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    cls = {t: get_message(types[t]) for t in TOPICS if t in types}

    t_start = t_end = None
    end_msg = ''
    map_odom = []      # (t, pose)
    odom_base = []
    cmd = []
    plans = []
    costmaps = []
    local_plans = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        t = t * 1e-9
        if topic == '/rosout':
            m = deserialize_message(data, cls[topic])
            if m.name != 'mower_manager':
                continue
            if t_start is None and ('送出任務 [%s]' % label) in m.msg:
                t_start = t
            elif t_start is not None and t_end is None and (
                    ('%s 完成' % label) in m.msg or ('[%s] 失敗' % label) in m.msg):
                t_end = t
                end_msg = m.msg
            continue
        if t_start is not None and t_end is not None and t > t_end + 3.0:
            break
        if topic == '/tf':
            m = deserialize_message(data, cls[topic])
            for tr in m.transforms:
                p = (tr.transform.translation.x, tr.transform.translation.y,
                     yaw_of(tr.transform.rotation))
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    map_odom.append((t, p))
        elif topic == '/odom':
            # 車子位置用 /odom（約 30 Hz）而不是 /tf 裡的 odom->base_footprint：
            # 實測 bag 裡後者稀疏，位置會卡住好幾秒再跳 2 m
            m = deserialize_message(data, cls[topic])
            odom_base.append((t, (m.pose.pose.position.x, m.pose.pose.position.y,
                                  yaw_of(m.pose.pose.orientation))))
        elif t_start is None:
            continue
        elif topic == '/cmd_vel':
            m = deserialize_message(data, cls[topic])
            cmd.append((t, m.linear.x, m.angular.z))
        elif topic == '/received_global_plan':
            m = deserialize_message(data, cls[topic])
            if m.poses:
                plans.append((t, m.header.frame_id,
                              [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif topic == '/local_plan':
            m = deserialize_message(data, cls[topic])
            local_plans.append((t, m.header.frame_id,
                                [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif topic == '/local_costmap/costmap':
            costmaps.append((t, deserialize_message(data, cls[topic])))

    if t_start is None or t_end is None:
        print('找不到「送出任務 [%s]」或它的結束訊息' % label)
        return 1

    def latest(seq, t):
        best = None
        for tt, p in seq:
            if tt <= t:
                best = p
            else:
                break
        return best

    def pose_map(t):
        mo, ob = latest(map_odom, t), latest(odom_base, t)
        if mo is None or ob is None:
            return None
        return compose(mo, ob)

    def pose_odom(t):
        return latest(odom_base, t)

    print('段落 = %s' % label)
    print('開始（sim）= %.2f s，結束 = %.2f s，歷時 %.2f s' % (t_start, t_end, t_end - t_start))
    print('結束訊息 = %s' % end_msg)
    first_plan = next((p for p in plans if p[0] >= t_start), None)
    if first_plan:
        pts = first_plan[2]
        print('收到的路徑 (%s)：%d 點，起點 (%.2f, %.2f) 終點 (%.2f, %.2f)'
              % (first_plan[1], len(pts), pts[0][0], pts[0][1], pts[-1][0], pts[-1][1]))
    for name, tt in (('開始', t_start), ('結束', t_end)):
        p = pose_map(tt)
        if p:
            print('%s 位置 (map) = (%.2f, %.2f)，朝向 %.1f°，車體離箱子 %.2f m，中心離箱子 %.2f m'
                  % (name, p[0], p[1], math.degrees(p[2]),
                     footprint_box_dist(*p), box_dist(p[0], p[1])))
    mo0, mo1 = latest(map_odom, t_start), latest(map_odom, t_end)
    if mo0 and mo1:
        print('map->odom 在這一段內的變化 = dx %.3f dy %.3f dyaw %.2f°'
              % (mo1[0] - mo0[0], mo1[1] - mo0[1], math.degrees(mo1[2] - mo0[2])))

    y_line = first_plan[2][0][1] if first_plan else None
    print('')
    print('   t(s)    x(map)  y(map)  yaw(°)  橫偏(m)  車體離箱(m)   v(m/s)  w(rad/s)')
    k = 0.0
    while t_start + k <= t_end + 0.01:
        tt = t_start + k
        p = pose_map(tt)
        c = latest([(a, (b, cc)) for a, b, cc in cmd], tt)
        if p:
            print('  %5.1f  %7.2f %7.2f  %6.1f  %7s  %9.2f   %7s  %7s'
                  % (k, p[0], p[1], math.degrees(p[2]),
                     '%.2f' % (p[1] - y_line) if y_line is not None else '-',
                     footprint_box_dist(*p),
                     '%.2f' % c[0] if c else '-', '%.2f' % c[1] if c else '-'))
        k += 0.5

    # 結束當下的 local costmap：footprint 內與車頭前方 0.5 m 的最大 cost
    cm = latest([(a, b) for a, b in costmaps], t_end)
    po = pose_odom(t_end)
    if cm is not None and po is not None:
        info = cm.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution

        def cost_at(lx, ly):
            c, s = math.cos(po[2]), math.sin(po[2])
            wx, wy = po[0] + c * lx - s * ly, po[1] + s * lx + c * ly
            i, j = int((wx - ox) / res), int((wy - oy) / res)
            if 0 <= i < info.width and 0 <= j < info.height:
                return cm.data[j * info.width + i]
            return -2
        inside = [cost_at(-HALF_L + 0.05 * a, -HALF_W + 0.05 * b)
                  for a in range(20) for b in range(14)]
        front = [cost_at(HALF_L + 0.05 * a, -HALF_W + 0.05 * b)
                 for a in range(1, 11) for b in range(14)]
        left = [cost_at(-HALF_L + 0.05 * a, HALF_W + 0.05 * b)
                for a in range(20) for b in range(1, 11)]
        print('')
        print('結束當下 local costmap（odom frame，%.2f s 的那一張）：' % (
            [a for a, _b in costmaps if a <= t_end][-1]))
        # /local_costmap/costmap 是 OccupancyGrid：100 = 致命，99 = 內切
        print('  footprint 內最大值 = %d（OccupancyGrid：100 致命 / 99 內切）' % max(inside))
        print('  車頭前方 0.5 m 內最大值 = %d，= 100 的格數 = %d'
              % (max(front), sum(1 for v in front if v == 100)))
        print('  車身左側 0.5 m 內最大值 = %d，= 100 的格數 = %d'
              % (max(left), sum(1 for v in left if v == 100)))
    lp = latest([(a, (b, c)) for a, b, c in local_plans], t_end - 0.1)
    if lp:
        fr, pts = lp
        print('結束前最後一個 /local_plan (%s)：%d 點%s' % (
            fr, len(pts), '，終點 (%.2f, %.2f)' % pts[-1] if pts else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
