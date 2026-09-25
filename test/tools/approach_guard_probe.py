#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""approach 迴圈守門的確定性檢查（不需要 Gazebo、不需要 Nav2）。

為什麼不用「跑一趟任務看它有沒有觸發」來測：
階段 25 的 approach_goal_checker 把 approach 的終點飄移修掉之後，
正常情況下守門**根本不會被觸發** —— 用真實任務當測試情境，
這一項會在修好之後變成「永遠測不到東西」。

所以改成直接建一個 mower_manager 節點（**完全不 spin**，不會送出任何速度），
把「車子在哪裡」換成腳本控制的座標，再逐次呼叫 maybe_insert_approach()。
輸出是機器可讀的 KEY=VALUE，由 smoke_test 的 Phase O 判定。
"""
import math
import sys

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from mowerbot_action.manager import MowerManager


def big_boundary():
    """一個夠大的正方形邊界，讓所有淨空檢查都通過"""
    msg = PolygonStamped()
    msg.header.frame_id = 'map'
    for x, y in ((-20.0, -20.0), (20.0, -20.0), (20.0, 20.0), (-20.0, 20.0)):
        msg.polygon.points.append(Point32(x=x, y=y, z=0.0))
    return msg.polygon


def straight_path(x0, y0, n=20, step=0.1):
    p = Path()
    p.header.frame_id = 'map'
    for k in range(n):
        ps = PoseStamped()
        ps.header = p.header
        ps.pose.position.x = x0 + k * step
        ps.pose.position.y = y0
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def main():
    # mower_manager 的 footprint 與刀盤寬沒有預設值 (階段 30)，
    # 照 mower_control.launch.py 一樣從 vehicle.yaml 傳進去。
    import os
    import yaml
    from ament_index_python.packages import get_package_share_directory
    with open(os.path.join(get_package_share_directory('mowerbot_description'),
                           'config', 'vehicle.yaml')) as fh:
        vehicle = yaml.safe_load(fh)
    rclpy.init(args=['--ros-args'] + [
        a for k in ('footprint_length', 'footprint_width', 'blade_width')
        for a in ('-p', '%s:=%r' % (k, vehicle[k]))])
    node = MowerManager()
    node.latest_boundary = big_boundary()
    node.latest_obstacles = []
    robot = {'xy': (0.0, 0.0)}
    node.get_robot_pose_in_map = lambda: robot['xy']

    warnings = []
    real_warn = node.get_logger().warn
    def capture(msg, *a, **kw):
        warnings.append(msg)
        return real_warn(msg, *a, **kw)
    node.get_logger().warn = capture

    def fresh(label='割草線 1/1', target_x=5.0):
        node._swath_queue = [(straight_path(target_x, 0.0), label)]
        node._current_swath_idx = 0
        node._reset_approach_tracking()
        del warnings[:]

    def step(x):
        """把車子放到 (x, 0)，問一次要不要插 approach；
        插了就把它當成已經走完並移除，模擬 approach 成功回來。"""
        robot['xy'] = (x, 0.0)
        before = len(node._swath_queue)
        inserted = node.maybe_insert_approach()
        if inserted:
            node._swath_queue.pop(node._current_swath_idx)
        assert len(node._swath_queue) == before
        return inserted

    out = {}

    # ---- A 沒有改善：第 2 次就該被擋下來 ----
    fresh()
    a1 = step(0.0)          # 離目標 5.00 m -> 插
    a2 = step(0.05)         # 離目標 4.95 m，只改善 0.05 m -> 擋
    out['A_first_inserted'] = a1
    out['A_second_inserted'] = a2
    out['A_warned_no_progress'] = any('未改善距離' in w for w in warnings)

    # ---- B 有改善但達上限：第 3 次該被擋下來 ----
    fresh()
    b = [step(0.0), step(2.0), step(3.5)]
    out['B_inserts'] = ''.join('1' if x else '0' for x in b)
    out['B_warned_max_tries'] = any('達到上限' in w for w in warnings)

    # ---- C 對照組：一路順利接近，不可以有任何守門警告 ----
    # 每一次都大幅縮短距離，最後進到 0.5 m 門檻內自然結束。
    fresh()
    c = [step(0.0), step(3.0)]
    robot['xy'] = (4.7, 0.0)            # 離目標 0.30 m < APPROACH_SKIP_DISTANCE
    c.append(node.maybe_insert_approach())
    out['C_inserts'] = ''.join('1' if x else '0' for x in c)
    out['C_no_warning'] = not any(
        ('未改善距離' in w or '達到上限' in w) for w in warnings)

    # ---- D 放棄接近時目標超出 local costmap 範圍：要跳過該段，不是直接送出 ----
    # 階段 26。直接送出的話起點落在 local costmap 之外，控制器 0.4 s 回 0 poses。
    # 範圍由 costmap 本身算 (max(寬,高) x 解析度 / 2)，這裡給 5 m x 5 m -> 2.50 m，
    # 與 nav2_params.yaml 的 local_costmap 相同。
    cm = OccupancyGrid()
    cm.info.width, cm.info.height, cm.info.resolution = 100, 100, 0.05
    node.local_costmap = cm
    fresh()                              # 目標 5.00 m 外
    node._skipped = []
    d1 = step(0.0)                       # 5.00 m -> 插
    d2 = step(0.05)                      # 4.95 m，沒進展，且 > 2.50 m -> 跳過
    out['D_reach'] = '%.2f' % node.local_costmap_reach()
    out['D_first_inserted'] = d1
    out['D_second_inserted'] = d2
    out['D_skipped'] = ','.join(s[0] for s in node._skipped) or '-'
    out['D_idx_advanced'] = node._current_swath_idx == 1
    out['D_warned_out_of_range'] = any('超出局部代價地圖範圍' in w for w in warnings)

    # ---- E 對照組：一樣沒進展，但目標在範圍內 -> 維持直接送出，不可以跳過 ----
    fresh(target_x=2.0)                  # 目標 2.00 m 外 (< 2.50 m)
    node._skipped = []
    e1 = step(0.0)                       # 2.00 m -> 插
    e2 = step(0.05)                      # 1.95 m，沒進展，但在範圍內 -> 直接送出
    out['E_first_inserted'] = e1
    out['E_second_inserted'] = e2
    out['E_skipped'] = ','.join(s[0] for s in node._skipped) or '-'
    out['E_idx_unchanged'] = node._current_swath_idx == 0
    out['E_warned_send'] = any('未改善距離' in w and '直接送出該段落' in w
                               for w in warnings)
    out['E_no_out_of_range'] = not any('超出局部代價地圖範圍' in w for w in warnings)
    node.local_costmap = None

    out['MAX_TRIES'] = node.APPROACH_MAX_TRIES
    out['MIN_IMPROVEMENT'] = node.APPROACH_MIN_IMPROVEMENT
    for k, v in out.items():
        print('%s=%s' % (k, v))

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
