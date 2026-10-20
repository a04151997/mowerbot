#!/usr/bin/env python3
"""mowerbot 網頁人機介面 (web HMI)。

N150 開機就由 systemd 啟動 (mowerbot-hmi.service)。Mac 或手機連上 mowerbot-n150 熱點，
用瀏覽器開 http://10.42.0.1:8080 就能看狀態、切模式。畫面是 share/mowerbot_hmi/web/index.html，
這支負責：

  * 當 ROS 節點：訂閱車子的狀態、呼叫 mower_manager / map_autosave 的服務
  * 當網頁伺服器：/ 給畫面，/api/stream 用 Server-Sent Events 每 0.2 秒推一次狀態，
    /api/map.png 給地圖，/api/cmd 收按鍵
  * 跑 car-start / car-stop (跟在 SSH 打指令一樣)

只用 Python 標準函式庫 + rclpy + numpy + cv2：N150 在熱點上沒有網路，不能再裝套件。
車上程式 (bringup) 沒在跑的時候這支照樣活著，只是收不到資料；所以它不是 bringup 的一部分。

=========================================================================
【安全】跟 hmi_node.py (模擬用的 PyQt 介面) 同一套原則
=========================================================================
  * 網頁不能開車。手動開車只用手把：要有「放開就停」的 deadman，網頁做不到。
  * 網頁的「停止」= mode 4，跟 RB 一樣是軟體停止，要靠 Wi-Fi。真正的急停是馬達總開關。
  * 網頁斷線不會讓車子停。停車靠手把 (manager 的 auto_requires_joystick) 和車上其他停車條件；
    這支當掉、Wi-Fi 斷掉都不影響它們。
  * 會讓車子動的指令只有持有「操作權」的那一台能送；停止、確認警報哪一台都能按。
  * 只接受熱點 (10.42.0.0/24) 和本機的連線：N150 連到實驗室 Wi-Fi 更新的時候，實驗室裡的人開不到。

【執行緒】rclpy 在自己的執行緒 spin；HTTP 每個連線一個執行緒。
共用的狀態只在 self._lock 底下改，送給網頁的是 tick() 每 0.2 秒做好的一份 dict (整份換掉，不改內容)。
"""

import argparse
import collections
import ipaddress
import json
import math
import os
import re
import shutil
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time

import tf2_ros
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mowerbot_interfaces.msg import JoyStatus, MissionStatus, MowerStatus
from mowerbot_interfaces.srv import SetDriveMode

MODE_NAME = {0: '建圖', 1: '自動割草', 2: '手動', 3: '保留', 4: '停止中'}
STATE_NAME = {0: '待命', 1: '規劃中', 2: '執行中', 3: '割完了', 4: '已中止'}

# 多久沒收到就算斷了 (秒)
MOWER_STATUS_TIMEOUT = 1.5      # manager 5 Hz
JOY_STATUS_TIMEOUT = 1.0        # teleop 5 Hz，LB/RB 一變也立刻發
SCAN_TIMEOUT = 0.5              # 跟 manager 的 auto_input_timeout 一樣
# 車上程式剛啟動的這段時間，節點還在一個一個起來，先不報「沒有回應」
STARTUP_GRACE = 40.0

# 停止區：跟 navigation.launch.py 的 collision_monitor 一樣 (保險桿前 0.40 m、左右各加 0.05 m、超過 3 點就停)。
# 有收到 /polygon_stop (collision_monitor 自己發的) 就用它，沒有才用這組。
STOP_ZONE_LENGTH = 0.40
STOP_ZONE_SIDE_MARGIN = 0.05
STOP_ZONE_MAX_POINTS = 3

# 地圖格子 -> PNG 灰階 (index.html 的 CELL_LEVELS / decodeCell)
CELL_UNKNOWN, CELL_FREE, CELL_OCC, CELL_KEEPOUT = 0, 85, 170, 255
OCC_THRESHOLD = 50

# 送給網頁的光達點最多幾個 (S3 一圈約 3000 點，畫面用不到那麼多)
SCAN_POINTS_MAX = 400
# 割過的軌跡每隔多遠記一點 (m)
TRAIL_STEP = 0.10
TRAIL_MAX = 20000

# bringup 的 ros2 launch 命令列 (跟 common.sh 的 launch_running 一樣)
LAUNCH_PATTERN = r'launch mowerbot_bringup bringup_real\.launch\.py'
RUN_BRINGUP_RE = re.compile(r'run_bringup\.sh\s+(\S+)\s+(\S+)(?:\s+(\S+))?')

# 存一次地圖最多等多久 (map_autosave 的 timeout 是 120 s)；超過就當作存完了，免得一直擋住自動割草
SAVE_MAX = 130.0
# 草地邊界：開回起點多近才能「完成」(跟 lawn_boundary.MAX_CLOSE_GAP_M 一樣)
LAWN_CLOSE_GAP = 1.0
LAWN_WHAT = {'start': '開始記錄草地邊界', 'stop': '草地邊界完成', 'cancel': '放棄記錄草地邊界', 'clear': '清除草地邊界'}

# 「診斷」頁看的程式，跟它在做什麼
NODES = [
    ('/mower_bridge', '控制板'), ('/mower_manager', '模式與任務'), ('/mower_teleop', '手把'),
    ('/joy_node', '手把接收'), ('/lidar', '光達'), ('/slam_toolbox', '建圖與定位'),
    ('/controller_server', '導航'), ('/collision_monitor', '停止區'), ('/f2c_server', '割草路線'),
    ('/map_to_boundary', '割草範圍'), ('/keepout_server', '禁區'), ('/map_autosave', '存地圖'),
    ('/boundary_recorder', '草地邊界'),
]
# 「診斷」頁的資料更新率 (話題, 下限 Hz)
TOPICS = [('/scan', 8.0), ('/odom', 10.0), ('/joy', 10.0), ('/joy_status', 3.0),
          ('/mower_status', 3.0), ('/cmd_vel', 5.0)]

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


def now_ms():
    return int(time.time() * 1000)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tf_matrix(t):
    """geometry_msgs/Transform -> (cos, sin, tx, ty)，只用平面部分 (車子在平地上)。"""
    yaw = yaw_of(t.rotation)
    return math.cos(yaw), math.sin(yaw), t.translation.x, t.translation.y


def apply_tf(m, xs, ys):
    c, s, tx, ty = m
    return c * xs - s * ys + tx, s * xs + c * ys + ty


def point_in_poly(xs, ys, poly):
    """向量化的點在多邊形內 (ray casting)。poly: [(x, y), ...]"""
    inside = np.zeros(xs.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = (y1 > ys) != (y2 > ys)
        with np.errstate(divide='ignore', invalid='ignore'):
            xc = (x2 - x1) * (ys - y1) / (y2 - y1) + x1
        inside ^= cond & (xs < xc)
    return inside


def mission_text(message):
    """MissionStatus.message 去掉 manager 加在最前面的「暫停：…」那一行，剩下任務自己的訊息。"""
    if message.startswith('暫停：'):
        return message.split('\n', 1)[1] if '\n' in message else ''
    return message


def poly_area_perimeter(pts):
    area, perim = 0.0, 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        area += x0 * y1 - x1 * y0
        perim += math.hypot(x1 - x0, y1 - y0)
    return abs(area) / 2.0, perim


def decimate(points, step):
    """折線每隔 step 公尺留一點 (頭尾一定留)。"""
    if len(points) < 3:
        return [list(p) for p in points]
    out = [points[0]]
    for p in points[1:-1]:
        q = out[-1]
        if math.hypot(p[0] - q[0], p[1] - q[1]) >= step:
            out.append(p)
    out.append(points[-1])
    return [[round(x, 3), round(y, 3)] for x, y in out]


class RateMeter:
    """最近幾秒收到幾筆 -> Hz。"""

    def __init__(self, window=3.0):
        self.window = window
        self.t = collections.deque()

    def tick(self, t):
        self.t.append(t)
        while self.t and t - self.t[0] > self.window:
            self.t.popleft()

    def hz(self, t):
        while self.t and t - self.t[0] > self.window:
            self.t.popleft()
        if len(self.t) < 2:
            return 0.0
        return (len(self.t) - 1) / max(self.t[-1] - self.t[0], 1e-3)


class Alarms:
    """ISA-18.2 的警報狀態：進行中/未確認、進行中/已確認、已恢復/未確認、已恢復 (進歷史)。"""

    def __init__(self, keep=300):
        self.items = []
        self.seq = 0
        self.ver = 0
        self.keep = keep
        self.on_raise = None

    def _active(self, key):
        for a in self.items:
            if a['key'] == key and a['active']:
                return a
        return None

    def set(self, key, on, p=2, text=''):
        a = self._active(key)
        if on and a is None:
            self.seq += 1
            self.items.insert(0, {'id': self.seq, 'key': key, 'p': p, 'text': text,
                                  'tIn': now_ms(), 'tOut': None, 'active': True, 'acked': False})
            del self.items[self.keep:]
            self.ver += 1
            if self.on_raise:
                self.on_raise(p, text)
        elif on and a is not None and (a['text'] != text or a['p'] != p):
            # 同一件事，嚴重程度跟著情況變 (例如手把斷線：自動割草中是 1，其他時候是 2)
            a['text'], a['p'] = text, p
            self.ver += 1
        elif not on and a is not None:
            a['active'] = False
            a['tOut'] = now_ms()
            self.ver += 1

    def pulse(self, key, p, text):
        """一下子就過去的事 (例如某段被跳過)：留一筆「已恢復、未確認」。"""
        self.set(key, True, p, text)
        self.set(key, False)

    def ack(self, alarm_id=None):
        for a in self.items:
            if (alarm_id is None or a['id'] == alarm_id) and not a['acked']:
                a['acked'] = True
                self.ver += 1

    def snapshot(self):
        return [dict(a) for a in self.items]


class HmiWeb(Node):

    def __init__(self, car_start, car_stop, home):
        super().__init__('mower_web_hmi')
        self.car_start = car_start
        self.car_stop = car_stop
        self.home = home
        self._lock = threading.RLock()

        # ---- 收到的原始資料 (ROS 執行緒寫，tick 讀) ----
        self.t = {}                                     # 話題 -> 最後收到的單調時間
        self.rates = {name: RateMeter() for name, _ in TOPICS}
        self.mower = None
        self.mission = None
        self.joy = None
        self.scan = None
        self.odom = None
        self.cmd_out = 0.0
        self.cmd_nav = 0.0
        self.cmd_nav_raw_t = None
        self.mapping = False
        self.map_msg = None
        self.map_dirty = False
        self.keepout_msg = None
        self.last_save = ''
        self.zone_poly = None                           # base_footprint 座標
        self.path_pts = None
        self.boundary_pts = None                        # /f2c_boundary 最新的 (跟著建圖一直更新)
        self.route_boundary = None                      # 路線送來那一刻的割草範圍
        self._shown_boundary = None
        # 草地邊界 (boundary_recorder)：沿草地邊緣開一圈記下來的範圍，割草範圍只取它裡面
        self.lawn_pts = None                            # /lawn_boundary
        self.lawn_ver = 0
        self.rec_on = False                             # /boundary_recorder/recording
        self.rec_pts = None                             # 正在記錄的軌跡
        self.rec_len = 0.0
        self.rec_ver = 0

        # ---- 送給網頁的 ----
        self.frame = {}
        self.heavy = {'alarms': ([], 0), 'events': ([], 0), 'route': (None, 0),
                      'boundary': (None, 0), 'trail': (None, 0), 'lawn': (None, 0), 'rec': (None, 0)}
        self.map_png = None
        self.map_meta = {'ready': False, 'ver': 0}
        self.map_ver = 0
        self.alarms = Alarms()
        self.events = collections.deque(maxlen=300)
        self.events_ver = 0
        self.trail = []
        self.trail_ver = 0
        self.trail_sent_t = 0.0
        self.route_ver = 0
        self.boundary_ver = 0

        # ---- 操作權與連線中的裝置 ----
        self.web_clients = {}                               # client_id -> {'label', 'seen', 'streams'}
        self.holder = None
        self.holder_label = ''

        # ---- 車上程式 (bringup) ----
        self.prog = {'running': False, 'busy': None, 'since': None, 'lidar': True, 'scale': None, 'output': ''}
        self.prog_seen_up = None                        # 這次開始跑的單調時間
        self.expect_down = False                        # 是網頁 (或 car-stop) 關的就不報警
        self._prog_check_t = 0.0
        self._sys = {'cpu': 0.0, 'temp': None, 'mem': 0.0, 'disk': 0.0, 'uptime': 0}
        self._sys_t = 0.0
        self._cpu_prev = None
        self._nodes_t = 0.0
        self._nodes = []

        # ---- 上一輪的狀態，用來寫事件 ----
        self._prev_mode = None
        self._prev_state = None
        self._prev_skipped = 0
        self._prev_save = ''
        self._sent_mode = None                          # (mode, 單調時間, 誰) 網頁剛送的
        self._abort_logged = None

        # ---- ROS ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.mode_cli = self.create_client(SetDriveMode, 'change_mower_mode')
        self.preview_cli = self.create_client(Trigger, '/mower_manager/preview')
        self.save_cli = self.create_client(Trigger, '/map_autosave/save')
        self.lawn_cli = {k: self.create_client(Trigger, '/boundary_recorder/' + k) for k in LAWN_WHAT}
        self.nav_ac = ActionClient(self, FollowPath, 'follow_path')
        self._save_future = None
        # 存地圖中 (單調時間，沒在存是 None)。網頁按的、手把 Y 按的都算。
        # 存圖的時候 slam_toolbox 很忙，map -> odom 會斷一下；這時候 Nav2 在跑的話會把這一段判失敗
        # (2026-10-20 雲端實測：存圖途中開始自動割草，連三段 Controller patience exceeded，任務中止)。
        self.saving_since = None
        self.button_y = self._button_y()
        self._y_prev = False

        sub = self.create_subscription
        sub(MowerStatus, '/mower_status', self._on_mower, 10)
        sub(MissionStatus, '/mission_status', self._on_mission, 10)
        sub(JoyStatus, '/joy_status', self._on_joy_status, 10)
        sub(Joy, '/joy', self._on_joy, qos_profile_sensor_data)
        sub(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        sub(Odometry, '/odom', self._on_odom, 10)
        sub(Twist, '/cmd_vel', self._on_cmd_out, 10)
        sub(Twist, '/cmd_vel_nav_raw', self._on_cmd_nav_raw, 10)
        sub(Twist, '/cmd_vel_nav', self._on_cmd_nav, 10)
        sub(OccupancyGrid, '/map', self._on_map, LATCHED)
        sub(OccupancyGrid, '/keepout_mask', self._on_keepout, LATCHED)
        sub(Path, '/f2c_path', self._on_path, QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        sub(PolygonStamped, '/f2c_boundary', self._on_boundary, 10)
        sub(Bool, '/slam_scan_gate/active', self._on_mapping, LATCHED)
        sub(String, '/map_autosave/last', self._on_save, LATCHED)
        sub(Bool, '/map_autosave/busy', self._on_save_busy, LATCHED)
        sub(PolygonStamped, '/lawn_boundary', self._on_lawn, LATCHED)
        sub(Bool, '/boundary_recorder/recording', self._on_rec, LATCHED)
        sub(Path, '/boundary_recorder/track', self._on_track, 10)
        sub(PolygonStamped, '/polygon_stop', self._on_zone, qos_profile_sensor_data)

        self.vehicle = self._vehicle()
        self.alarms.on_raise = lambda p, text: self.log('系統', ('警報 %d：' % p) + text)
        self.create_timer(0.2, self.tick)
        self.get_logger().info('網頁人機介面啟動')

    # ------------------------------------------------------------------
    # 收資料
    # ------------------------------------------------------------------
    def _mono(self):
        return time.monotonic()

    def _tick_rate(self, name):
        t = self._mono()
        self.t[name] = t
        if name in self.rates:
            self.rates[name].tick(t)

    def _on_mower(self, msg):
        self.mower = msg
        self._tick_rate('/mower_status')

    def _on_joy(self, msg):
        self._tick_rate('/joy')
        # 手把 Y = 存地圖 + 預覽 (teleop 的 save_map)。teleop 在自動割草中不存，這裡照抄同一個條件
        y = len(msg.buttons) > self.button_y and msg.buttons[self.button_y] == 1
        if y and not self._y_prev and self.mower is not None and self.mower.mode not in (1, 3) and self.mapping:
            with self._lock:
                if self.saving_since is None:
                    self.saving_since = self._mono()
        self._y_prev = y

    def _button_y(self):
        """手把 Y 是第幾顆鍵：讀 teleop 用的同一份 joystick.yaml (車主可能改過)，讀不到就用 3。"""
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('mowerbot_bridge'), 'config', 'joystick.yaml')
            with open(path, encoding='utf-8') as f:
                m = re.search(r'^\s*button_y\s*:\s*(\d+)', f.read(), re.MULTILINE)
            return int(m.group(1)) if m else 3
        except Exception:
            return 3

    def _on_mission(self, msg):
        self.mission = msg
        self.t['/mission_status'] = self._mono()

    def _on_joy_status(self, msg):
        self.joy = msg
        self._tick_rate('/joy_status')

    def _on_scan(self, msg):
        self.scan = msg
        self._tick_rate('/scan')

    def _on_odom(self, msg):
        self.odom = msg
        self._tick_rate('/odom')

    def _on_cmd_out(self, msg):
        self.cmd_out = msg.linear.x
        self._tick_rate('/cmd_vel')

    def _on_cmd_nav_raw(self, msg):
        self.cmd_nav = msg.linear.x
        self.cmd_nav_raw_t = self._mono()
        self.t['/cmd_vel_nav'] = self._mono()

    def _on_cmd_nav(self, msg):
        # collision_monitor 開著的時候 Nav2 想要的速度在 /cmd_vel_nav_raw；沒開才看這裡
        if self.cmd_nav_raw_t is None or self._mono() - self.cmd_nav_raw_t > 1.0:
            self.cmd_nav = msg.linear.x
        self.t['/cmd_vel_nav'] = self._mono()

    def _on_map(self, msg):
        with self._lock:
            self.map_msg = msg
            self.map_dirty = True
        self.t['/map'] = self._mono()

    def _on_keepout(self, msg):
        with self._lock:
            self.keepout_msg = msg if msg.header.frame_id == 'map' else None
            self.map_dirty = True

    def _on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self.path_pts = decimate(pts, 0.05) if pts else None
            self.route_boundary = self.boundary_pts if pts else None
            self.route_ver += 1

    def _on_boundary(self, msg):
        pts = [[round(p.x, 3), round(p.y, 3)] for p in msg.polygon.points]
        with self._lock:
            if pts != self.boundary_pts:
                self.boundary_pts = pts or None

    def _on_mapping(self, msg):
        self.mapping = bool(msg.data)

    def _on_save(self, msg):
        self.last_save = msg.data
        with self._lock:
            self.saving_since = None

    def _on_save_busy(self, msg):
        # map_autosave 自己說在不在存 (手把 Y、網頁、預覽都算)；舊版沒有這個話題就只靠網頁自己記的
        with self._lock:
            if msg.data:
                if self.saving_since is None:
                    self.saving_since = self._mono()
            else:
                self.saving_since = None

    def _on_lawn(self, msg):
        pts = [[round(p.x, 3), round(p.y, 3)] for p in msg.polygon.points]
        with self._lock:
            self.lawn_pts = pts if len(pts) >= 3 else None
            self.lawn_ver += 1

    def _on_rec(self, msg):
        with self._lock:
            self.rec_on = bool(msg.data)
            self.rec_ver += 1

    def _on_track(self, msg):
        pts = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses]
        with self._lock:
            self.rec_pts = pts or None
            self.rec_len = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
            self.rec_ver += 1

    def _on_zone(self, msg):
        if msg.header.frame_id in ('base_footprint', 'base_link') and len(msg.polygon.points) >= 3:
            self.zone_poly = [(p.x, p.y) for p in msg.polygon.points]

    def _vehicle(self):
        v = {'front': 0.975, 'rear': 0.155, 'half': 0.42, 'bladeX': 0.375, 'bladeR': 0.25, 'lidarX': 0.30,
             'zoneLen': STOP_ZONE_LENGTH, 'zoneSide': STOP_ZONE_SIDE_MARGIN, 'zoneMax': STOP_ZONE_MAX_POINTS}
        try:
            from mowerbot_description import vehicle_geometry
            g = vehicle_geometry.load()
            v.update(front=g.front_extent, rear=g.rear_extent, half=g.lateral_half_extent,
                     bladeX=g.blade_offset_x, bladeR=g.blade_width / 2.0, lidarX=g.lidar_x)
        except Exception as exc:        # 只影響畫車子的大小
            self.get_logger().warn('讀不到 vehicle_geometry，用預設尺寸：%s' % exc)
        return v

    # ------------------------------------------------------------------
    # 事件紀錄
    # ------------------------------------------------------------------
    def log(self, src, text):
        with self._lock:
            self.events.appendleft({'t': now_ms(), 'src': src, 'text': text})
            self.events_ver += 1

    # ------------------------------------------------------------------
    # 每 0.2 秒：整理一份給網頁的狀態
    # ------------------------------------------------------------------
    def fresh(self, name, timeout):
        t = self.t.get(name)
        return t is not None and self._mono() - t <= timeout

    def lookup(self, target, source):
        try:
            return tf_matrix(self.tf_buffer.lookup_transform(target, source, Time()).transform)
        except Exception:               # TF 還沒有 / 斷了
            return None

    def tick(self):
        try:
            self._tick()
        except Exception as exc:        # 畫面資料出錯不能讓節點死掉
            self.get_logger().error('tick 失敗：%r' % exc)

    def _tick(self):
        mono = self._mono()
        with self._lock:
            if self.saving_since is not None and (mono - self.saving_since > SAVE_MAX or not self.prog['running']):
                self.saving_since = None
        if mono - self._prog_check_t >= 1.0:
            self._prog_check_t = mono
            self.check_program()
        if mono - self._sys_t >= 2.0:
            self._sys_t = mono
            self.read_sys()
        if mono - self._nodes_t >= 2.0:
            self._nodes_t = mono
            self.read_nodes()

        running = self.prog['running']
        manager_ok = running and self.mower is not None and self.fresh('/mower_status', MOWER_STATUS_TIMEOUT)
        mode = int(self.mower.mode) if manager_ok else None
        ms = self.mission if (manager_ok and self.mission is not None) else None
        state = int(ms.state) if ms else 0
        message = ms.message if ms else ''

        joy_ok = running and self.joy is not None and self.fresh('/joy_status', JOY_STATUS_TIMEOUT) and self.joy.connected
        latched = '按一下 LB' in message
        scan_ok = running and self.scan is not None and self.fresh('/scan', SCAN_TIMEOUT)

        # 車子在哪：有 map 用 map，還沒開始建圖就用 odom
        pose, frame = None, None
        for fr in ('map', 'odom'):
            m = self.lookup(fr, 'base_footprint') if running else None
            if m is not None:
                pose = {'x': round(m[2], 3), 'y': round(m[3], 3), 'yaw': round(math.atan2(m[1], m[0]), 4), 'frame': fr}
                frame = fr
                break

        # 光達點：base_footprint 算停止區、map (或 odom) 給畫面
        zone_n, scan_flat = 0, []
        if scan_ok:
            zone_n, scan_flat = self.process_scan(frame)
        zone_blocked = zone_n > STOP_ZONE_MAX_POINTS
        executing = mode == 1 and state == MissionStatus.STATE_EXECUTING

        # 割過的軌跡：自動割草執行中才記
        with self._lock:
            if self._prev_state is not None and state == MissionStatus.STATE_PLANNING and self._prev_state != state:
                self.trail = []
                self.trail_ver += 1
            if executing and pose and frame == 'map':
                last = self.trail[-1][-1] if self.trail and self.trail[-1] else None
                if last is None or math.hypot(pose['x'] - last[0], pose['y'] - last[1]) >= TRAIL_STEP:
                    if not self.trail:
                        self.trail.append([])
                    self.trail[-1].append([pose['x'], pose['y']])
                    if sum(len(s) for s in self.trail) > TRAIL_MAX:
                        self.trail[0] = self.trail[0][len(self.trail[0]) // 2:]
                    if mono - self.trail_sent_t >= 1.0:
                        self.trail_sent_t = mono
                        self.trail_ver += 1
            elif self.trail and self.trail[-1]:
                self.trail.append([])           # 停下來 (被擋、跳過) 就斷開，下一段另起一條
                self.trail_ver += 1

        block = self.block_reason(mode, state, message, joy_ok, latched, zone_blocked)
        self.update_alarms(running, manager_ok, mode, state, message, joy_ok, latched, scan_ok, zone_blocked, executing)
        self.update_events(mode, state, ms)
        if self.map_dirty:
            self.render_map()

        vx = self.odom.twist.twist.linear.x if (self.odom is not None and self.fresh('/odom', 1.0)) else 0.0
        wz = self.odom.twist.twist.angular.z if (self.odom is not None and self.fresh('/odom', 1.0)) else 0.0
        nav = self.cmd_nav if self.fresh('/cmd_vel_nav', 0.5) else 0.0
        out = self.cmd_out if self.fresh('/cmd_vel', 0.5) else 0.0
        save_name = None
        mm = re.search(r'已存 (\S+)', self.last_save or '')
        if mm:
            save_name = os.path.basename(mm.group(1))

        nav_ready = False
        if running:
            try:
                nav_ready = self.nav_ac.server_is_ready()
            except Exception:
                nav_ready = False

        with self._lock:
            lawn = {'recording': self.rec_on, 'set': bool(self.lawn_pts), 'length': None, 'gap': None,
                    'area': None, 'perimeter': None, 'close_gap': LAWN_CLOSE_GAP}
            if self.rec_on and self.rec_pts:
                lawn['length'] = round(self.rec_len, 1)
                if pose and frame == 'map':
                    x0, y0 = self.rec_pts[0]
                    lawn['gap'] = round(math.hypot(pose['x'] - x0, pose['y'] - y0), 2)
            if self.lawn_pts:
                area, perim = poly_area_perimeter(self.lawn_pts)
                lawn['area'], lawn['perimeter'] = round(area, 1), round(perim, 1)
            mission = {
                'state': state, 'total': int(ms.total_segments) if ms else 0,
                'completed': int(ms.completed_segments) if ms else 0, 'skipped': int(ms.skipped_segments) if ms else 0,
                'label': ms.current_label if ms else '', 'skipped_labels': list(ms.skipped_labels) if ms else [],
                'message': message, 'swath_deg': None, 'lines': None,
            }
            md = re.search(r'割草線 (\d+) 條 \(方向 (-?\d+)°\)', message)
            if md:
                mission['lines'], mission['swath_deg'] = int(md.group(1)), int(md.group(2))
            self.frame = {
                'stamp': now_ms(),
                'program': dict(self.prog),
                'manager': manager_ok,
                'mode': mode,
                'authority': {'holder': self.holder, 'label': self.holder_label},
                'mission': mission,
                'block': block,
                'joy': {'connected': bool(joy_ok), 'latched': latched},
                'lidar_hz': round(self.rates['/scan'].hz(mono), 2) if scan_ok else 0.0,
                'nav_ready': nav_ready,
                'zone': {'n': zone_n, 'blocked': zone_blocked},
                'pose': pose,
                'vel': {'v': round(vx, 3), 'w': round(wz, 3), 'nav': round(nav, 3), 'out': round(out, 3)},
                'map': dict(self.map_meta, ready=bool(running and self.mapping and self.map_meta.get('ver')), name=save_name),
                'saving': self.saving_since is not None,
                'lawn': lawn,
                'geo': {'scan': scan_flat},
                'vehicle': self.vehicle,
                'diag': {
                    'topics': [{'name': n, 'hz': round(self.rates[n].hz(mono), 2), 'min': lo} for n, lo in TOPICS],
                    'nodes': self._nodes,
                    'sys': dict(self._sys, clients=self.count_clients()),
                },
            }
            self.heavy = {
                'alarms': (self.alarms.snapshot(), self.alarms.ver),
                'events': (list(self.events), self.events_ver),
                'route': (self.path_pts, self.route_ver),
                'boundary': (self.shown_boundary(mode), self.boundary_ver),
                'trail': ([s for s in self.trail if len(s) > 1] or None, self.trail_ver),
                'lawn': (self.lawn_pts, self.lawn_ver),
                'rec': (self.rec_pts if self.rec_on else None, self.rec_ver),
            }
            self.release_stale_holder()

    def shown_boundary(self, mode):
        """總覽畫的割草範圍。呼叫時要拿著 self._lock。

        /f2c_boundary 跟著建圖一直更新。自動割草中地圖還在長大，直接畫最新的範圍會比這趟路線
        大很多，看起來像整片都會割。所以割草中畫路線算的那一刻的範圍，其他時候畫最新的
        (按 Y 或 LB + B 會用的就是最新的)。
        """
        shown = self.boundary_pts
        if mode in (1, 3) and self.route_boundary:
            shown = self.route_boundary
        if shown is not self._shown_boundary:
            self._shown_boundary = shown
            self.boundary_ver += 1
        return shown

    def process_scan(self, frame):
        msg = self.scan
        laser = msg.header.frame_id or 'radar'
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= msg.range_max)
        r, a = r[ok], a[ok]
        lx, ly = r * np.cos(a), r * np.sin(a)
        zone_n = 0
        to_base = self.lookup('base_footprint', laser)
        if to_base is not None and lx.size:
            bx, by = apply_tf(to_base, lx, ly)
            v = self.vehicle
            poly = self.zone_poly or [
                (v['front'] + STOP_ZONE_LENGTH, v['half'] + STOP_ZONE_SIDE_MARGIN),
                (v['front'] + STOP_ZONE_LENGTH, -v['half'] - STOP_ZONE_SIDE_MARGIN),
                (v['front'], -v['half'] - STOP_ZONE_SIDE_MARGIN),
                (v['front'], v['half'] + STOP_ZONE_SIDE_MARGIN)]
            zone_n = int(point_in_poly(bx, by, poly).sum())
        flat = []
        if frame is not None and lx.size:
            to_frame = self.lookup(frame, laser)
            if to_frame is not None:
                step = max(1, int(math.ceil(lx.size / SCAN_POINTS_MAX)))
                wx, wy = apply_tf(to_frame, lx[::step], ly[::step])
                flat = np.round(np.stack([wx, wy], axis=1).ravel(), 2).tolist()
        return zone_n, flat

    def block_reason(self, mode, state, message, joy_ok, latched, zone_blocked):
        """車子為什麼停著 (只在自動割草執行中有意義)。文字以 manager 的為準。"""
        if mode != 1 or state != MissionStatus.STATE_EXECUTING:
            return None
        first = message.split('\n', 1)[0]
        if first.startswith('暫停：'):
            text = first[len('暫停：'):].rstrip('。')
            if not joy_ok or '手把斷線' in text and not latched:
                return {'key': 'joy', 'p': 1, 'text': text, 'hint': '打開手把，連上之後按一下 LB 才會繼續走。'}
            if latched:
                return {'key': 'latch', 'p': 2, 'text': text, 'hint': '手把已經重新連上了。按一下 LB 車子才繼續走。'}
            if '/scan' in text or '光達' in text:
                return {'key': 'scan', 'p': 1, 'text': text, 'hint': '檢查光達的線和電源。'}
            return {'key': 'other', 'p': 2, 'text': text, 'hint': ''}
        if zone_blocked:
            return {'key': 'zone', 'p': 2, 'text': '前方停止區有東西',
                    'hint': '東西拿走就會接著走。停超過 15 秒這一段會跳過。'}
        return None

    def update_alarms(self, running, manager_ok, mode, state, message, joy_ok, latched, scan_ok, zone_blocked, executing):
        A = self.alarms
        mono = self._mono()
        grace = self.prog_seen_up is not None and mono - self.prog_seen_up < STARTUP_GRACE
        A.set('estop', mode == 4, 1, '停止中。到「操作」按手動，或手把 LB + X 解除')
        auto = mode in (1, 3)
        A.set('joy', running and not grace and not joy_ok, 1 if auto else 2,
              '手把斷線，車子停在原地' if auto else '手把斷線：RB 急停不能用，自動割草也不會出發')
        A.set('latch', executing and latched and joy_ok, 2, '手把重新連上了，按一下 LB 車子才繼續走')
        A.set('scan', running and not grace and self.prog.get('lidar', True) and not scan_ok, 1 if auto else 2,
              '光達沒有資料' + ('，車子停在原地' if auto else '：停止區沒有作用，自動割草也不會出發'))
        A.set('zone', executing and zone_blocked, 2, '前方停止區有東西，車子暫停')
        A.set('manager', running and not grace and not manager_ok, 1, 'mower_manager 沒有回應')
        temp = self._sys.get('temp')
        A.set('temp', temp is not None and temp >= 85.0, 2, 'N150 溫度 %.0f °C' % (temp or 0))
        A.set('disk', 0 < self._sys.get('disk', 99) < 3.0, 2, 'SSD 只剩 %.1f GB' % self._sys.get('disk', 0))
        if self.mission is not None and manager_ok:
            skipped = int(self.mission.skipped_segments)
            if skipped > self._prev_skipped and self.mission.skipped_labels:
                # 2026-10-20 起 manager 把原因 (對正超時、導航判失敗…) 寫在 skipped_labels 後面
                A.pulse('skip%d' % skipped, 3, '跳過 %s' % self.mission.skipped_labels[-1])
            self._prev_skipped = skipped
            if state == MissionStatus.STATE_ABORTED and self._abort_logged not in (None, state):
                # 進到「已中止」的那一刻報一次。message 開頭的「暫停：…」是 manager 另外加的 (自動模式中擋住的原因)，不是中止的原因
                reason = mission_text(message)
                A.pulse('abort', 2, '任務中止' + ('：' + reason if reason else '（原因看 car-log）'))
            self._abort_logged = state

    def update_events(self, mode, state, ms):
        if mode is not None and mode != self._prev_mode and self._prev_mode is not None:
            src = '手把'
            sent = self._sent_mode
            if sent and sent[0] == mode and self._mono() - sent[1] < 3.0:
                src = sent[2]
            self.log(src, '%s → %s' % (MODE_NAME.get(self._prev_mode, '?'), MODE_NAME.get(mode, '?')))
        self._prev_mode = mode
        if ms is not None and state != self._prev_state and self._prev_state is not None:
            text = '任務：' + STATE_NAME.get(state, '?')
            if mission_text(ms.message):
                text += '。' + mission_text(ms.message)
            self.log('系統', text)
        self._prev_state = state if ms is not None else self._prev_state
        if self.last_save and self.last_save != self._prev_save:
            # /map_autosave/last 是 latched：網頁比車上程式晚開的時候，一連上就會收到上一次的結果，那筆不算新的
            up = self.prog_seen_up
            if up is not None and self._mono() - up > 3.0:
                self.log('系統', '存地圖：' + self.last_save)
            self._prev_save = self.last_save

    # ------------------------------------------------------------------
    # 地圖 -> PNG
    # ------------------------------------------------------------------
    def render_map(self):
        with self._lock:
            msg, keep = self.map_msg, self.keepout_msg
            self.map_dirty = False
        if msg is None or msg.info.width == 0:
            return
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        x0, y0 = msg.info.origin.position.x, msg.info.origin.position.y
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        img = np.full((h, w), CELL_UNKNOWN, dtype=np.uint8)
        img[(grid >= 0) & (grid < OCC_THRESHOLD)] = CELL_FREE
        img[grid >= OCC_THRESHOLD] = CELL_OCC
        if keep is not None and keep.info.width > 1:
            kg = np.asarray(keep.data, dtype=np.int8).reshape(keep.info.height, keep.info.width)
            rows, cols = np.nonzero(kg > OCC_THRESHOLD)
            if rows.size:
                kr = keep.info.resolution
                wx = keep.info.origin.position.x + (cols + 0.5) * kr
                wy = keep.info.origin.position.y + (rows + 0.5) * kr
                mc = np.floor((wx - x0) / res).astype(int)
                mr = np.floor((wy - y0) / res).astype(int)
                ok = (mc >= 0) & (mc < w) & (mr >= 0) & (mr < h)
                img[mr[ok], mc[ok]] = CELL_KEEPOUT
        ok, png = cv2.imencode('.png', np.flipud(img))
        if not ok:
            return
        with self._lock:
            self.map_ver += 1
            self.map_png = png.tobytes()
            self.map_meta = {'ver': self.map_ver, 'res': res, 'x0': x0, 'y0': y0, 'w': w, 'h': h}

    # ------------------------------------------------------------------
    # 車上程式、N150 本身
    # ------------------------------------------------------------------
    def check_program(self):
        try:
            out = subprocess.run(['pgrep', '-u', str(os.getuid()), '-af', LAUNCH_PATTERN],
                                 capture_output=True, text=True, timeout=3).stdout
        except Exception:
            out = ''
        running = bool(out.strip())
        info = {}
        if running:
            try:
                rb = subprocess.run(['pgrep', '-u', str(os.getuid()), '-af', r'run_bringup\.sh'],
                                    capture_output=True, text=True, timeout=3).stdout
                m = RUN_BRINGUP_RE.search(rb)
                if m:
                    info['lidar'] = m.group(1) == 'true'
                    info['scale'] = float(m.group(2))
            except Exception:
                pass
            pid = out.split()[0]
            try:
                et = subprocess.run(['ps', '-o', 'etimes=', '-p', pid], capture_output=True, text=True, timeout=3).stdout
                info['since'] = now_ms() - int(et.strip()) * 1000
            except Exception:
                pass
        with self._lock:
            was = self.prog['running']
            self.prog['running'] = running
            self.prog.update(info)
            if running and not was:
                self.prog_seen_up = self._mono()
                self.expect_down = False
                scale = self.prog.get('scale')
                self.log('系統', '車上程式在跑了（光達%s，速度上限 %s%s）' % (
                    '開' if self.prog.get('lidar', True) else '關', scale,
                    '，最快約 %.2f m/s' % (0.7 * scale) if scale else ''))
            if was and not running:
                self.prog_seen_up = None
                self.mapping = False
                self.trail = []
                self.trail_ver += 1
                self.path_pts = None
                self.route_ver += 1
                self.boundary_pts = None
                self.route_boundary = None
                self.lawn_pts = None
                self.lawn_ver += 1
                self.rec_on = False
                self.rec_pts = None
                self.rec_ver += 1
                self.map_msg = None
                self.keepout_msg = None
                self.map_meta = {'ready': False, 'ver': 0}
                self.mission = None
                self.mower = None
                if self.expect_down:
                    self.log('系統', '車上程式關掉了')
                else:
                    self.alarms.pulse('bringup', 2, '車上程式關掉了（不是網頁關的：car-stop，或程式自己結束，看 ~/car_logs/latest.log）')
                self.expect_down = False

    def read_sys(self):
        s = self._sys
        try:
            with open('/proc/stat') as f:
                v = [int(x) for x in f.readline().split()[1:]]
            idle, total = v[3] + v[4], sum(v)
            if self._cpu_prev:
                di, dt = idle - self._cpu_prev[0], total - self._cpu_prev[1]
                s['cpu'] = round(100.0 * (1 - di / dt), 1) if dt > 0 else 0.0
            self._cpu_prev = (idle, total)
        except Exception:
            pass
        temps = []
        try:
            for z in os.listdir('/sys/class/thermal'):
                if z.startswith('thermal_zone'):
                    try:
                        with open('/sys/class/thermal/%s/temp' % z) as f:
                            t = int(f.read().strip()) / 1000.0
                        if 0 < t < 150:
                            temps.append(t)
                    except Exception:
                        pass
        except Exception:
            pass
        s['temp'] = round(max(temps), 1) if temps else None
        try:
            mem = {}
            with open('/proc/meminfo') as f:
                for line in f:
                    k, val = line.split(':', 1)
                    mem[k] = int(val.split()[0])
            s['mem'] = round(100.0 * (1 - mem['MemAvailable'] / mem['MemTotal']), 1)
        except Exception:
            pass
        try:
            s['disk'] = round(shutil.disk_usage(self.home).free / 1e9, 1)
        except Exception:
            pass
        try:
            with open('/proc/uptime') as f:
                s['uptime'] = int(float(f.read().split()[0]) * 1000)
        except Exception:
            pass

    def read_nodes(self):
        try:
            names = {('/' + n if not ns or ns == '/' else ns.rstrip('/') + '/' + n)
                     for n, ns in self.get_node_names_and_namespaces()}
        except Exception:
            names = set()
        up = self.prog['running']
        self._nodes = [{'name': n, 'what': what, 'ok': up and n in names} for n, what in NODES]

    # ------------------------------------------------------------------
    # 操作權
    # ------------------------------------------------------------------
    def client_seen(self, cid, label, streaming=None):
        with self._lock:
            c = self.web_clients.setdefault(cid, {'label': label, 'seen': 0.0, 'streams': 0})
            c['seen'] = self._mono()
            c['label'] = label or c['label']
            if streaming is not None:
                c['streams'] = max(0, c['streams'] + (1 if streaming else -1))
            if self.holder is None and streaming:
                # 沒有人拿著操作權：第一個打開網頁的裝置拿到
                self.holder, self.holder_label = cid, c['label']

    def count_clients(self):
        return sum(1 for c in self.web_clients.values() if c['streams'] > 0)

    def release_stale_holder(self):
        c = self.web_clients.get(self.holder)
        if self.holder is not None and (c is None or (c['streams'] == 0 and self._mono() - c['seen'] > 15.0)):
            self.holder, self.holder_label = None, ''
            for cid, cc in self.web_clients.items():
                if cc['streams'] > 0:
                    self.holder, self.holder_label = cid, cc['label']
                    break

    # ------------------------------------------------------------------
    # 指令 (HTTP 執行緒呼叫)
    # ------------------------------------------------------------------
    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=1.0):
            return None, '服務沒有開（車上程式還在啟動？）'
        fut = client.call_async(req)
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if not ev.wait(timeout):
            return None, '沒有回應'
        return fut.result(), ''

    def set_mode(self, mode, who):
        req = SetDriveMode.Request()
        req.mode = int(mode)
        with self._lock:
            self._sent_mode = (int(mode), self._mono(), who)
        res, err = self.call(self.mode_cli, req)
        if res is None:
            return False, err
        return bool(res.success), '' if res.success else 'manager 不接受'

    def command(self, cid, label, cmd, arg):
        who = '網頁 (%s)' % (label or '?')
        self.client_seen(cid, label)
        with self._lock:
            frame = self.frame
            holder = self.holder
        running = self.prog['running']
        mode = frame.get('mode')

        if cmd == 'stop':
            if not running:
                return False, '車上程式沒在跑'
            ok, err = self.set_mode(4, who)
            self.log(who, '停止鍵' + ('' if ok else '（沒送到：%s）' % err))
            return ok, err
        if cmd == 'ack':
            with self._lock:
                self.alarms.ack(int(arg))
            return True, ''
        if cmd == 'ackAll':
            with self._lock:
                self.alarms.ack(None)
            self.log(who, '確認全部警報')
            return True, ''
        if cmd == 'takeAuth':
            with self._lock:
                self.holder, self.holder_label = cid, label
            self.log(who, '操作權改到這台')
            return True, ''

        # 以下會讓車子動、或改變車上程式：只有拿著操作權的那一台
        if holder is not None and holder != cid:
            return False, '操作權在另一台（%s）。要用這台操作，先按「改由這台操作」' % self.holder_label
        if holder is None:
            with self._lock:
                self.holder, self.holder_label = cid, label

        if cmd == 'start':
            if running or self.prog['busy']:
                return False, '車上程式已經在跑了'
            self.run_script('starting', [self.car_start], who, '啟動車上程式')
            return True, '啟動中（N150 上大約 20～40 秒）'
        if cmd == 'shutdown':
            if not running or self.prog['busy']:
                return False, '車上程式沒在跑'
            self.expect_down = True
            self.run_script('stopping', [self.car_stop], who, '關閉車上程式')
            return True, '關閉中：先切停止，輪子停了才關'
        if not running:
            return False, '車上程式沒在跑'
        if not frame.get('manager'):
            return False, 'mower_manager 沒有回應'
        if cmd == 'mode':
            try:
                m = int(arg)
            except (TypeError, ValueError):
                return False, '模式不對'
            if m not in (0, 1, 2):
                return False, '網頁只能切手動、建圖、自動割草'
            if m == mode:
                # 自動割草再切一次 = 重新規劃 (manager 的行為，手把 LB + B 也一樣)；網頁不要因為按兩下就重來
                return False, '已經是%s了' % MODE_NAME[m]
            if mode == 4 and m != 2:
                return False, '停止中只能先切手動（跟手把 LB + X 一樣）'
            if m == 1 and not frame.get('map', {}).get('ready'):
                return False, '還沒有地圖：先開始建圖，把範圍開一圈'
            if m == 1 and not frame.get('joy', {}).get('connected'):
                return False, '手把沒連上：自動割草要手把連著（打開手把再按）'
            if m == 1 and self.rec_on:
                return False, '正在記錄草地邊界：先按「完成」或「取消這次」'
            if m == 1 and not frame.get('nav_ready'):
                return False, '導航 (Nav2) 還沒準備好：car-start planonly 開的，或 Nav2 沒起來（關閉再啟動一次）'
            ok, err = self.set_mode(m, who)
            if not ok:
                self.log(who, '切到%s（沒成功：%s）' % (MODE_NAME[m], err))
            elif m == 1 and self.saving_since is not None:
                # 存圖的時候車子在地圖上的位置會斷一下，manager 會等存好才出發 (2026-10-20)
                return ok, '正在存地圖，存好就出發（大約 10～30 秒）'
            return ok, err
        if cmd == 'preview':
            if mode in (1, 3):
                return False, '自動割草中不能預覽，先切手動'
            if self.rec_on:
                return False, '正在記錄草地邊界：先按「完成」，預覽才會用新的邊界'
            res, err = self.call(self.preview_cli, Trigger.Request())
            if res is None or not res.success:
                return False, err or res.message
            self.log(who, '預覽路線')
            err = self.save_async(who)
            return True, '算路線中，大約 1 秒後畫在總覽的地圖上；' + ('同時存地圖' if not err else '地圖沒存：' + err)
        if cmd == 'lawn':
            if arg not in LAWN_WHAT:
                return False, '不認得的指令'
            if mode in (1, 3) and arg in ('start', 'stop'):
                return False, '自動割草中不能記錄草地邊界，先切手動'
            if arg == 'start' and not self.mapping:
                return False, '還沒開始建圖：先「開始建圖」（手把 LB + A），邊界要記在地圖上'
            if arg == 'start' and self.rec_on:
                return False, '已經在記錄了'
            if arg == 'stop' and not self.rec_on:
                return False, '目前沒有在記錄'
            res, err = self.call(self.lawn_cli[arg], Trigger.Request())
            if res is None:
                return False, '草地邊界記錄器 (boundary_recorder) ' + err
            self.log(who, LAWN_WHAT[arg] + ('' if res.success else '（沒成功：%s）' % res.message))
            return bool(res.success), res.message
        if cmd == 'savemap':
            if mode in (1, 3):
                return False, '自動割草中不存地圖（存圖的時候路線會被判失敗）；先切手動'
            if not self.mapping:
                return False, '還沒開始建圖，沒有地圖可以存'
            err = self.save_async(who)
            if err:
                return False, err
            return True, '存地圖中（大約 10～30 秒），存好會寫在「紀錄」'
        return False, '不認得的指令'

    def save_async(self, who):
        """存地圖 (跟手把 Y 一樣叫 map_autosave)。存一次 10～30 秒，結果由 /map_autosave/last 寫進紀錄。"""
        if self.saving_since is not None or (self._save_future is not None and not self._save_future.done()):
            return '上一次還在存，等它存完'
        if not self.save_cli.service_is_ready():
            self.log('系統', '存地圖：存圖服務 (map_autosave) 沒有開')
            return '存圖服務 (map_autosave) 沒有開'
        with self._lock:
            self.saving_since = self._mono()
        self._save_future = self.save_cli.call_async(Trigger.Request())
        self.log(who, '存地圖')
        return ''

    def run_script(self, busy, argv, who, what):
        with self._lock:
            self.prog['busy'] = busy
            self.prog['output'] = ''
        self.log(who, what)

        def work():
            # 輸出寫到檔案，不用管線：car-start 開的 tmux 會留在背景，用管線的話要等它結束才讀得完
            log_dir = os.path.join(self.home, 'car_logs')
            log_path = os.path.join(log_dir, 'hmi_%s.log' % os.path.basename(argv[0]))
            try:
                os.makedirs(log_dir, exist_ok=True)
                with open(log_path, 'w+', encoding='utf-8', errors='replace') as out:
                    try:
                        rc = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=out, stderr=subprocess.STDOUT,
                                            cwd=self.home, timeout=150).returncode
                        out.seek(0)
                        text = out.read().strip()
                    except subprocess.TimeoutExpired:
                        out.seek(0)
                        text, rc = out.read().strip() + '\n超過 150 秒還沒結束', -1
            except Exception as exc:
                text, rc = '跑不起來：%s' % exc, -1
            lines = text.splitlines()
            fail = [ln for ln in lines if '❌' in ln or '⚠' in ln]
            with self._lock:
                self.prog['busy'] = None
                # 失敗留最後幾行 (car-start 的 ❌ 原因在這裡)；成功但有 ❌ / ⚠ 的行 (例如光達停止區沒起來) 只留那幾行
                self.prog['output'] = '\n'.join(lines[-12:]) if rc != 0 else '\n'.join(ln.strip() for ln in fail)
            if rc != 0:
                result = '失敗（%s）' % (fail[-1].strip() if fail else '回傳 %d' % rc)
            elif fail:
                result = '完成，但是：' + '；'.join(ln.strip() for ln in fail)
            else:
                result = '完成'
            self.log('系統', '%s：%s' % (what, result))
            self._prog_check_t = 0.0
        threading.Thread(target=work, daemon=True).start()


# ======================================================================
# HTTP
# ======================================================================
def make_handler(node, page, allow):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'mowerbot-hmi'
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):       # 不要每個請求都印
            pass

        def allowed(self):
            try:
                ip = ipaddress.ip_address(self.client_address[0])
            except ValueError:
                return False
            if ip.version == 6 and ip.ipv4_mapped:
                ip = ip.ipv4_mapped
            return any(ip in net for net in allow)

        def send_bytes(self, code, body, ctype, extra=None):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, obj, code=200):
            self.send_bytes(code, json.dumps(obj, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8')

        def do_GET(self):
            if not self.allowed():
                return self.send_bytes(403, '只接受 mowerbot-n150 熱點上的裝置'.encode('utf-8'), 'text/plain; charset=utf-8')
            u = urlparse(self.path)
            if u.path in ('/', '/index.html'):
                return self.send_bytes(200, page, 'text/html; charset=utf-8')
            if u.path == '/api/stream':
                return self.stream(parse_qs(u.query))
            if u.path == '/api/map.png':
                with node._lock:
                    png = node.map_png
                if png is None:
                    return self.send_bytes(404, b'no map', 'text/plain')
                return self.send_bytes(200, png, 'image/png')
            if u.path == '/api/state':
                with node._lock:
                    f = dict(node.frame)
                    for k, (v, _ver) in node.heavy.items():
                        if k in ('alarms', 'events'):
                            f[k] = v
                        else:
                            f.setdefault('geo', {})[k] = v
                return self.send_json(f)
            return self.send_bytes(404, b'not found', 'text/plain')

        def do_POST(self):
            if not self.allowed():
                return self.send_bytes(403, b'forbidden', 'text/plain')
            if urlparse(self.path).path != '/api/cmd':
                return self.send_bytes(404, b'not found', 'text/plain')
            # 只收這個網頁自己送的指令：別的網站 (手機同時開著行動網路) 想偷送的話，
            # 瀏覽器會帶別的 Origin；而 application/json 會先問 OPTIONS，這裡不回，瀏覽器就不送。
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host', ''):
                return self.send_json({'ok': False, 'text': '只接受從車子網頁送出的指令'}, 403)
            if not self.headers.get('Content-Type', '').startswith('application/json'):
                return self.send_json({'ok': False, 'text': '看不懂的請求'}, 415)
            try:
                n = int(self.headers.get('Content-Length', '0'))
                body = json.loads(self.rfile.read(min(n, 65536)).decode('utf-8') or '{}')
            except Exception:
                return self.send_json({'ok': False, 'text': '看不懂的請求'}, 400)
            cid = str(body.get('client', ''))[:40]
            if not cid:
                return self.send_json({'ok': False, 'text': '看不懂的請求（沒有裝置代號）'}, 400)
            try:
                ok, text = node.command(cid, str(body.get('label', ''))[:40], str(body.get('cmd', '')), body.get('arg'))
            except Exception as exc:
                node.get_logger().error('指令 %r 出錯：%r' % (body, exc))
                ok, text = False, '出錯了：%s' % exc
            return self.send_json({'ok': bool(ok), 'text': text or ''})

        def stream(self, q):
            cid = (q.get('client') or [''])[0][:40]
            label = (q.get('label') or [''])[0][:40]
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            # 沒有裝置代號 (不是這個網頁開的，例如 curl) 只能看，不算進操作權
            streaming = True if cid else None
            node.client_seen(cid or '-', label, streaming=streaming)
            sent = {}
            try:
                self.wfile.write(b'retry: 2000\n\n')
                while rclpy.ok():
                    with node._lock:
                        f = dict(node.frame)
                        heavy = dict(node.heavy)
                    if not f:
                        time.sleep(0.2)
                        continue
                    geo = dict(f.get('geo', {}))
                    for k, (val, ver) in heavy.items():
                        if sent.get(k) != ver:
                            sent[k] = ver
                            if k in ('alarms', 'events'):
                                f[k] = val
                            else:
                                geo[k] = val
                    f['geo'] = geo
                    data = json.dumps(f, ensure_ascii=False, separators=(',', ':'))
                    self.wfile.write(('data: ' + data + '\n\n').encode('utf-8'))
                    self.wfile.flush()
                    node.client_seen(cid or '-', label)
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            finally:
                if cid:
                    node.client_seen(cid, label, streaming=False)
            self.close_connection = True

    return Handler


def load_page():
    """share/mowerbot_hmi/web/index.html 是片段 (<title>、<style>、畫面、<script>)，這裡包成完整的網頁。

    同一份片段也直接當 claude.ai 上的樣品頁 (沒有 window.MOWER_LIVE 就用示範資料)，所以不寫 <html> 那些。
    """
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('mowerbot_hmi'), 'web', 'index.html')
    with open(path, encoding='utf-8') as f:
        frag = f.read()
    # 片段開頭到第一個 </style> 是 <head> 的東西 (標題與樣式)，其餘是畫面
    cut = frag.find('</style>')
    head_part, body = (frag[:cut + len('</style>')], frag[cut + len('</style>'):]) if cut >= 0 else ('', frag)
    head = ('<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">'
            '<meta name="color-scheme" content="light dark">'
            '<style>html{padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px);'
            'box-sizing:border-box}body{margin:0}</style>'
            '<script>window.MOWER_LIVE=true;</script>' + head_part + '</head><body>')
    return (head + body + '</body></html>').encode('utf-8')


def main(argv=None):
    ap = argparse.ArgumentParser(description='mowerbot 網頁人機介面')
    ap.add_argument('--port', type=int, default=int(os.environ.get('MOWER_HMI_PORT', '8080')))
    ap.add_argument('--bind', default=os.environ.get('MOWER_HMI_BIND', '0.0.0.0'))
    ap.add_argument('--allow', default=os.environ.get('MOWER_HMI_ALLOW', '10.42.0.0/24,127.0.0.0/8'),
                    help='接受哪些網段的連線，逗號分開')
    ap.add_argument('--car-start', default=os.environ.get('MOWER_CAR_START', '/usr/local/bin/car-start'))
    ap.add_argument('--car-stop', default=os.environ.get('MOWER_CAR_STOP', '/usr/local/bin/car-stop'))
    args, ros_args = ap.parse_known_args(argv)
    allow = [ipaddress.ip_network(a.strip(), strict=False) for a in args.allow.split(',') if a.strip()]

    rclpy.init(args=ros_args)
    node = HmiWeb(args.car_start, args.car_stop, os.path.expanduser('~'))
    page = load_page()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros():
        try:
            executor.spin()
        except ExternalShutdownException:      # 關程式時 rclpy.try_shutdown() 造成的，正常
            pass
    spin = threading.Thread(target=spin_ros, daemon=True)
    spin.start()

    ThreadingHTTPServer.allow_reuse_address = True
    ThreadingHTTPServer.daemon_threads = True
    httpd = ThreadingHTTPServer((args.bind, args.port), make_handler(node, page, allow))
    node.get_logger().info('網頁：http://10.42.0.1:%d （接受 %s）' % (args.port, args.allow))

    def stop(*_a):
        threading.Thread(target=httpd.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        httpd.serve_forever(poll_interval=0.5)
    finally:
        httpd.server_close()
        # 先關 ROS 的 context：spin 的執行緒和 /api/stream 的迴圈 (while rclpy.ok()) 都會自己結束，
        # 再收掉節點。順序反過來的話，節點在 spin 途中被拆掉，程式結束時會 abort (systemd 記成 core dump)。
        rclpy.try_shutdown()
        spin.join(timeout=5.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
