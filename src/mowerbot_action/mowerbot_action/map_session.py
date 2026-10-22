#!/usr/bin/env python3
"""開機用存好的地圖，不用每次重新建圖 (2026-10-21 will)。

以前每次開機 slam_toolbox 都從頭建一張新圖，map 座標系的原點就是這次開機時車子停的位置，
所以禁區、草地外框每次都要重畫 (keepout_server、boundary_recorder 開頭的說明)。
現在 car-start 可以選「用存好的地圖」：slam_toolbox 改用定位模式讀 save_map.sh 存的 .posegraph，
map 座標系就是存圖那一次的座標系，那一次畫的禁區、圈的外框都還對得上。

【session】一次建圖 = 一個 session (keepout_server 以前每次開機產生的那個 id，save_map.sh 存在 <地圖>.session)。
  * 建新地圖：session = 這次開機新產生的
  * 用存好的地圖：session = 那張地圖的 .session (舊的地圖沒有 .session 就用 map_<檔名>)
禁區存在 ~/.ros/mowerbot_keepout/sessions/<session>.yaml，外框存在 ~/.ros/mowerbot_lawn/<session>.yaml，
開機時同一個 session 的就自動載回來。session 不同 (不同次建的圖) 絕對不載：座標系不一樣。

【車子在地圖上的哪裡】定位模式要先知道車子大概在地圖上的哪裡 (slam_toolbox 只在附近 ±0.25 m、±20° 找)：
  1. car-start 有指定 (map_start_pose) 就用它
  2. 上次用這張地圖時，關機前記下的位置 (這個節點每秒記一次，每張地圖各記一份，
     ~/.ros/mowerbot_last_pose/<session>.yaml；~/.ros/mowerbot_last_pose.yaml 是最後一次的)
  3. 都沒有就當作在建圖的起點 (0, 0, 0)
開機之後這個節點再拿光達跟地圖比對 (map_match)，在猜的位置附近 ±1.5 m、±30° 找最對得上的位置，
差得多就送 /initialpose 叫 slam_toolbox 換到那裡。還是對不上 (換了場地、車子被搬到很遠) 就停在「對不上」，
網頁上點出車子在哪、朝哪 (~/locate_request) 再找一次。對上之前 mower_manager 不讓自動割草出發。

發布：
    ~/info   (std_msgs/String JSON，latched)
             {mode: mapping | localization, session, saved_map, start}
    ~/match  (std_msgs/String JSON，latched，每秒) {mode, state, score, n, ok, text}
             state：mapping (建新地圖，不用比對) / waiting (等地圖、光達) / locating / ok / lost
             score = 光達的點落在地圖牆邊 10 cm 內的比例；ok = state 是 ok 而且最近 3 秒最好的 score 夠高
    /initialpose (PoseWithCovarianceStamped) 給 slam_toolbox
訂閱：
    ~/locate_request (PoseWithCovarianceStamped，map 座標)：使用者猜的位置，在它附近找
服務：
    ~/locate (std_srvs/Trigger)：在現在的位置附近重新找一次
"""
import glob
import json
import math
import os
import time
import uuid
from collections import deque

import yaml

KEEPOUT_SESSIONS_DIR = os.path.expanduser('~/.ros/mowerbot_keepout/sessions')
LAWN_DIR = os.path.expanduser('~/.ros/mowerbot_lawn')
LAST_POSE_FILE = os.path.expanduser('~/.ros/mowerbot_last_pose.yaml')
OK_SCORE = 0.5          # 光達的點至少一半落在地圖的牆邊才算對上 (室內實測對上時 0.8 ~ 0.95)
OK_POINTS = 30          # 而且至少要有這麼多點 (四周都是空地時沒辦法比)
MOVED_M = 0.05          # 找到的位置跟現在差這麼多以上才叫 slam_toolbox 換
MOVED_RAD = math.radians(1.5)


# ---- 給 launch 與網頁用的純函式 (不依賴 ROS) ------------------------------------
def new_session_id():
    return time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]


def session_of(saved_map):
    """存好的地圖 (路徑，不含副檔名) 是哪一次建圖存的。"""
    try:
        with open(saved_map + '.session', encoding='utf-8') as fh:
            sid = fh.read().strip()
        if sid:
            return sid
    except OSError:
        pass
    return 'map_' + os.path.basename(saved_map)


def check_saved_map(saved_map):
    """定位模式要的兩個檔都在才行。回傳缺的副檔名 list。"""
    return [e for e in ('posegraph', 'data') if not os.path.isfile('%s.%s' % (saved_map, e))]


def parse_start_pose(text):
    """car-start 指定的起點 "x,y,yaw度" -> (x, y, yaw 弧度)。空字串 -> None。"""
    t = text.strip()
    if not t:
        return None
    parts = [p for p in t.replace(' ', ',').split(',') if p]
    if len(parts) != 3:
        raise ValueError('map_start_pose 要寫成 x,y,朝向(度)，收到 %r' % text)
    x, y, deg = (float(p) for p in parts)
    if not all(math.isfinite(v) for v in (x, y, deg)):
        raise ValueError('map_start_pose 不是數字：%r' % text)
    return x, y, math.radians(deg)


def read_last_pose(path=LAST_POSE_FILE):
    try:
        with open(path, encoding='utf-8') as fh:
            d = yaml.safe_load(fh)
        return {'session': str(d['session']), 'x': float(d['x']), 'y': float(d['y']),
                'yaw': float(d['yaw']), 'stamp': float(d.get('stamp', 0.0))}
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        return None


def write_last_pose(path, session, x, y, yaw, mode):
    data = {'session': session, 'x': round(float(x), 4), 'y': round(float(y), 4),
            'yaw': round(float(yaw), 5), 'stamp': round(time.time(), 1), 'mode': mode}
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, sort_keys=False)
    os.replace(tmp, path)


def session_pose_file(session, last_pose_file=LAST_POSE_FILE):
    """每張地圖 (session) 各自的「上次的位置」：中間建過別張圖，回來用這張時還找得到。"""
    return os.path.join(os.path.splitext(last_pose_file)[0], session + '.yaml')


def start_pose(session, explicit_text='', last_pose_file=LAST_POSE_FILE):
    """定位模式的起點：(x, y, yaw, 說明)。順序見開頭的說明。"""
    p = parse_start_pose(explicit_text)
    if p is not None:
        return p + ('car-start 指定的位置',)
    for path in (session_pose_file(session, last_pose_file), last_pose_file):
        last = read_last_pose(path)
        if last is not None and last['session'] == session:
            when = (time.strftime('%m-%d %H:%M', time.localtime(last['stamp']))
                    if last['stamp'] else '?')
            return last['x'], last['y'], last['yaw'], '上次關機前的位置 (%s 記的)' % when
    return 0.0, 0.0, 0.0, '建圖的起點 (沒有記到上次的位置)'


def default_maps_dir():
    """save_map.sh 存圖的地方：原始碼的 src/mowerbot_bringup/maps，推不出來就用 share 裡的 maps。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('mowerbot_bringup')
    except Exception:          # 套件找不到 (單元測試)
        return ''
    src = os.path.normpath(
        os.path.join(share, '..', '..', '..', '..', 'src', 'mowerbot_bringup', 'maps'))
    return src if os.path.isdir(src) else os.path.join(share, 'maps')


def keepout_file(session):
    return os.path.join(KEEPOUT_SESSIONS_DIR, session + '.yaml')


def lawn_file(session):
    return os.path.join(LAWN_DIR, session + '.yaml')


def list_saved_maps(maps_dir, limit=12):
    """存好的地圖，新的在前面：[{name, path, mtime, session, keepout, lawn}]。要有 .posegraph 和 .data。"""
    out = []
    if not maps_dir or not os.path.isdir(maps_dir):
        return out
    for pg in glob.glob(os.path.join(maps_dir, '*.posegraph')):
        path = pg[:-len('.posegraph')]
        if check_saved_map(path):
            continue
        sid = session_of(path)
        out.append({'name': os.path.basename(path), 'path': path, 'mtime': os.path.getmtime(pg),
                    'session': sid, 'keepout': os.path.isfile(keepout_file(sid)),
                    'lawn': os.path.isfile(lawn_file(sid))})
    out.sort(key=lambda m: m['mtime'], reverse=True)
    return out[:limit]


# ---- 節點 -------------------------------------------------------------------
def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def main(args=None):
    import numpy as np
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import tf2_ros

    from mowerbot_interfaces.msg import MowerStatus
    from mowerbot_action import map_match

    class MapSession(Node):

        def __init__(self):
            super().__init__('map_session')
            self.declare_parameter('session_id', '')
            self.declare_parameter('saved_map', '')
            self.declare_parameter('start_source', '')
            self.declare_parameter('last_pose_file', LAST_POSE_FILE)
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('robot_frame', 'base_footprint')
            self.declare_parameter('ok_score', OK_SCORE)
            self.session = self.get_parameter('session_id').value or new_session_id()
            self.saved_map = self.get_parameter('saved_map').value
            self.mode = 'localization' if self.saved_map else 'mapping'
            self.last_pose_file = self.get_parameter('last_pose_file').value
            self.gf = self.get_parameter('global_frame').value
            self.rf = self.get_parameter('robot_frame').value
            self.ok_score = float(self.get_parameter('ok_score').value)

            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.info_pub = self.create_publisher(String, '~/info', latched)
            self.match_pub = self.create_publisher(String, '~/match', latched)
            self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.info_pub.publish(String(data=json.dumps({
                'mode': self.mode, 'session': self.session, 'saved_map': self.saved_map,
                'name': os.path.basename(self.saved_map) if self.saved_map else '',
                'start': self.get_parameter('start_source').value}, ensure_ascii=False)))

            self.tf = tf2_ros.Buffer()
            self.tfl = tf2_ros.TransformListener(self.tf, self)
            self.field = None
            self.scans = deque(maxlen=5)
            self.base_from_laser = {}        # 光達座標系 -> 車子座標系 (固定的，查一次)
            self.moving = True
            self._was_moving = False
            self.mower_mode = None
            self.state = 'mapping' if self.mode == 'mapping' else 'waiting'
            self.text = ''
            self.scores = deque(maxlen=3)
            self.last_written = None
            self._tf_since = None
            self._check_at = None
            self._attempts = 0
            self._pending_guess = None

            if self.mode == 'localization':
                self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched)
                self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
                self.create_subscription(PoseWithCovarianceStamped, '~/locate_request',
                                         self.request_cb, 10)
                self.create_service(Trigger, '~/locate', self.locate_srv)
            self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
            self.create_subscription(MowerStatus, '/mower_status', self.status_cb, 10)
            self.create_timer(1.0, self.tick)
            if self.mode == 'mapping':
                self.get_logger().info('🗺️ 建新地圖 (session %s)：禁區、外框會跟著這張圖存起來。' % self.session)
            else:
                self.get_logger().info('🗺️ 用存好的地圖 %s (session %s)，起點：%s' % (
                    self.saved_map, self.session, self.get_parameter('start_source').value))

        # ---- 訂閱 ----
        def map_cb(self, m):
            if m.info.width == 0 or m.info.height == 0:
                return
            grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
            self.field = map_match.MapField(grid, m.info.resolution, m.info.origin.position.x,
                                            m.info.origin.position.y)

        def scan_cb(self, m):
            self.scans.append(m)

        def odom_cb(self, m):
            t = m.twist.twist
            self.moving = abs(t.linear.x) > 0.02 or abs(t.angular.z) > 0.03

        def status_cb(self, m):
            self.mower_mode = m.mode

        # ---- TF ----
        def robot_pose(self, stamp=None):
            try:
                when = Time() if stamp is None else Time.from_msg(stamp)
                tr = self.tf.lookup_transform(self.gf, self.rf, when)
            except Exception:       # tf2 的各種例外 (還沒有 map、時間超出)
                return None
            t = tr.transform.translation
            return t.x, t.y, _yaw_of(tr.transform.rotation)

        def scan_in_base(self, scan):
            f = scan.header.frame_id
            if f not in self.base_from_laser:
                try:
                    tr = self.tf.lookup_transform(self.rf, f, Time())
                except Exception:
                    return None
                t = tr.transform.translation
                self.base_from_laser[f] = (t.x, t.y, _yaw_of(tr.transform.rotation))
            pts = map_match.scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                        scan.range_min, scan.range_max)
            return map_match.transform(pts, *self.base_from_laser[f])

        def latest_scan_and_pose(self):
            """最新一筆查得到當時位置的光達 (光達時間點的位置，車子在動也對得上)。"""
            for scan in reversed(self.scans):
                pose = self.robot_pose(scan.header.stamp)
                if pose is None:
                    continue
                pts = self.scan_in_base(scan)
                if pts is not None:
                    return pts, pose
            return None, None

        # ---- 每秒 ----
        def tick(self):
            pose = self.robot_pose()
            if self.mode == 'localization':
                self.localization_tick(pose is not None)
            self.record_pose(pose)
            self.publish_match()

        def localization_tick(self, have_tf):
            stopped_now = self._was_moving and not self.moving
            self._was_moving = self.moving
            if self.field is None or not self.scans or not have_tf:
                self.state = 'waiting'
                self.text = '等地圖、光達' if have_tf else '等定位 (slam_toolbox) 開始'
                self._tf_since = None
                return
            if self._tf_since is None:
                self._tf_since = time.monotonic()
            pts, pose = self.latest_scan_and_pose()
            if pts is None:
                return
            frac, n = map_match.score(self.field, map_match.transform(pts, *pose))
            self.scores.append((frac, n))
            good = frac >= self.ok_score and n >= OK_POINTS
            if self._pending_guess is not None:
                guess, self._pending_guess = self._pending_guess, None
                self.try_locate(pts, guess, '你點的位置', wide=True)
            elif self.state == 'waiting':
                if self.moving:
                    self.text = '車子在動，停下來才開始找位置'
                elif time.monotonic() - self._tf_since >= 2.0:
                    # slam_toolbox 已經在猜的起點上對過第一筆光達了：在它附近再找一次
                    self.try_locate(pts, pose, '開機的起點')
            elif (self.state == 'locating' and self._check_at is not None
                    and time.monotonic() >= self._check_at):
                self._check_at = None
                if good:
                    self.set_state('ok', '對上了：光達 %d%% 落在地圖的牆上' % round(100 * frac))
                elif self._attempts < 2:
                    self.try_locate(pts, pose, '再找一次')
                else:
                    self.set_state('lost', '還是對不上 (光達 %d%% 落在牆上)：在網頁上點出車子在哪、朝哪'
                                   % round(100 * frac))
            elif self.state == 'lost':
                if len(self.scores) == self.scores.maxlen and min(self.scores)[0] >= self.ok_score:
                    # 開到看得見地圖上的牆的地方，自己對上了
                    self.set_state('ok', '對上了：光達 %d%% 落在地圖的牆上' % round(100 * frac))
                elif stopped_now:
                    # 對不上的時候每次車子停下來就在附近再找一次 (換個地方看到的牆可能比較多)
                    self._attempts = 0
                    self.try_locate(pts, pose, '停下來的位置')
            elif self.state == 'ok' and good:
                self.text = '對上了：光達 %d%% 落在地圖的牆上' % round(100 * frac)

        def try_locate(self, pts, guess, what, wide=False):
            if self.moving:
                self.set_state(self.state, '車子在動，停下來才能找位置 (%s)' % what)
                return
            self._attempts += 1
            t0 = time.monotonic()
            r = map_match.locate(self.field, pts, guess,
                                 xy_half=1.5, yaw_half=math.radians(45.0 if wide else 30.0))
            self.get_logger().info(
                '📍 找位置 (%s)：(%.2f, %.2f, %.0f°) -> (%.2f, %.2f, %.0f°)，'
                '光達 %d%% 對上 (%d 點，%.1f s)' % (
                    what, guess[0], guess[1], math.degrees(guess[2]),
                    r['x'], r['y'], math.degrees(r['yaw']),
                    round(100 * r['score']), r['n'], time.monotonic() - t0))
            if r['n'] < OK_POINTS or r['score'] < self.ok_score:
                self.set_state('lost', '對不上地圖 (最好的位置也只有 %d%% 光達落在牆上)：'
                               '車子可能不在%s附近，或換了場地。在網頁上點出車子在哪、朝哪；換了場地就重新開始、選建新地圖'
                               % (round(100 * r['score']), what))
                return
            near_now = self.robot_pose()
            if (near_now is not None
                    and math.hypot(r['x'] - near_now[0], r['y'] - near_now[1]) < MOVED_M
                    and _angle_diff(r['yaw'], near_now[2]) < MOVED_RAD):
                self.set_state('ok', '對上了：光達 %d%% 落在地圖的牆上' % round(100 * r['score']))
                return
            self.send_initialpose(r['x'], r['y'], r['yaw'])
            self._check_at = time.monotonic() + 3.0
            self.scores.clear()
            self.set_state('locating', '找到了 (%d%% 對上)，換到那個位置' % round(100 * r['score']))

        def send_initialpose(self, x, y, yaw):
            m = PoseWithCovarianceStamped()
            m.header.frame_id = self.gf
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x = float(x)
            m.pose.pose.position.y = float(y)
            m.pose.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.pose.orientation.w = math.cos(yaw / 2.0)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
            m.pose.covariance[35] = math.radians(2.0) ** 2
            self.init_pub.publish(m)
            self.get_logger().info('📍 /initialpose -> (%.2f, %.2f, %.1f°)'
                                   % (x, y, math.degrees(yaw)))

        def set_state(self, state, text):
            if (state, text) != (self.state, self.text):
                # warn 和 info 要分兩行寫：rclpy 同一行呼叫換嚴重度會丟 ValueError，整個節點就掛了
                # (10-20 模擬：lost 之後使用者點位置 -> locating 那一下 map_session 當掉)
                if state == 'lost':
                    self.get_logger().warn('📍 %s：%s' % (state, text))
                else:
                    self.get_logger().info('📍 %s：%s' % (state, text))
            self.state, self.text = state, text

        def can_locate(self):
            if self.mode != 'localization':
                return '建新地圖的時候不用找位置'
            if self.mower_mode in (1, 3):
                return '自動割草中不能換位置，先 LB + X 回手動'
            if self.moving:
                return '車子在動，停下來才能找位置'
            if self.field is None or not self.scans:
                return '還沒有地圖或光達'
            return None

        def request_cb(self, m):
            why = self.can_locate()
            if why is not None:
                self.set_state(self.state, why)
                return
            q = m.pose.pose
            self._attempts = 0
            self._pending_guess = (q.position.x, q.position.y, _yaw_of(q.orientation))
            self.set_state('locating', '在你點的位置附近找')

        def locate_srv(self, _req, resp):
            why = self.can_locate()
            pose = self.robot_pose()
            if why is None and pose is None:
                why = '還不知道車子在哪 (沒有 map -> base_footprint)'
            if why is not None:
                resp.success, resp.message = False, why
                return resp
            self._attempts = 0
            self._pending_guess = pose
            self.set_state('locating', '在現在的位置附近重新找')
            resp.success, resp.message = True, '開始找'
            return resp

        def best_score(self):
            if not self.scores:
                return 0.0, 0
            return max(self.scores)

        def publish_match(self):
            if self.mode == 'mapping':
                d = {'mode': 'mapping', 'state': 'mapping', 'ok': True, 'score': None, 'n': None,
                     'text': '建新地圖'}
            else:
                frac, n = self.best_score()
                ok = self.state == 'ok' and frac >= self.ok_score and n >= OK_POINTS
                text = self.text
                if self.state == 'ok' and not ok:
                    text = '光達只有 %d%% 落在地圖的牆上 (要 %d%%)：有人或東西擋住光達？' % (
                        round(100 * frac), round(100 * self.ok_score))
                d = {'mode': 'localization', 'state': self.state, 'ok': ok,
                     'score': round(frac, 3), 'n': n, 'text': text}
            d['t'] = round(time.time(), 1)
            self.match_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

        def record_pose(self, pose):
            """車子的位置每秒記一次 (動了才寫)：下次用這張地圖開機就從這裡開始找。
            定位模式只在對上的時候記，對不上的位置記了下次只會更糟。"""
            if pose is None or (self.mode == 'localization' and self.state != 'ok'):
                return
            if self.last_written is not None:
                lx, ly, lyaw = self.last_written
                if (math.hypot(pose[0] - lx, pose[1] - ly) < 0.05
                        and _angle_diff(pose[2], lyaw) < math.radians(3.0)):
                    return
            try:
                for path in (self.last_pose_file,
                             session_pose_file(self.session, self.last_pose_file)):
                    write_last_pose(path, self.session, pose[0], pose[1], pose[2], self.mode)
                self.last_written = pose
            except OSError as exc:
                self.get_logger().warn('記不了車子的位置 (%s)：%s' % (self.last_pose_file, exc),
                                       throttle_duration_sec=60.0)

    rclpy.init(args=args)
    node = MapSession()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
