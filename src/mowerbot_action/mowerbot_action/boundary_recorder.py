#!/usr/bin/env python3
"""用手把沿草地邊緣開一圈，把軌跡記成割草範圍 (2026-10-08)。

為什麼需要、怎麼跟 map_to_boundary 接起來，見 lawn_boundary.py 開頭的說明。

用法 (bringup 開著、建圖模式 = 模式 0、手動開車)：
    ros2 service call /boundary_recorder/start  std_srvs/srv/Trigger   # 開始記錄
    (沿著草地邊緣開一圈，開回起點附近)
    ros2 service call /boundary_recorder/stop   std_srvs/srv/Trigger   # 收成多邊形並發布
    ros2 service call /boundary_recorder/cancel std_srvs/srv/Trigger   # 放棄這次記錄
    ros2 service call /boundary_recorder/clear  std_srvs/srv/Trigger   # 清除，回到「只看牆」
RViz 可以加 /boundary_recorder/track (Path) 看正在記錄的軌跡、/lawn_boundary (Polygon) 看結果。
/boundary_recorder/recording (std_msgs/Bool，transient local)：true = 正在記錄。網頁人機介面和 car-boundary 讀它。

記的是 map 座標系裡 base_footprint (後輪軸中心) 的位置。邊界就是這條軌跡本身，
不往外推半個車寬：使用者不一定剛好把輪子壓在草地邊上，往外推可能推到草地外面。

【跟著地圖存起來】(2026-10-21) 圈好的外框另外存一份在 ~/.ros/mowerbot_lawn/<session>.yaml (清除時刪掉)。
session = 哪一次建圖 (map_session.py)：用存好的地圖開機時 session 跟那一次一樣，外框自動載回來；
建新地圖時 session 是新的，不會載到別次建圖的外框 (座標系不一樣)。
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros

from mowerbot_interfaces.msg import MowerStatus

from mowerbot_action import lawn_boundary, map_session


class BoundaryRecorder(Node):

    def __init__(self):
        super().__init__('boundary_recorder')
        self.declare_parameter(
            'boundary_file', os.path.expanduser('~/.ros/mowerbot_lawn_boundary.yaml'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        # 【預設不載入舊檔】每次啟動 bringup，slam_toolbox 都從頭建一張新圖，
        # map 座標系的原點就是這次開機時車子停的位置。上一次存的多邊形放進這次的 map，
        # 只要車子起點差一點、朝向差幾度，整個割草範圍就會平移、旋轉到草地外面。
        # 所以 boundary_file 只留紀錄，不要打開 load_on_start；要載回請用下面的 session (同一張地圖才載)。
        self.declare_parameter('load_on_start', False)
        # 2026-10-21：mower_control 傳進來 (map_session.py)。restore = 開機載回同一個 session 存的外框
        self.declare_parameter('session_id', '')
        self.declare_parameter('restore', True)
        self.session_id = self.get_parameter('session_id').value

        self.boundary_file = self.get_parameter('boundary_file').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.boundary_pub = self.create_publisher(PolygonStamped, '/lawn_boundary', latched)
        self.track_pub = self.create_publisher(Path, '~/track', 10)
        self.recording_pub = self.create_publisher(Bool, '~/recording', latched)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.recording = False
        self.recording_pub.publish(Bool(data=False))
        self.track = []
        self._tf_fail_since = None
        self._last_track_pub = 0.0

        self.create_service(Trigger, '~/start', self.start_cb)
        self.create_service(Trigger, '~/stop', self.stop_cb)
        self.create_service(Trigger, '~/cancel', self.cancel_cb)
        self.create_service(Trigger, '~/clear', self.clear_cb)
        self.create_timer(0.1, self.sample)
        # 記錄中切到自動割草 (手把 LB + B)：取消這次記錄。不然自動割草的路線也會被記進邊界裡，
        # 而且那一趟用的還是舊的邊界 (網頁在記錄中會擋住自動割草，手把擋不住)
        self.create_subscription(MowerStatus, '/mower_status', self.mower_status_cb, 10)

        if self.get_parameter('load_on_start').value and os.path.exists(self.boundary_file):
            try:
                poly, frame = lawn_boundary.load_yaml(self.boundary_file)
                self.publish_polygon(poly, frame)
                self.get_logger().warn(
                    f'⚠️ 已載入上次存的草地邊界 {self.boundary_file}（{len(poly)} 個頂點）。'
                    f'只有在使用同一張存檔地圖時才可信。')
            except Exception as exc:     # 檔案壞掉不該讓節點起不來
                self.get_logger().error(f'載入 {self.boundary_file} 失敗：{exc}')
        if self.session_id and self.get_parameter('restore').value:
            self.restore()
        self.get_logger().info(
            '✅ 草地邊界記錄器就緒。沿草地邊緣開一圈前呼叫 ~/start，開回起點附近呼叫 ~/stop。')

    def session_file(self):
        return map_session.lawn_file(self.session_id) if self.session_id else None

    def restore(self):
        path = self.session_file()
        if not os.path.isfile(path):
            return
        try:
            poly, frame, sid = lawn_boundary.load_yaml(path, with_session=True)
        except Exception as exc:     # 檔案壞掉不該讓節點起不來
            self.get_logger().error(f'讀不到存著的外框 {path}：{exc}')
            return
        if sid != self.session_id or len(poly) < 3:
            return
        self.publish_polygon(poly, frame)
        self.get_logger().info(
            f'🌱 載回這張地圖的外框：{len(poly)} 個頂點、面積 {lawn_boundary.polygon_area(poly):.1f} m²')

    # ---- 取樣 ---------------------------------------------------------------
    def sample(self):
        if not self.recording:
            return
        try:
            # 不帶 timeout：只拿 buffer 裡現成的最新值，不在 timer 裡等待
            # (waitForTransform 曾經在本專案造成死鎖，見 sim-verify 紀錄)。
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.robot_frame, Time())
        except Exception as exc:     # tf2 的各種例外都一樣處理
            now = time.monotonic()
            if self._tf_fail_since is None:
                self._tf_fail_since = now
            elif now - self._tf_fail_since > 2.0:
                self.get_logger().warn(
                    f'⚠️ 查不到 {self.global_frame} -> {self.robot_frame}，這段軌跡沒有記到：{exc}',
                    throttle_duration_sec=5.0)
            return
        self._tf_fail_since = None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        lawn_boundary.append_if_moved(self.track, x, y)
        now = time.monotonic()
        if now - self._last_track_pub > 1.0:
            self._last_track_pub = now
            self.publish_track()

    def publish_track(self):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.track:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.track_pub.publish(path)

    def publish_polygon(self, poly, frame_id):
        msg = PolygonStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in poly:
            # z 跟 map_to_boundary 一樣抬高 0.15 m，RViz 裡才不會被地圖蓋住
            msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.15))
        self.boundary_pub.publish(msg)

    def mower_status_cb(self, msg):
        if self.recording and msg.mode == 1:
            self.set_recording(False)
            self.track = []
            self.publish_track()
            self.get_logger().warn('⚠️ 切到自動割草了，這次草地邊界的記錄取消。要記的話先完成 (car-boundary stop) 再割。')

    def set_recording(self, on):
        self.recording = on
        self.recording_pub.publish(Bool(data=on))

    # ---- 服務 ---------------------------------------------------------------
    def start_cb(self, _req, resp):
        self.track = []
        self.set_recording(True)
        self._tf_fail_since = None
        self.publish_track()
        resp.success = True
        resp.message = ('開始記錄。用手把沿著草地邊緣開一整圈，開回起點 %.1f m 內再按完成 (car-boundary stop)。'
                        % lawn_boundary.MAX_CLOSE_GAP_M)
        self.get_logger().info('▶️ ' + resp.message)
        return resp

    def stop_cb(self, _req, resp):
        if not self.recording:
            resp.success = False
            resp.message = '目前沒有在記錄 (先開始記錄)。'
            return resp
        try:
            result = lawn_boundary.close_and_simplify(self.track)
        except ValueError as exc:
            # 繼續記錄：使用者可以再往起點開一點再按一次 stop，不必整圈重來
            resp.success = False
            resp.message = f'{exc} (還在記錄中，可以繼續開；要放棄就取消 = car-boundary cancel)'
            self.get_logger().warn('⚠️ ' + resp.message)
            return resp
        self.set_recording(False)
        poly = result['polygon']
        self.publish_polygon(poly, self.global_frame)
        saved = ''
        try:
            for path in filter(None, (self.boundary_file, self.session_file())):
                os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
                lawn_boundary.save_yaml(path, poly, self.global_frame,
                                        stamp_sec=time.time(), session_id=self.session_id or None)
            saved = ('，跟著這張地圖存好了 (下次用這張圖開機自動載回)' if self.session_id
                     else f'，紀錄存到 {self.boundary_file}')
        except OSError as exc:
            saved = f'，但存檔失敗：{exc}'
        resp.success = True
        resp.message = (f'草地邊界完成：{len(self.track)} 個取樣點 -> {len(poly)} 個頂點，'
                        f'面積 {result["area"]:.1f} m²、周長 {result["perimeter"]:.1f} m、'
                        f'終點離起點 {result["gap"]:.2f} m{saved}')
        self.get_logger().info('✅ ' + resp.message)
        return resp

    def cancel_cb(self, _req, resp):
        was = self.recording
        self.set_recording(False)
        self.track = []
        self.publish_track()
        resp.success = True
        resp.message = '已放棄這次記錄。' if was else '目前沒有在記錄。'
        self.get_logger().info(resp.message)
        return resp

    def clear_cb(self, _req, resp):
        self.set_recording(False)
        self.track = []
        self.publish_track()
        # 空的多邊形 = 「沒有草地邊界」，map_to_boundary 回到只看牆與障礙物
        self.publish_polygon([], self.global_frame)
        path = self.session_file()
        if path and os.path.isfile(path):
            try:
                os.remove(path)
            except OSError as exc:
                self.get_logger().error(f'刪不掉存著的外框 {path}：{exc}')
        resp.success = True
        resp.message = '已清除草地邊界，割草範圍回到只由地圖 (牆、障礙物) 決定。'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
