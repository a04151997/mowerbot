#!/usr/bin/env python3
"""手把 Y 存圖 (2026-10-19 will)。

呼叫的人：teleop 的 Y 鍵；以及 save_map_before_f2c 開著時的 mower_manager (切進 mode 1 先存圖，
存成功才規劃。2026-10-18 的做法，10-19 起實車預設關掉：will 要 LB + B 直接規劃割草、存圖改按 Y)。
存圖本身交給 mowerbot_bringup 的 save_map.sh (car-savemap 用的同一支)：重試 save_map、
free_thresh 改 0.10、記下這次開機的 session 與畫禁區用的圖，全部只有那一處。

服務 (std_srvs/srv/Trigger)：
    ~/save   存成 <prefix>_<月日>_<時分秒>，成功時 message 是存好的路徑 (不含副檔名)。
             還沒開始建圖 (/slam_scan_gate/active 是 false) 時直接拒絕。
話題：
    ~/last   (std_msgs/String，latched) 最後一次存圖的結果，car-status 顯示這一行
    ~/busy   (std_msgs/Bool，latched) 存圖中 = true。mower_manager 在存圖中不送出第一段：
             存圖時 slam_toolbox 序列化地圖，map -> odom 會斷一下，路線會被判失敗 (2026-10-20)
參數：
    map_dir  存到哪裡；留空 = save_map.sh 自己決定 (原始碼的 maps/，與 car-savemap 相同)
    prefix   檔名開頭，預設 map
    timeout  save_map.sh 最多跑幾秒 (N150 上一次約 10 ~ 30 秒)
    saved_map  用存好的地圖開機時 (2026-10-21，map_session.py) 的那張地圖：定位模式不會改地圖，
             不用再存，~/save 直接回答成功 (message = 那張地圖)，手把 Y 的路線預覽照常
save_map.sh 的完整輸出存在 ~/.ros/mowerbot_autosave/<名稱>.log。
"""
import os
import re
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

MAP_EXTS = ('pgm', 'yaml', 'posegraph', 'data')


class MapAutosave(Node):

    def __init__(self):
        super().__init__('map_autosave')
        self.declare_parameter('map_dir', '')
        self.declare_parameter('prefix', 'map')
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/.ros/mowerbot_autosave'))
        self.declare_parameter('saved_map', '')
        self.saved_map = self.get_parameter('saved_map').value
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # 建圖開始了沒：slam_scan_gate 的 latched 話題。沒有這個話題 (模擬，一開機就在建圖) = 不擋
        self.mapping = None
        self.create_subscription(Bool, '/slam_scan_gate/active',
                                 lambda m: setattr(self, 'mapping', bool(m.data)), latched)
        self.last_pub = self.create_publisher(String, '~/last', latched)
        self.busy_pub = self.create_publisher(Bool, '~/busy', latched)
        self.busy_pub.publish(Bool(data=False))
        self.create_service(Trigger, '~/save', self.save_cb)
        self.get_logger().info('💾 存圖就緒：手把按 Y 存一份地圖。')

    @staticmethod
    def script_path():
        from ament_index_python.packages import get_package_prefix
        return os.path.join(get_package_prefix('mowerbot_bringup'),
                            'lib', 'mowerbot_bringup', 'save_map.sh')

    def save_cb(self, _req, resp):
        if self.saved_map:
            resp.success = True
            resp.message = self.saved_map
            self.get_logger().info('💾 用的是存好的地圖 %s，不用再存' % self.saved_map)
            self.last_pub.publish(String(data='%s ✅ 沿用存好的地圖 %s (不用再存)' % (
                time.strftime('%H:%M:%S'), os.path.basename(self.saved_map))))
            return resp
        if self.mapping is False:
            ok, message = False, '還沒開始建圖 (按住 LB + A)，沒有地圖可以存'
        else:
            name = '%s_%s' % (self.get_parameter('prefix').value, time.strftime('%m%d_%H%M%S'))
            self.busy_pub.publish(Bool(data=True))
            try:
                ok, message = self.save(name)
            finally:
                self.busy_pub.publish(Bool(data=False))
        resp.success = ok
        resp.message = message
        if ok:
            self.get_logger().info('💾 地圖已存：' + message)
        else:
            self.get_logger().error('⚠️ 存圖失敗：' + message)
        self.last_pub.publish(String(data='%s %s%s' % (
            time.strftime('%H:%M:%S'), '✅ 已存 ' if ok else '❌ 失敗：', message)))
        return resp

    def save(self, name):
        try:
            script = self.script_path()
        except Exception as exc:        # PackageNotFoundError 等
            return False, '找不到 mowerbot_bringup 的 save_map.sh (%s)' % exc
        if not os.path.isfile(script):
            return False, '找不到 %s' % script
        env = dict(os.environ)
        map_dir = self.get_parameter('map_dir').value
        if map_dir:
            env['MAP_DIR'] = os.path.expanduser(map_dir)
        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, name + '.log')
        self.get_logger().info('💾 存圖中：%s (save_map.sh，約 10 ~ 30 秒)' % name)
        try:
            out = subprocess.run(['bash', script, name], env=env,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 timeout=float(self.get_parameter('timeout').value))
            text = out.stdout.decode('utf-8', 'replace')
            rc = out.returncode
        except subprocess.TimeoutExpired as exc:
            text = (exc.stdout or b'').decode('utf-8', 'replace')
            rc = None
        with open(log_path, 'w', encoding='utf-8') as fh:
            fh.write(text)
        if rc is None:
            return False, 'save_map.sh 超過 %.0f 秒沒有結束 (紀錄 %s)' % (
                self.get_parameter('timeout').value, log_path)
        m = re.search(r'^存圖目標 \(不含副檔名\): (.+)$', text, re.MULTILINE)
        target = m.group(1).strip() if m else None
        missing = [e for e in MAP_EXTS if not (target and os.path.isfile('%s.%s' % (target, e)))]
        if rc != 0 or missing:
            return False, 'save_map.sh 回傳 %s，缺 %s (紀錄 %s)' % (
                rc, ' '.join('.' + e for e in missing) or '(無)', log_path)
        return True, target


def main(args=None):
    rclpy.init(args=args)
    node = MapAutosave()
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
