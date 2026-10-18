#!/usr/bin/env python3
"""發布使用者在地圖上畫的禁區 (2026-10-18)。為什麼需要、怎麼畫，見 keepout.py 開頭的說明。

發布 (transient local)：
    /keepout_mask   nav_msgs/OccupancyGrid，map 座標系，100 = 禁區、0 = 不是。
                    map_to_boundary 與 local costmap 的 keepout_layer 都讀它。
                    沒有禁區時是 odom 座標系的 1 x 1 空遮罩 (見 publish_empty)。
服務 (std_srvs/srv/Trigger)：
    ~/session   回傳這次開機的 session_id (save_map.sh 存圖時記下來)
    ~/load      載入 <keepout_dir>/active.yaml (keepout_tool apply 寫的)
    ~/clear     清除禁區
一般不直接呼叫，用 keepout_tool (ros2 run mowerbot_action keepout_tool -h)。

【開機永遠沒有禁區】session_id 每次啟動都重新產生，開機不載入任何舊檔。
上一次開機畫的禁區放進這次的 map 座標系，只要車子起點差一點、朝向差幾度，整片禁區就會
平移、旋轉到別的地方 —— 而且畫面上看起來一切正常。所以只接受這次開機存的地圖畫出來的禁區。
"""
import os
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger

from mowerbot_action import keepout


class KeepoutServer(Node):

    def __init__(self):
        super().__init__('keepout_server')
        self.declare_parameter('keepout_dir', os.path.expanduser('~/.ros/mowerbot_keepout'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('mask_topic', '/keepout_mask')
        self.declare_parameter('empty_frame', 'odom')
        self.keepout_dir = self.get_parameter('keepout_dir').value
        self.global_frame = self.get_parameter('global_frame').value
        self.empty_frame = self.get_parameter('empty_frame').value
        mask_topic = self.get_parameter('mask_topic').value

        self.session_id = time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6]

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.mask_pub = self.create_publisher(OccupancyGrid, mask_topic, latched)
        # 一開始就發一張「沒有禁區」的遮罩 (1 x 1、值 0)，訂閱的一方不用處理「還沒收到」
        self.publish_empty()

        self.create_service(Trigger, '~/session', self.session_cb)
        self.create_service(Trigger, '~/load', self.load_cb)
        self.create_service(Trigger, '~/clear', self.clear_cb)
        self.get_logger().info(
            f'✅ 禁區節點就緒 (session {self.session_id})。開機不載入舊的禁區：'
            f'要用這次開機存的地圖畫，再用 keepout_tool apply 套用。')

    def publish_empty(self):
        # 空遮罩放在 odom，不放在 map (2026-10-18)：實車要按 LB + A 才開始建圖，在那之前
        # 沒有 map 座標系。local costmap 的 keepout_layer (StaticLayer) 收到 map 座標系的遮罩
        # 之後，每次更新都要查 map -> odom，查不到就印一行 ERROR (實測一秒約 3 行，直到開始建圖)。
        # 空的遮罩放在哪個座標系都一樣是「沒有禁區」，odom 一開機就有。
        msg = OccupancyGrid()
        msg.header.frame_id = self.empty_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = 0.05
        msg.info.width = 1
        msg.info.height = 1
        msg.info.origin.orientation.w = 1.0
        msg.data = [0]
        self.mask_pub.publish(msg)

    def publish_mask(self, m):
        msg = OccupancyGrid()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(m['resolution'])
        msg.info.height, msg.info.width = (int(v) for v in m['grid'].shape)
        msg.info.origin.position.x = float(m['origin_x'])
        msg.info.origin.position.y = float(m['origin_y'])
        msg.info.origin.orientation.w = 1.0
        msg.data = (m['grid'].astype('int8') * 100).ravel().tolist()
        self.mask_pub.publish(msg)

    def session_cb(self, _req, resp):
        resp.success = True
        resp.message = self.session_id
        return resp

    def load_cb(self, _req, resp):
        path = os.path.join(self.keepout_dir, 'active.yaml')
        try:
            m = keepout.load_mask(path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            resp.success = False
            resp.message = f'讀不到禁區檔 {path}：{exc}'
            self.get_logger().error('⚠️ ' + resp.message)
            return resp
        if m['session_id'] != self.session_id:
            resp.success = False
            resp.message = (
                f'這張禁區是用另一次開機存的地圖畫的 (session {m["session_id"]})，'
                f'現在是 {self.session_id}。每次開機地圖的原點都不一樣，舊的禁區會對不上，'
                f'所以不載入。請這次開機重新存圖、重新畫。')
            self.get_logger().error('🚫 ' + resp.message)
            return resp
        if m['zones'] == 0:
            resp.success = False
            resp.message = '禁區檔裡沒有任何禁區，沒有載入。要清除禁區請用 clear。'
            self.get_logger().warn('⚠️ ' + resp.message)
            return resp
        self.publish_mask(m)
        resp.success = True
        resp.message = (f'已套用禁區：{m["zones"]} 塊、共 {m["area"]:.2f} m²。'
                        f'割草範圍與 Nav2 都會避開。')
        self.get_logger().info('⛔ ' + resp.message)
        return resp

    def clear_cb(self, _req, resp):
        self.publish_empty()
        resp.success = True
        resp.message = '已清除禁區，割草範圍回到只由地圖 (與草地邊界) 決定。'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutServer()
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
