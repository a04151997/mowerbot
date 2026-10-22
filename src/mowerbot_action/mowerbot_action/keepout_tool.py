#!/usr/bin/env python3
"""禁區的命令列工具 (2026-10-18)。說明見 keepout.py 開頭。

    ros2 run mowerbot_action keepout_tool session              印出這次開機的 session_id
    ros2 run mowerbot_action keepout_tool template <地圖>       產生給人畫的 <地圖>_draw.png
    ros2 run mowerbot_action keepout_tool apply <地圖> [圖片]   紅色 -> 禁區，套用到正在跑的程式
    ros2 run mowerbot_action keepout_tool clear                清除禁區

<地圖> 是存圖時的路徑、不帶副檔名 (save_map.sh 的 <名稱>)，例如 ~/mowerbot/src/mowerbot_bringup/maps/lab1。
[圖片] 預設是 <地圖>_draw.png。
session、apply、clear 要在 bringup 開著時用；template 不用。
"""
import argparse
import os
import sys

import cv2

from mowerbot_action import keepout

DEFAULT_KEEPOUT_DIR = os.path.expanduser('~/.ros/mowerbot_keepout')
SERVER = '/keepout_server'


def call_trigger(service, timeout=10.0):
    """呼叫 std_srvs/Trigger，回傳 (success, message)。服務不在就回傳 (False, 原因)。"""
    import rclpy
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('keepout_tool')
    try:
        cli = node.create_client(Trigger, service)
        if not cli.wait_for_service(timeout_sec=timeout):
            return False, ('找不到 %s：車上程式 (bringup) 沒開，或是舊版程式沒有禁區功能。' % service)
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return False, '%s 沒有回應。' % service
        return bool(fut.result().success), fut.result().message
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cmd_session(_args):
    ok, msg = call_trigger(SERVER + '/session')
    if not ok:
        print('❌ ' + msg, file=sys.stderr)
        return 1
    print(msg)
    return 0


def cmd_template(args):
    prefix = args.map
    meta = keepout.load_map_meta(prefix + '.yaml')
    out = args.out or prefix + '_draw.png'
    if os.path.exists(out) and not args.force:
        print('❌ %s 已經存在，不會蓋掉 (要重做加 --force)。' % out, file=sys.stderr)
        return 1
    gray = keepout.read_gray(meta['image'])
    if not cv2.imwrite(out, keepout.template_from_gray(gray)):
        print('❌ 寫不出 %s' % out, file=sys.stderr)
        return 1
    h, w = gray.shape
    print('✅ 畫禁區用的圖：%s (%d x %d 像素，1 像素 = %.0f 公分)'
          % (out, w, h, meta['resolution'] * 100))
    return 0


def cmd_apply(args):
    prefix = args.map
    drawn_path = args.image or prefix + '_draw.png'
    session_path = prefix + '.session'
    try:
        meta = keepout.load_map_meta(prefix + '.yaml')
        template = keepout.template_from_gray(keepout.read_gray(meta['image']))
        drawn = cv2.imread(drawn_path, cv2.IMREAD_COLOR)
        if drawn is None:
            raise ValueError('讀不到畫好的圖 %s' % drawn_path)
        red, other = keepout.drawing_to_keepout(drawn, template)
    except (OSError, ValueError) as exc:
        print('❌ %s' % exc, file=sys.stderr)
        return 1
    zones, area = keepout.summarize(red, meta['resolution'])
    if zones == 0:
        print('❌ 圖上找不到紅色。請用紅色的筆把牆和禁區畫上去，存檔後再傳一次。', file=sys.stderr)
        if other > 0:
            print('   (有 %d 個像素被改過但不是紅色，可能用了別的顏色)' % other, file=sys.stderr)
        return 1
    try:
        with open(session_path, encoding='utf-8') as fh:
            session_id = fh.read().strip()
    except OSError:
        print('❌ 找不到 %s：這張地圖存的時候沒有記下是哪一次開機，不能拿來畫禁區。'
              '請重新存圖再畫。' % session_path, file=sys.stderr)
        return 1

    record = keepout.write_mask(prefix + '_keepout', red, meta['resolution'],
                                meta['origin_x'], meta['origin_y'], session_id,
                                source=os.path.abspath(drawn_path))
    os.makedirs(args.keepout_dir, exist_ok=True)
    keepout.write_mask(os.path.join(args.keepout_dir, 'active'), red, meta['resolution'],
                       meta['origin_x'], meta['origin_y'], session_id,
                       source=os.path.abspath(drawn_path))
    print('畫好的圖：%d 塊紅色、共 %.2f m² (紀錄存在 %s)' % (zones, area, record))
    if other > 50:
        print('⚠️ 有 %d 個像素被改過但不是紅色 (可能用了別的顏色的筆)，那些不算禁區。' % other)

    ok, msg = call_trigger(SERVER + '/load')
    print(('✅ ' if ok else '❌ ') + msg, file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


def cmd_clear(_args):
    ok, msg = call_trigger(SERVER + '/clear')
    print(('✅ ' if ok else '❌ ') + msg, file=sys.stdout if ok else sys.stderr)
    return 0 if ok else 1


def main(argv=None):
    # ros2 run 會在後面加 --ros-args ...，這個工具用不到
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    ap = argparse.ArgumentParser(prog='keepout_tool', description='地圖上畫的禁區')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('session', help='印出這次開機的 session_id')
    p = sub.add_parser('template', help='產生給人畫的 <地圖>_draw.png')
    p.add_argument('map', help='地圖路徑，不帶副檔名')
    p.add_argument('--out', help='輸出圖片 (預設 <地圖>_draw.png)')
    p.add_argument('--force', action='store_true', help='已經有圖時蓋掉')
    p = sub.add_parser('apply', help='把紅色換成禁區並套用')
    p.add_argument('map', help='地圖路徑，不帶副檔名')
    p.add_argument('image', nargs='?', help='畫好的圖 (預設 <地圖>_draw.png)')
    p.add_argument('--keepout-dir', default=DEFAULT_KEEPOUT_DIR,
                   help='keepout_server 讀 active.yaml 的資料夾')
    sub.add_parser('clear', help='清除禁區')
    args = ap.parse_args(argv)
    if getattr(args, 'map', None):
        args.map = os.path.expanduser(args.map)
        if args.map.endswith('.yaml'):
            args.map = args.map[:-len('.yaml')]
    return {'session': cmd_session, 'template': cmd_template,
            'apply': cmd_apply, 'clear': cmd_clear}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
