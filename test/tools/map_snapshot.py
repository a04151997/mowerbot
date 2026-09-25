#!/usr/bin/env python3
"""抓一張 OccupancyGrid，數 -1 / 0 / 100 / 其他 各幾格（報告 29.1，只量測）。

用法:
    python3 test/tools/map_snapshot.py <topic> <輸出.npz> [--volatile]
    python3 test/tools/map_snapshot.py --pgm <地圖.yaml>      # 直接讀 .pgm 的灰階值

另外照 map_to_boundary.py 的規則（0 <= v <= 20 才算草地）數「自由格」與
外輪廓、內部洞，讓「哪一層把 -1 變成 0」可以用數字對照。
"""
import sys, time
import numpy as np
import cv2


def analyse(grid, res, ox, oy):
    vals, cnt = np.unique(grid, return_counts=True)
    d = dict(zip(vals.tolist(), cnt.tolist()))
    print('  格數：-1(未知)=%d  0(自由)=%d  100(佔據)=%d  其他=%d  總=%d'
          % (d.get(-1, 0), d.get(0, 0), d.get(100, 0),
             sum(c for v, c in d.items() if v not in (-1, 0, 100)), grid.size))
    free = ((grid >= 0) & (grid <= 20)).astype(np.uint8) * 255
    pad = cv2.copyMakeBorder(free, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
    cs, h = cv2.findContours(pad, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    outer = [i for i in range(len(cs)) if h[0][i][3] == -1]
    L = max(outer, key=lambda i: cv2.contourArea(cs[i]))
    holes, small = [], 0
    c = h[0][L][2]
    while c != -1:
        a = abs(cv2.contourArea(cs[c])) * res * res
        if a >= 0.09:
            p = cs[c].reshape(-1, 2) - 1
            holes.append((a, ox + p[:, 0].min() * res, ox + p[:, 0].max() * res,
                          oy + p[:, 1].min() * res, oy + p[:, 1].max() * res))
        else:
            small += 1
        c = h[0][c][0]
    print('  map_to_boundary 規則：自由格 %d（%.2f m²），最大外輪廓 %.2f m²，'
          '內部洞 >= 0.09 m² 有 %d 個（另 %d 個小洞）'
          % (int((free > 0).sum()), (free > 0).sum() * res * res,
             cv2.contourArea(cs[L]) * res * res, len(holes), small))
    for a, x0, x1, y0, y1 in sorted(holes, reverse=True):
        print('     洞 %.2f m²  x %.2f ~ %.2f  y %.2f ~ %.2f' % (a, x0, x1, y0, y1))


def from_pgm(yaml_path):
    import yaml, os
    y = yaml.safe_load(open(yaml_path))
    img = cv2.imread(os.path.join(os.path.dirname(yaml_path), y['image']), cv2.IMREAD_UNCHANGED)
    vals, cnt = np.unique(img, return_counts=True)
    print('%s：mode=%s free_thresh=%s occupied_thresh=%s negate=%s'
          % (yaml_path, y.get('mode'), y.get('free_thresh'), y.get('occupied_thresh'), y.get('negate')))
    print('  .pgm 灰階值：%s' % ', '.join('%d×%d' % (v, c) for v, c in zip(vals, cnt)))
    for v in vals:
        p = (255 - int(v)) / 255.0
        cls = '佔據' if p > y['occupied_thresh'] else ('自由' if p < y['free_thresh'] else '未知')
        print('    灰階 %3d -> p = %.4f -> map_server 會判成 %s' % (v, p, cls))


def main():
    if sys.argv[1] == '--pgm':
        from_pgm(sys.argv[2])
        return
    topic, out = sys.argv[1], sys.argv[2]
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    rclpy.init()
    n = Node('map_snapshot')
    dur = DurabilityPolicy.VOLATILE if '--volatile' in sys.argv else DurabilityPolicy.TRANSIENT_LOCAL
    got = []
    n.create_subscription(OccupancyGrid, topic, got.append,
                          QoSProfile(depth=1, durability=dur, reliability=ReliabilityPolicy.RELIABLE))
    t0 = time.time()
    while not got and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not got:
        print('%s：30 秒內沒收到' % topic)
        sys.exit(1)
    m = got[-1]
    g = np.array(m.data, dtype=np.int8).reshape((m.info.height, m.info.width))
    np.savez(out, grid=g, res=m.info.resolution,
             ox=m.info.origin.position.x, oy=m.info.origin.position.y)
    print('%s：%dx%d，解析度 %.3f，原點 (%.2f, %.2f)'
          % (topic, m.info.width, m.info.height, m.info.resolution,
             m.info.origin.position.x, m.info.origin.position.y))
    analyse(g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
