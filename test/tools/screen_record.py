# -*- coding: utf-8 -*-
"""錄螢幕成 MP4 (階段 38 後續 6；這台 VM 沒有 ffmpeg，改用 Pillow 抓 X 畫面 + OpenCV 編碼)。

用法：python3 test/tools/screen_record.py <輸出.mp4> [--fps 2] [--speed 10] [--scale 0.5] [--display :0]
  每 1/fps 秒抓一張，以 fps x speed 的播放速率寫入 —— 影片是 speed 倍速。收到 SIGINT / SIGTERM 就收尾。
  結束時印出「實際錄了幾秒、影片幾秒、倍率」，並把倍率寫進同名 .txt。
編碼是 mp4v (MPEG-4 Part 2)：VLC / 一般播放器都能播；瀏覽器不一定，需要時再轉 H.264。
"""
import argparse, signal, time

import cv2
import numpy as np
from PIL import ImageGrab

ap = argparse.ArgumentParser()
ap.add_argument('out')
ap.add_argument('--fps', type=float, default=2.0)
ap.add_argument('--speed', type=float, default=10.0)
ap.add_argument('--scale', type=float, default=0.5)
ap.add_argument('--display', default=':0')
a = ap.parse_args()
stop = {'v': False}
for s in (signal.SIGINT, signal.SIGTERM):
    signal.signal(s, lambda *_: stop.update(v=True))
im = ImageGrab.grab(xdisplay=a.display)
W, H = int(im.size[0] * a.scale) // 2 * 2, int(im.size[1] * a.scale) // 2 * 2
vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), a.fps * a.speed, (W, H))
t0 = time.time(); n = 0
while not stop['v']:
    t = time.time()
    fr = cv2.cvtColor(np.array(ImageGrab.grab(xdisplay=a.display)), cv2.COLOR_RGB2BGR)
    fr = cv2.resize(fr, (W, H), interpolation=cv2.INTER_AREA)
    cv2.putText(fr, '%s  x%.0f' % (time.strftime('%H:%M:%S'), a.speed), (10, H - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    vw.write(fr); n += 1
    time.sleep(max(0.0, 1.0 / a.fps - (time.time() - t)))
vw.release()
rec = time.time() - t0
msg = '錄了 %.0f s 實際時間，%d 張，影片 %.0f s，%.0f 倍速' % (rec, n, n / (a.fps * a.speed), a.speed)
print(msg)
open(a.out.rsplit('.', 1)[0] + '.txt', 'w').write(msg + '\n')
