# 模擬展示影片（README）

影片本身在 `test/logs/`（不進版控，.gitignore）。產生方式與數據見 `docs/simulation_results.md` 38.17。

| 檔案 | 內容 | 長度 / 倍率 |
|---|---|---|
| `test/logs/20261002_c1_demo_rviz_screen_x10.mp4` | 完整 C1 一趟的 RViz 螢幕錄影（任務開始 → DONE） | 108 s，**10 倍速**（實際 1084 s） |
| `test/logs/20261002_c1_demo_replay_x20.mp4` | 同一趟由紀錄重畫的俯視動畫（不是螢幕錄影） | 54 s，**20 倍速**（模擬 1017 s） |
| `test/logs/20261002_align_pn_curve.png` | 單次對正成功率 p 與 p^n 曲線 | — |

設定：stage29_lawn 同圖、demo_lawn.world、align yaw 容忍 0.25、AlignController min_speed_theta 0.3。
重現：`ALIGN_MST=0.3 RECORD=1 test/tools/stage38_demo_run.sh 0.25 <輸出目錄>`，
俯視動畫：`python3 test/tools/replay_video.py <run 目錄> <輸出.mp4> --speed 20`。

## ⚠️ 格式：mp4v，放進簡報前要轉檔

這台 VM 沒有 ffmpeg（要 sudo 才能裝），影片是用 OpenCV 以 **mp4v（MPEG-4 Part 2）** 編碼的。
VLC 與多數桌面播放器能播，但**瀏覽器、PowerPoint / Google 簡報不一定能播**。放進簡報前轉成 H.264：

```bash
sudo apt install ffmpeg      # 只要裝一次
ffmpeg -i 20261002_c1_demo_rviz_screen_x10.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart \
       20261002_c1_demo_rviz_screen_x10_h264.mp4
```

## 已知的畫面瑕疵

* 螢幕錄影右上角一直有一個「colcon build successful」的桌面通知（錄影前的重編留下的），不影響內容。
* 沒有 Gazebo 的 3D 畫面：gzclient 在這台 VM 會中途閃退並拖垮整組（demo.launch.py 記載），沒有冒險。
* 倍率寫在檔名、畫面左下角（螢幕錄影）/ 上方（俯視動畫），以及同名 .txt。
