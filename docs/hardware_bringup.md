# 實機上線步驟（M0 ~ M7）

**相關文件**：模擬端的驗證數據在 [`simulation_results.md`](simulation_results.md)，
系統架構與資料流圖在 [`architecture.md`](architecture.md)，
驅動板要查的資訊清單在 [`../src/mowerbot_bridge/mowerbot_bridge/drivers/README.md`](../src/mowerbot_bridge/mowerbot_bridge/drivers/README.md)。

這份文件寫給**拿著手機、站在車子旁邊照著做的人**。每一步都有：
可以直接複製貼上的指令、正常時會看到什麼、最常見的三種失敗與徵狀、安全前提。
**沒有做到「怎麼確認成功」的步驟不算做完，不要跳到下一步。**

指令裡的 `<尖括號>` 是要換成實際值的地方。所有指令都假設已經：

```bash
source /opt/ros/humble/setup.bash
source ~/mowerbot/install/setup.bash      # workspace 放在別處就改這一行
```

目前的狀態：

| 項目 | 狀態 |
|------|------|
| 模擬端（SLAM / F2C / Nav2 / 安全機制） | 完成，自動化測試 51 項全過，見 `simulation_results.md` |
| `bridge_node`（ROS ↔ 驅動板） | 骨架完成，用 loopback 假驅動測過（Phase H） |
| 里程計數學 | 完成，單元測試在 `mowerbot_bridge/test/test_odometry.py` |
| 馬達驅動（`drivers/wheeltec.py`） | 依輪趣 C30D 協定寫的，**待驗證**：新控制板 STM32F407 的協定待學長 ROS 1 原始碼比對 |
| 光達驅動 | **空白**：RPLIDAR，型號細節待確認 |
| 車輛幾何 `vehicle.yaml` | 階段 38：實測 / 粗略實測 / 暫定三類並存，見下面「實機資訊」與 `measurement_worklist.md` |

---

## 實機資訊（階段 38，2026-10-02）

| 項目 | 內容 |
|------|------|
| 車輛 | **XLK X2RS**，Honda GCVxe200 汽油引擎驅動刀盤。前輪萬向腳輪 / 後輪加裝減速馬達（附編碼器）。電池 XLK 24V |
| 原廠規格 | 98 × 73 × 46 cm，57.2 kg + 電池 16.2 kg，割幅約 50 cm，圓盤 4 支刀組，割草高度 1 ~ 8 cm 無段可調，最高車速 4.5 km/h |
| | ⚠ 最高車速是原廠值，**本機已更換驅動馬達，不適用**，實際上限待實測。⚠ 本機加裝鋁架，實測外形 1.13 × 0.84 m（`vehicle.yaml` 一律用實測值） |
| 控制板 | **STM32F407**。OLED 顯示 DIFF / GYRO / ROS / 左右輪數值 / BIAS；板上有 MotorA ~ D 四個馬達通道（目前用兩個）、USB-C、Power 滑動開關、USER / RESET 按鍵。具體型號未確認，但目錄結構（`turn_on_mini3_robot`）與 OLED 格式高度符合輪趣（Wheeltec）系控制板 |
| 光達 | **RPLIDAR**，裝於引擎與電池之間的鋁架上，位於後輪軸前方。現場目視確認**未**被引擎遮擋 |
| 深度相機 | **無**。學長 workspace 裡的 `ros_astra_camera` / `rtabmap` / `3d_navigation` 全是原「mini3」平台遺留，與割草機無關，忽略 |
| 上位機 | Intel N100 四核 / 約 16 GB RAM |
| 上位機 OS | **Ubuntu 18.04.6 LTS**，核心 `5.16.0-051600rc7-generic`（手動安裝的 mainline RC 核心，為支援 N100 硬體）。⚠ **不要執行 `apt upgrade`**，會動到這顆核心，換回舊核心可能導致顯示異常 |
| 原有系統 | ROS Melodic，workspace 在 `/home/dreamworker/catkin_ws`，驅動節點 `turn_on_mini3_robot`（`mini3_robot.cpp / .h`）。⚠ **原系統已完整備份，不得覆蓋或重裝** |

**原系統的用途：已知可用的參照系統。** 我方驅動不動輪子時，先停掉我方節點、開原系統看輪子會不會動，
就能立刻區分是程式問題還是硬體問題。

**⚠ ROS 1 與 ROS 2 的驅動節點不可同時執行**（`CLAUDE.md` 硬性規則）：兩者會搶同一個序列埠。
每次測試前先跑：

```bash
ps aux | grep -E "mini3|bridge_node|roslaunch|ros2" | grep -v grep     # 必須沒有輸出才能開始
```

**ROS 2 的執行環境：chroot（已決定，2026-10-02）。** 上位機是 Ubuntu 18.04，ROS 2 Humble 需要 22.04，做法如下：

| 項目 | 做法 |
|------|------|
| rootfs | 在家的 VM 用 `debootstrap` 做一套 Ubuntu 22.04 rootfs，裝好 ROS 2 Humble 與本專案套件，打包帶到實驗室解壓 |
| 執行 | `chroot` 進去跑。22.04 使用者空間跑在 5.16 核心上（比 22.04 原廠的 5.15 還新） |
| 學長的系統 | 18.04 / Melodic / mainline 5.16 核心**完全不動** |
| 序列埠 | `mount --rbind /dev <rootfs>/dev` |
| GUI | bind mount `/tmp/.X11-unix` |
| 網路 | 實驗室有網路，現場可以在 chroot 裡 `apt install`、重新編譯 |

rootfs 的建置步驟另外提供（不在本輪）。M0 的「N150 + 22.04 安裝流程」在這台上位機上改由 chroot 取代。
chroot 裡外的驅動節點仍然**不可同時執行**（上面的硬性規則）。

---

## 安全總表：哪幾步必須架高、輪子離地

74 kg 的機器，最高速 0.7 m/s。接線接反、編碼器方向相反、或速度單位換算差一個數量級，
車子會瞬間往某個方向衝出去，而你的手正放在它旁邊。

| 步驟 | 架高、輪子離地 | 急停在手邊 | 旁邊要有第二個人 | 場地淨空 |
|------|:---:|:---:|:---:|------|
| M0 裝系統、編譯 | 不用（不通馬達電） | —— | —— | —— |
| M1 接通 C30D | **必須** | **必須** | 建議 | —— |
| M2 量七個數字 | 不用（不通馬達電） | —— | —— | —— |
| M3 極性驗證 | **必須** | **必須** | **必須** | —— |
| M6 實體手把驗證 | **必須** | **必須** | **必須** | —— |
| **M-1 原地旋轉角速度量測** | 落地（M3、M6 全過之後才可以） | **必須** | **必須** | 半徑 2 m 以上 |
| M4 里程計標定 | 落地 | **必須** | **必須** | 直線方向 3 m、周圍 2 m |
| M5 光達安裝與 TF | 不用（不通馬達電） | —— | —— | —— |
| M7 落地低速 → 建圖 → 自動割草 | 落地 | **必須** | **必須** | 整個場地淨空、沒有人在裡面 |

另外，從第一次通電到 M6 全部通過之前：

- 電池先接一顆保險絲，或用可限流的電源供應器
- 急停開關放在**手邊**，不是放在車上
- 刀盤馬達**不接電**（到 M7 最後一段才接）

建議的執行順序：**M0 → M1 → M2 → M3 → M6 → M-1（原地旋轉量測）→ M4 → M5 → M7**。
M-1 寫在最前面是因為它最重要，但它需要車子能動、而且安全機制都驗過，所以排在 M6 之後做。

---

## M-1 草地上原地旋轉角速度量測（最優先，結果可能推翻後面的規劃策略）

### 為什麼這一項排第一

整套自動割草的規劃都假設**車子可以原地掉頭**：弓字形割草線走到底要原地轉 180°，
淨空檢查用的外接半徑、地頭寬 0.70 m 都是這樣算出來的。

真車是**後兩輪驅動 + 前兩輪固定方向**（滑移轉向）。原地旋轉時前輪要被拖著橫向刮過地面。
模擬裡的實驗（`simulation_results.md` 30.4 節）顯示：
**前輪的側向抓地力一旦到後輪的一半左右，車子就完全轉不動**，後輪只會空轉。
模擬沒辦法告訴我們真車落在哪一邊 —— 那取決於草地、輪胎、前後軸各壓多少重量。

**這項量測在拿到結果之前，程式裡的 URDF、地頭寬、車輛幾何、所有跟掉頭有關的邏輯都凍結不動**
（見專案根目錄 `CLAUDE.md`）。

### 前提

- M0、M1、M2、M3、M6 全部通過（車子能用手把開、急停與 deadman 都驗過）
- 刀盤馬達不接電
- 手機或相機架在三腳架上，從**正上方或斜上方**拍得到整台車與地面的記號

### 要量的四件事

| # | 量什麼 | 怎麼量 | 記錄 |
|---|--------|--------|------|
| 1 | **草地**原地轉 360° 的秒數 | 手把全推（左右軸推到底 = 指令 1.2 rad/s），從車頭對準記號開始計時，轉回記號停表。做**順時針 3 次、逆時針 3 次** | 每次秒數；實際角速度 = 2π ÷ 秒數（rad/s） |
| 2 | **水泥地**同樣一次（對照組） | 同上，順逆各 3 次 | 同上 |
| 3 | 前軸、後軸各自承重 | 兩個體重計，前兩輪各壓一個 → 讀兩個數字相加 = 前軸重；換後兩輪 → 後軸重 | 前軸 kg、後軸 kg；兩者相加應該接近整車重，差超過 5 kg 就重量一次 |
| 4 | **全程錄影** | 每一次旋轉都錄，畫面裡要看得到車與地面記號 | 影片檔名寫進記錄表 |

### 具體指令

```bash
# 終端機 1：實車啟動（C30D 的 driver 名稱與 tick 數用 M1 / M3 確認過的值）
ros2 launch mowerbot_bringup bringup_real.launch.py \
     driver_type:=<M1 的 driver 名稱> encoder_ticks_per_rev:=<M3 確認的 tick 數>

# 終端機 2：錄一份很小的紀錄（只有指令、里程計、馬達狀態，沒有影像，幾 MB 而已）
ros2 bag record -o spin_grass_$(date +%H%M) /cmd_vel /odom /motor_status
```

用手把操作：按住 LB（deadman）→ 左搖桿左右推到底 → 車子開始原地轉 → 轉回記號放開 LB。

地面記號：車頭正前方地上插一根竹筷或放一個顯眼的東西，車頭中央貼一條膠帶當指針。

### 預期看到什麼

- 指令 1.2 rad/s 時，理論上轉一圈 **5.2 秒**。實際一定更慢（輪胎打滑、加速段）。
- 模擬裡現況的設定（前輪幾乎不抵抗側滑）是約 **0.99 rad/s**（8 秒平均，含加速段）。
- 水泥地應該比草地快；兩者差多少本身就是有用的數字。

### 不正常時的判斷（這一項「不正常」本身就是答案，不要硬修）

| 徵狀 | 代表什麼 | 怎麼做 |
|------|---------|--------|
| **完全轉不動**，後輪原地空轉、在草地上刨出痕跡 | 前輪側向阻力大於後輪能給的轉向力 —— 就是模擬 30.4 節的情況 | **立刻放開 LB。** 記下來、錄影、量軸重。**不要加大扭力或速度重試**。這個結果要交回來重新討論規劃策略 |
| 轉得動，但旋轉中心明顯不在車子中央（整台車繞著某一側畫圈） | 左右兩側抓地不對稱，或前後軸重差很多 | 記下圓心大概在哪裡（拍影片就看得出來），量軸重 |
| 轉一圈之後車子離原位超過 0.5 m | 同上 | 用捲尺量離原位多遠，記下來 |
| 順時針與逆時針秒數差超過 20 % | 左右馬達或輪胎不對稱 | 先回 M3 確認兩側在同樣指令下轉速一樣 |

### 結果交回來時要附的東西

1. 記錄表（草地 6 次秒數、水泥 6 次秒數、前軸重、後軸重、整車重）
2. 影片
3. `ros2 bag` 的資料夾（`spin_grass_*`、`spin_concrete_*`）

**里程計在 M4 標定之前是不準的，角速度一律用碼表或影片的秒數算，不要看 `/odom`。**

---

## 車輛幾何：量測之後只改 `vehicle.yaml`（階段 30；階段 38 改寫）

真車是 **XLK X2RS：後兩輪差速驅動 + 前兩個腳輪（可自由轉向）**。
`base_link` = **後輪軸中心**（純差速的瞬時旋轉中心），`base_footprint` 是它在地面的投影。
量測值出來之後**只改一個檔案**，並把該項的 `provenance` 改成 `measured`：

```
src/mowerbot_description/config/vehicle.yaml
```

讀它的唯一入口是 `mowerbot_description/vehicle_geometry.py` 的 `load()`（啟動時檢查
provenance 是否完整、算出所有衍生量、做斷言）。改完重新 `colcon build`，所有讀它的地方會一起更新：

| 讀它的地方 | 用到的項目 |
|-----------|-----------|
| URDF（`car_base.xacro` / `car_wheels.xacro` / `car_engine.xacro` / `car_radar.xacro`） | 車體方塊、質心、後輪、腳輪、引擎遮擋方塊、光達位置、`blade_link`、diff_drive plugin 的輪距與輪徑 |
| `navigation.launch.py` | local_costmap 的 footprint（真實四角）與 `inflation_radius` |
| `mower_control.launch.py` → `mower_manager` / `f2c_server` | 兩種淨空門檻、`blade_width`、`headland_width` |
| `bringup_real.launch.py` → `bridge_node` | `rear_wheel_radius`、`wheel_separation`；幾何閘門 |
| `test/smoke_test.py`、`test/tools/` | 同上各項（讀**安裝後**的那一份） |

衍生值只在 `vehicle_geometry.py` 算一次，不在任何地方寫死：

- 軸距 = `body_length_total` − `rear_wheel_radius` − `front_wheel_radius`（0.875）
- footprint = 前 `wheelbase + front_wheel_radius`（0.975）/ 後 `rear_wheel_radius`（0.155）/ 側 `body_width_total`/2（0.42）
- `rotation_swept_radius` = √(0.975² + 0.42²) = 1.0616（掉頭門檻）；`lateral_half_extent` = 0.42（通過門檻）；
  `costmap_inscribed_radius` = 0.155（只給 Nav2 inflation）
- `headland_width` = ceil((1.0616 + 0.12) / 0.05) × 0.05 = 1.20

**幾何閘門**：`vehicle.yaml` 還有任何 `provisional` 項目時，`bringup_real.launch.py`
（`allow_provisional` 預設 false）第一個節點 `geometry_guard` 就拒絕，整個 launch 以非零結束；
`bridge_node` 以 `read_only:=false` 啟動時也會再查一次。暫定值的清單與量法見
`docs/measurement_worklist.md`。真的要先用暫定值上車時（例如架高測極性），要同時給
`provisional_override_reason:="<理由>"`，理由會寫進 log。

**改完之後要人工重新檢查的東西**：見 `docs/stage38_decisions.md` 的「待重新推導清單」。

---

## M0 N150 裝 Ubuntu 22.04 + ROS 2 Humble + 編譯 workspace

**安全前提**：不接馬達電。

### 指令

1. 用 Ubuntu 22.04 LTS Desktop 的 USB 開機碟裝系統（N150 的 BIOS 開機順序選 USB）。
   **一定要 22.04**：安裝腳本在其他版本上會拒絕執行（ROS 2 Humble 只有 22.04 的套件）。

2. 插上 mowerbot 隨身碟（`deploy/make_usb.sh` 做的那一支），在隨身碟根目錄執行：

```bash
cd /media/$USER/<隨身碟名稱>
bash setup_new_machine.sh
```

   **要打 `bash`**，不要 `./setup_new_machine.sh`：FAT 格式的隨身碟掛載後檔案沒有執行權限，`./` 會 `Permission denied`。

   開頭會問一次 sudo 密碼。它會依序做：ROS 2 apt 來源 → ROS 2 Humble desktop + colcon + rosdep →
   專案需要的其他套件 → 從隨身碟的 git bundle 還原到 `~/mowerbot` → `rosdep install` → `colcon build` →
   寫 `~/.bashrc` → 把使用者加進 `dialout`。隨身碟上有 `debs/` 快取時幾乎不用下載（階段 34 實測 3 分半、51 MiB），
   沒有快取要下載約 0.9 ~ 1.1 GB。可以重複執行，做過的步驟會跳過；失敗時會印出是哪一步、哪一行。

   Fields2Cover（F2C，割草線規劃）用 apt 的 `ros-humble-fields2cover` **2.1.0**，腳本會用 `apt-mark hold`
   鎖住版本，之後 `apt upgrade` 不會把它升級（覆蓋率基準線是用 2.1.0 量的，見 `simulation_results.md` 35 節）。
   **不需要**再從原始碼編 F2C。

3. **登出再登入**（`dialout` 群組才會生效）。

（實車上其實不需要 Gazebo，但腳本會一起裝，這樣 N150 上也能跑 `./run_demo.sh` 與 smoke test 的模擬 Phase。）

### 預期看到什麼

- `colcon build` 最後一行：`Summary: 8 packages finished`，沒有 `failed`
- 下面三個指令都有輸出、沒有錯誤：

```bash
ros2 pkg list | grep mowerbot            # 7 個 mowerbot_* 套件
ros2 pkg executables mowerbot_bridge     # 有 bridge_node 與 teleop_node
python3 test/smoke_test.py --phases=AH   # A（編譯與解析）與 H（loopback bridge）全過，不需要 Gazebo
```

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 腳本在第 1 步拒絕執行 | 不是 Ubuntu 22.04，或用了 `sudo bash setup_new_machine.sh` | 重灌 22.04；用一般使用者執行，不要加 sudo |
| `./setup_new_machine.sh: Permission denied` | 隨身碟是 FAT 格式，檔案沒有執行權限 | 改打 `bash setup_new_machine.sh` |
| 腳本在 `apt-get update` 或 `rosdep update` 失敗 | 網路不通，或實驗室網路擋了 GitHub（`rosdep update` 要連 raw.githubusercontent.com） | 換網路後重跑，做過的步驟會跳過 |
| `mowerbot_planner` 編譯失敗：`Could not find a package configuration file provided by "Fields2Cover"` | `ros-humble-fields2cover` 沒裝 | `dpkg -l ros-humble-fields2cover` 應該是 2.1.0；重跑腳本 |
| `smoke_test.py --phases=A` 的 A5（xacro）失敗 | `vehicle.yaml` 沒被安裝，或 xacro 沒裝 | `colcon build --packages-select mowerbot_description` 再試；`sudo apt install ros-humble-xacro` |

---

## M1 接通 C30D（序列埠、權限、udev 規則、波特率）

**安全前提：車子架高、四輪離地；急停在手邊；刀盤不接電。**

C30D 的通訊協定、波特率、指令格式都**還不知道** —— 一律照 C30D 的文件填，不要猜。
要查的項目清單在 `mowerbot_bridge/drivers/README.md` 的 A ~ E 節（26 項），
其中**沒有就不能開工**的是 A3（協定格式）、B1（速度指令單位）、C1（每轉 tick 數）。

### 指令

1. 插上 C30D 的 USB（或 USB 轉 RS-232/485），看它出現在哪裡：

```bash
sudo dmesg | tail -20                    # 找 "ttyUSB0" 或 "ttyACM0"
ls -l /dev/serial/by-id/                 # 會有一個帶廠商名稱的長檔名
```

2. 權限：把自己加進 `dialout` 群組（**要登出再登入才生效**）：

```bash
sudo usermod -aG dialout $USER
groups                                   # 登入後確認有 dialout
```

3. udev 規則：把裝置固定成 `/dev/mowerbot_base`，重開機或換 USB 孔都不會跑掉。
   規則檔已經寫好（`deploy/99-mowerbot.rules`），但裡面的 VID:PID 與序號是照輪趣的慣例填的，
   **先對照 `deploy/hw_probe.sh` 第 3 節的「VID:PID」「序號」**，不一樣就改規則檔：

```bash
bash ~/mowerbot/deploy/hw_probe.sh                           # 看第 3 節 C30D 那一個序列埠
nano ~/mowerbot/deploy/99-mowerbot.rules                     # 需要時改 idVendor / idProduct / serial
sudo cp ~/mowerbot/deploy/99-mowerbot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/mowerbot_base                 # 應該指向 ttyUSB0（或 ACM0）
```

4. 先**不透過 ROS**，用最土法的方式讓一顆輪子轉起來（證明「電腦講的話板子聽得懂」）：

```bash
python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/mowerbot_base', <C30D 文件上的波特率>, timeout=0.5)
s.write(<C30D 文件上「左輪最低速正轉」的指令位元組>)
time.sleep(1.0)
print('回應:', s.read(64))
s.write(<C30D 文件上「停止」的指令位元組>)
s.close()
EOF
```

5. 土法指令確認可行之後，照 `drivers/README.md` 新增 `drivers/c30d.py`（實作 `MotorDriver` 的五個方法），
   在 `drivers/__init__.py` 的 `DRIVERS` 加一行，然後不透過 ROS 測它：

```bash
python3 -c "
from mowerbot_bridge.drivers.c30d import <類別名稱>
d = <類別名稱>(ticks_per_rev=<C1 的值>)
d.connect()
print('encoders:', d.read_encoders())
d.set_wheel_velocities(0.5, 0.0)   # 只轉左輪，0.5 rad/s
import time; time.sleep(2)
d.stop()
print('encoders:', d.read_encoders())
d.disconnect()
"
```

### 預期看到什麼

- 第 4 步：左輪轉約 1 秒後停；`回應:` 後面不是空的 `b''`
- 第 5 步：左輪轉、右輪不轉；兩次 `encoders` 的左輪數值有變、右輪沒變；`stop()` 之後輪子停住
- 拔掉 USB 再插回去，`ls -l /dev/mowerbot_base` 仍然存在

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| `Permission denied: '/dev/mowerbot_base'` | 還沒登出再登入，`dialout` 沒生效 | 登出再登入（規則檔給的是 `dialout` 群組讀寫，刻意不開 0666）；`groups` 裡要有 `dialout` |
| `/dev/mowerbot_base` 不存在 | 規則檔的 VID:PID 或序號跟這塊板子不符 | 對照 `hw_probe.sh` 第 3 節改規則檔，再 reload + trigger |
| 送了指令，`回應: b''`，輪子不動 | 波特率錯、TX/RX 接反、或板子要先送「致能（enable）」指令（`drivers/README.md` D5） | 依序確認這三項；用 C30D 附的原廠工具先確認板子本身能動 |
| 回應是亂碼 | 波特率、資料位元、同位、停止位元其中一項不對（A2） | 對照文件逐項改 |

---

## M2 填入實機量測的七個數字

**安全前提**：不接馬達電（量尺寸時車子不會動）。

### 要量的七個數字

`vehicle.yaml` 裡跟物理有關的七項（`body_length / width / height` 只是模擬畫面用，可以不量）：

| # | `vehicle.yaml` 項目 | 怎麼量 | 單位 |
|---|------------------|--------|------|
| 1 | `wheel_radius` | 後輪（驅動輪）落地時，輪軸中心到地面的高度。**車子要壓在地上量**（輪胎受壓會變扁） | m |
| 2 | `wheel_separation` | 左右後輪**輪胎中心線**之間的距離（量兩輪內緣距離 + 一個輪寬） | m |
| 3 | `wheelbase` | 前軸中心到後軸中心的距離 | m |
| 4 | `footprint_length` | 車子最前端到最後端（含保險桿、任何突出物） | m |
| 5 | `footprint_width` | 車子最左到最右（輪外緣到輪外緣；車殼比輪子寬就量車殼） | m |
| 6 | `blade_width` | 刀盤實際割到的寬度（刀尖旋轉直徑） | m |
| 7 | `mass` | 整車重（M-1 的前軸重 + 後軸重） | kg |

另外**順便量輪寬**（後輪胎寬，m）：它還在 `car_wheels.xacro` 裡（0.1），
`footprint_width` 不能比「`wheel_separation` + 輪寬」窄，啟動時有斷言會檢查。

每一項**量三次取中間值**，捲尺讀到 mm。

### 指令

```bash
nano ~/mowerbot/src/mowerbot_description/config/vehicle.yaml
# 改數字，並把後面的「暫定值」改成「量測值」。只改這個檔案。
cd ~/mowerbot && colcon build --packages-select mowerbot_description mowerbot_bringup
python3 test/smoke_test.py --phases=A      # 至少 A5（xacro 解析）要過
```

**在 M-1（原地旋轉量測）的結果交回來之前**，改完的數字先不要拿去跑模擬的調校 ——
程式端的凍結條件見 `CLAUDE.md`。

### 預期看到什麼

- `colcon build` 沒有錯
- 啟動 `bringup_real.launch.py` 時**沒有**出現下面兩則訊息之一（出現代表數字彼此矛盾）：
  - `車輛幾何變更後 headland 未同步更新`
  - `vehicle.yaml 的 footprint_width = ... 比後輪外緣寬度還窄`

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 啟動時出現「headland 未同步更新」 | 量到的 footprint 比暫定值大，外接半徑變大，地頭 0.70 m 不夠掉頭 | **這不是量錯**，是真實的限制。記下訊息裡的數字交回來，由使用者決定地頭寬度；不要自己改 headland |
| 啟動時出現「footprint_width 比後輪外緣寬度還窄」 | `footprint_width` 量的是車殼、但輪子凸出車殼；或輪距量成內緣距離 | 重量 #2 與 #5 |
| `xacro` 解析錯誤 | yaml 格式壞了（冒號後面少空白、用了全形字元） | 對照原本的格式，數字後面要有空白再接 `#` |

---

## M3 極性驗證（左右輪、編碼器方向）—— 必須架高

**安全前提：車子架高、四輪離地；急停在手邊；旁邊有第二個人；刀盤不接電。**

### 重要：這一步只啟動 `bridge_node`，不要用 `bringup_real.launch.py`

`bringup_real.launch.py` 會一起起 `mower_manager`。manager 在手動模式下只要 0.5 秒沒收到手把指令，
就以 **20 Hz 對 `/cmd_vel` 發零速度**（watchdog）。這時候用 `ros2 topic pub /cmd_vel` 手動送指令，
會跟 manager 的零速度互搶，輪子會一頓一頓的，判斷不出方向。
所以極性驗證只起 `bridge_node`（它自己的 0.5 秒 watchdog 仍然有效）。

### 指令

```bash
# 終端機 1：只起 bridge_node。輪半徑與輪距照 vehicle.yaml 填（M2 的值）
ros2 run mowerbot_bridge bridge_node --ros-args \
     -p driver_type:=c30d -p encoder_ticks_per_rev:=<C1 的值> \
     -p wheel_radius:=<vehicle.yaml 的值> -p wheel_separation:=<vehicle.yaml 的值> \
     -p use_sim_time:=false

# 終端機 2：編碼器讀值
ros2 topic echo /motor_status
```

**(a) 輪子方向**（每一條送 3 秒，Ctrl-C 停；bridge 會在 0.5 秒內把輪子停下）：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"      # 前進
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1}}"     # 後退
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.3}}"     # 原地左轉
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.3}}"    # 原地右轉
```

**(b) 編碼器方向與 tick 數**（不送指令，用手轉）：

1. 左後輪上貼一條膠帶，對準車體上的一個固定點
2. 記下 `/motor_status` 左輪的 tick 值
3. **用手**把左後輪往「讓車子前進」的方向轉整整一圈，回到膠帶對齊
4. 再記一次 tick 值；右輪同樣做一次

### 預期看到什麼

| 指令 | 應該看到 |
|------|---------|
| `linear.x = 0.1` | 兩個後輪都往「使車輛前進」的方向轉 |
| `linear.x = -0.1` | 兩個後輪都反轉 |
| `angular.z = 0.3`（左轉） | 右輪正轉、左輪反轉 |
| `angular.z = -0.3`（右轉） | 左輪正轉、右輪反轉 |
| 手往前轉一圈 | 該輪 tick **增加**，增加量 ≈ `encoder_ticks_per_rev`（差幾個 tick 正常，差一倍不正常） |

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 某一輪方向相反 | 馬達接線或驅動板通道定義相反 | **改參數，不要改接線**：`ros2 param set /mower_bridge invert_left true`（或 `invert_right`），確認後寫進 `bringup_real.launch.py` |
| 左右輪對調（下左轉指令，左輪正轉） | C30D 的通道 1/2 與左右對應相反 | 在 `drivers/c30d.py` 裡把通道對調（這是驅動層的事，不是 invert） |
| tick 增加量是預期的 2 倍、4 倍或 1/減速比 | 四倍頻重複算或沒算、減速比沒乘進去 | 修正 `encoder_ticks_per_rev`；見 `drivers/README.md` C1、E1 |

---

## M6 實體手把驗證（按住 / 放開 / 拔接收器 / 急停鈕）

**安全前提：車子架高、四輪離地；急停在手邊；旁邊有第二個人；刀盤不接電。**
M3 必須先過。

按鍵對應（`mowerbot_bridge/config/joystick.yaml`）：LB = deadman（index 4）、RB = 急停（index 5）、
A = 建圖模式、B = 自動割草、X = 手動；左搖桿上下 = 前後（全推 0.7 m/s）、左右 = 轉向（全推 1.2 rad/s）。

### 指令

```bash
# 終端機 1：這次用完整的實車啟動（manager 的仲裁要一起驗）
ros2 launch mowerbot_bringup bringup_real.launch.py \
     driver_type:=c30d encoder_ticks_per_rev:=<C1 的值>

# 終端機 2：看實際送到底盤的速度
ros2 topic echo /cmd_vel --field linear.x

# 終端機 3：看手把狀態
ros2 topic echo /joy_status
```

四項測試：

| # | 動作 | 預期 |
|---|------|------|
| 1 **按住** | 按住 LB，左搖桿往前推一半 | 兩後輪前轉；`/cmd_vel` 約 0.35；`/joy_status` 的 `deadman_held: true` |
| 2 **放開** | 搖桿維持推著，放開 LB | 輪子**立刻**停；`/cmd_vel` 變 0.0 |
| 3 **拔接收器** | 按住 LB 推搖桿讓輪子轉，然後**拔掉手把的 USB 接收器** | 輪子在 **0.5 秒內**停（manager watchdog）；`/joy_status` 的 `connected: false`、`last_msg_age` 持續變大 |
| 4 **急停鈕** | 按住 LB 推搖桿讓輪子轉，按 RB | 輪子**立刻**停；終端機 1 印 `觸發緊急停止,所有功能已鎖定`；之後再按 LB 推搖桿，輪子**不會動**；要按住 LB + X 切回手動模式才能再動 |

另外做一次 bridge 層的 watchdog：按住 LB 讓輪子轉，然後在終端機 1 **Ctrl-C** 停掉整個 launch，
輪子要停（這時 `bridge_node` 也一起被停掉，靠 C30D 自己的逾時保護；見 `drivers/README.md` B5）。
**如果 C30D 沒有逾時保護，這一項輪子會繼續轉 —— 這是要記下來的安全缺口**，要在 C30D 的設定或接線上處理（硬體急停）。

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 按住 LB 推搖桿輪子不動，`/joy_status` 是 `connected: false` | `joy_node` 沒抓到手把（`device_id` 不對或接收器沒配對） | `ls /dev/input/js*`；`ros2 run joy joy_enumerate_devices`；改 `joystick.yaml` 的 `device_id` |
| 不按 LB 輪子也會動 | 按鍵 index 與這支手把不一致（不同廠牌的 LB 編號不同） | `ros2 topic echo /joy` 按 LB 看是哪一個 index 變 1，改 `joystick.yaml`。**這一項沒過絕對不能落地** |
| 拔接收器之後輪子繼續轉超過 1 秒 | 手把驅動在斷線時沒有停止發布，或持續發舊值 | `ros2 topic hz /joy` 拔掉時看頻率是否歸零；記下來交回來，不要落地 |

---

## M4 里程計標定（直線 1 m、原地 360°）

**安全前提：落地；急停在手邊；旁邊有第二個人；直線方向前方 3 m、周圍 2 m 淨空。M-1 先做完。**

### 重要：這一步也只啟動 `bridge_node`

`test/tools/calibrate_odometry.py` 直接對 `/cmd_vel` 發指令，
跟 M3 同樣的理由（manager 的 watchdog 零速度會互搶），**不要同時起 `bringup_real.launch.py`**。
（階段 33 起工具自己也會檢查：啟動時聽到 `/mower_status` 就拒絕執行，印出
「偵測到 mower_manager 正在執行……請單獨啟動 bridge_node 後再執行。」）

### 指令

```bash
# 終端機 1：同 M3 的 bridge_node 指令（這次 invert_* 帶上 M3 確認過的值）
ros2 run mowerbot_bridge bridge_node --ros-args \
     -p driver_type:=c30d -p encoder_ticks_per_rev:=<C1 的值> \
     -p wheel_radius:=<vehicle.yaml> -p wheel_separation:=<vehicle.yaml> \
     -p invert_left:=<M3> -p invert_right:=<M3> -p use_sim_time:=false

# 終端機 2：互動式標定，會問你「實際量到多少」
cd ~/mowerbot
python3 test/tools/calibrate_odometry.py --only=a --distance=1.0 --speed=0.15     # 直線 1 m
python3 test/tools/calibrate_odometry.py --only=b --turns=1 --turn-speed=0.5      # 原地 360°
```

地面準備：
- **直線**：地上貼一條起點膠帶，車子後輪軸對齊它；跑完用捲尺量後輪軸離膠帶多遠
- **原地 360°**：車頭正前方地上放一個記號，車頭中央貼膠帶；轉完看膠帶離記號差幾度
  （拍一張正上方的照片最準）

### 修正係數怎麼算

腳本會直接印出來，公式是：

- `wheel_radius_correction` = 目前值 × （捲尺量到的實際距離 ÷ `/odom` 報告的距離）
- `wheel_separation_correction` = 目前值 × （`/odom` 報告的角度 ÷ 實際轉的角度）

例：`/odom` 報轉了 360°，但實際只轉了 250° → 係數 × 1.44
（滑移轉向時輪胎橫向滑動，有效輪距比幾何值大，這是正常的）。

填進 `bringup_real.launch.py` 的 `wheel_radius_correction` / `wheel_separation_correction`，
然後**重跑一次**兩項。

### 預期看到什麼

- 直線的係數落在 **0.9 ~ 1.1**
- 原地旋轉的係數落在 **1.2 ~ 2.0**（滑移轉向的正常範圍）
- 填入之後重跑，兩項係數都接近 **1.0**（0.97 ~ 1.03）
- 1 m、1 圈的解析度有限，結果不穩的話改成 `--distance=3.0`、`--turns=3` 再做一次

腳本本身在 Gazebo 乾跑驗證過（2026-09-21，`--auto`，對著完美里程計算出係數 1.0000）——
那只證明流程沒寫錯，不代表車子準。

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 直線係數差很多（例如 0.5 或 2.0） | `encoder_ticks_per_rev` 錯 | 回 M3 (b) 重數 tick |
| 直線行駛腳本警告「轉了超過 5 度」 | 左右輪在同樣指令下轉速不同 | 先解決不對稱（C30D 的左右通道增益、輪胎氣壓）再標定 |
| 原地旋轉腳本一直不停、最後逾時 | 車子轉不動或轉得非常慢 —— M-1 的情況 | 放開急停之外什麼都不要做，回頭看 M-1 的結果 |

---

## M5 光達安裝位置量測與 TF 填寫

**安全前提**：不接馬達電。

### 要量的東西

以 `base_link`（**後輪軸中心**，階段 38 起）為原點，右手座標：x 往車頭、y 往左、z 往上。
**捲尺注意**：現場捲尺同時印有公分與台寸（1 台寸 ≈ 3.03 cm），大數字是台寸，讀之前確認單位。

| 量什麼 | 怎麼量 | 填到哪裡 |
|--------|--------|---------|
| 光達掃描中心的 x | 鉛錘從光達旋轉中心垂到地面做記號，捲尺量到後輪軸中心在地面的投影（車頭方向為正） | `vehicle.yaml` 的 `lidar_x` |
| 光達掃描中心的 y | 同上，往左為正（目前模型假設 0） | `car_radar.xacro` 的 `radar_joint_y` |
| 光達掃描平面離地高度 | **從側面**拍或量：地面 → 光達開口中線（不是外殼頂）。現有兩次都是俯視，量不到高度 | `vehicle.yaml` 的 `lidar_z_ground`（直接填離地高度，base_link 相對高度由載入器減 `rear_wheel_radius`） |
| 光達的 0° 朝向 | 看光達外殼上的箭頭或纜線出口，對照型號文件的 0° 方向 | `car_radar.xacro` 的 joint `rpy` |
| **引擎 / 車體遮擋** | 光達照常掃描、車停在空曠處，`python3 test/tools/scan_fov.py 4` 直接列出被遮擋的角度範圍 | 現場目視「未被遮擋」；跑一次確認，結果交回來 |

目前 `lidar_x = 0.30 ± 0.08`（measured_coarse，**量測基準點未確認**）、`lidar_z_ground = 0.55`（provisional，照片目測）。
實機由 `allow_provisional` 閘門擋住（因為 lidar_z_ground 是 provisional），模擬放行。

### 指令

```bash
nano ~/mowerbot/src/mowerbot_description/config/vehicle.yaml       # 改 lidar_x / lidar_z_ground，provenance 改 measured
cd ~/mowerbot && colcon build --packages-select mowerbot_description
ros2 launch mowerbot_description robot_state_publisher.launch.py use_sim_time:=false &
ros2 run tf2_ros tf2_echo base_link radar                          # 印出來的平移要等於量到的值
```

光達驅動節點填在 `bringup_real.launch.py` 的 TODO（型號確定之後），
`frame_id` 必須是 `radar`，裝置用 udev 固定成 `/dev/mowerbot_lidar`（在 `deploy/99-mowerbot.rules` 的光達 TODO 那一行填好，做法同 M1 第 3 步）。

### 預期看到什麼

- `tf2_echo base_link radar` 的平移與量到的值差 < 1 cm
- 光達驅動起來之後 `ros2 topic hz /scan` 有穩定頻率
- RViz 裡（Fixed Frame 設 `base_link`）車頭前方的牆出現在車頭前方，不是側面或後面

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| RViz 裡牆面在車子側面或後面 | 光達 0° 方向與車頭不一致 | 在 xacro 的 radar joint 加 yaw（`rpy="0 0 <角度>"`） |
| 車子原地轉時掃到的牆也跟著「甩」 | 光達的 x/y 偏移量錯，旋轉時掃描中心不在填的位置 | 重量 x、y |
| `/scan` 裡有一圈固定的近距離點 | 光達掃到車體自己（引擎、支架、相機）。X2RS 的引擎就在光達正前方、高度相近，**預期會發生** | 記錄被遮擋的角度範圍交回來（報告 38 節有模擬預估）；要不要抬高光達或在驅動設定遮掉那段角度由使用者決定 |

---

## M7 落地低速 → SLAM 建圖 → 自動割草

**安全前提：落地；急停在手邊；旁邊有第二個人；場地淨空、沒有人或寵物在裡面。
M0 ~ M6 與 M-1 全部完成、M-1 的結果已經交回來並確認規劃策略可行。**

### 7-1 落地低速

```bash
ros2 launch mowerbot_bringup bringup_real.launch.py \
     driver_type:=c30d encoder_ticks_per_rev:=<C1 的值>
```

手動模式（X），按住 LB，搖桿只推 1/4，前進、後退、左右轉各走幾公尺。
隨時準備放開 LB 或按 RB。

**預期**：車子直線走時不偏；放開 LB 立刻停；轉向方向與搖桿一致。

### 7-2 SLAM 建圖

```bash
# bringup_real 維持開著（它已經包含建圖模式的 slam_toolbox）
ros2 topic hz /scan                                # 光達有資料
ros2 run tf2_ros tf2_echo odom base_footprint      # 里程計 TF 有在動
ros2 run tf2_ros tf2_echo base_link radar          # 光達外參對得上 M5
rviz2 -d ~/mowerbot/src/mowerbot_description/rviz/mowerbot.rviz
```

按住 LB + A 切到建圖模式，慢慢（搖桿推 1/3 以下）沿著場地邊界繞一圈，回到起點。然後存圖：

```bash
ros2 run mowerbot_bringup save_map.sh <地圖名稱>
```

**預期**：RViz 裡的地圖沒有重影或糊掉；繞回起點時牆面與第一次掃到的重疊；
`save_map.sh` 印出四個檔案（`.posegraph .data .pgm .yaml`），`.yaml` 的 `free_thresh` 是 0.10。

### 7-3 自動割草（刀盤先不接電）

第一次不要直接跑整個覆蓋任務：

1. 先確認 Nav2 起得來：

```bash
ros2 launch mowerbot_bringup navigation.launch.py use_sim_time:=false
ros2 lifecycle get /controller_server              # 要是 active
```

2. 按住 LB + B 切到自動割草（mode 1）。manager 會自己從地圖萃取邊界、呼叫 F2C 規劃、開始執行。
3. **第一趟刀盤不接電**，人跟在車旁邊，手放在急停上。
4. 第一趟正常跑完，才在第二趟接上刀盤。

**預期**：`mower_manager` 的 log 依序出現 `邊界提取成功`、`📋 佇列`、`➡️ 送出任務`、`✅ ... 完成`；
車子沿割草線走，每條線結束時原地掉頭（掉頭的可行性就是 M-1 在驗的東西）。

**這裡的數字一定會比模擬差。**模擬的里程計是完美的（沒有打滑、沒有漂移），
DWB 的權重、`xy_goal_tolerance`、跑道長度全部要在實車上重新實測決定 —— **不要照抄模擬的結果**。

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| 建圖時地圖轉一轉就糊掉 | 九成是 M4 的 `wheel_separation_correction` 還不夠準 | 回 M4 用 `--turns=3` 重做原地旋轉 |
| `controller_server` 卡在 inactive | 已知的 Nav2 生命週期競態（`simulation_results.md` 7.12 節，模擬裡也會發生） | 停掉 navigation.launch.py 重起一次；記下發生次數 |
| 每條割草線結尾都 `Failed to make progress`，車子在掉頭處原地打轉或不動 | 原地掉頭做不到或太慢 —— M-1 的情況 | 按急停。這不是調參數能解決的，交回來重新討論規劃策略 |

---

## 附錄：需要從 C30D 文件查到的資訊

完整清單在 `mowerbot_bridge/drivers/README.md`（A 通訊 / B 速度指令 /
C 編碼器 / D 狀態與安全 / E 機械，共 26 項）。
拿著那張表去問供應商或查手冊就好。

其中**三項沒有就不能開工**：

| 編號 | 項目 | 沒有的話會怎樣 |
|------|------|--------------|
| A3 | 協定格式 | 寫不出 driver |
| B1 | 速度指令的單位 | 車速差一個數量級，很危險 |
| C1 | 每轉的 tick 數 | `bridge_node` 會拒絕啟動（刻意的） |

另外**三項與安全直接相關**，第一次落地前一定要問清楚：

| 編號 | 項目 |
|------|------|
| B5 | 板子自己有沒有指令逾時保護（M6 的 Ctrl-C 測試會用到） |
| D3 | 有沒有硬體急停輸入腳位 |
| D6 | 斷電或斷線時馬達是自由滑行還是煞車 |
