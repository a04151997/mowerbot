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
| C30D 驅動實作（`drivers/` 下） | **空白**，要等 C30D 的通訊協定文件 |
| 光達驅動 | **空白**，要等光達型號 |
| 車輛幾何 `vehicle.yaml` | 全部是**暫定值**，等 M2 的實測 |

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

## 車輛幾何：量測之後只改 `vehicle.yaml`（階段 30）

真車是**後兩輪差速驅動 + 前兩輪固定方向從動輪**（滑移轉向）。
輪徑、輪距、軸距、footprint 的量測值出來之後，**只改一個檔案**：

```
src/mowerbot_description/config/vehicle.yaml
```

改完重新 `colcon build`（至少 `mowerbot_description`），所有讀它的地方就會一起更新：

| 讀它的地方 | 用到的項目 |
|-----------|-----------|
| URDF（`car_base.xacro` / `car_wheels.xacro`） | 車體方塊 `body_*`、`mass`、輪半徑、輪子的 y（±`wheel_separation`/2）與 z（剛好著地）、diff_drive plugin 的輪距與輪徑 |
| `navigation.launch.py` | local_costmap 的 footprint（由 `footprint_length` / `footprint_width` 算，`nav2_params.yaml` 裡已經沒有 footprint） |
| `mower_control.launch.py` → `mower_manager` | 淨空檢查的內切 / 外接半徑、`blade_width` |
| `bringup_real.launch.py` → `bridge_node` | `wheel_radius`、`wheel_separation` |
| `test/smoke_test.py`、`test/tools/` | 同上各項（讀**安裝後**的那一份，與節點拿到的一致） |

衍生值一律用算的，不在任何地方寫死：

- 內切半徑 = `footprint_width` / 2
- 外接半徑 = sqrt((`footprint_length`/2)² + (`footprint_width`/2)²)

`mower_manager` 與 `bridge_node` 的這幾個參數**沒有預設值**：不經過 launch 檔直接
`ros2 run` 而沒有帶 `-p` 的話，節點會在啟動時以 `ParameterUninitializedException` 失敗，
而不是安靜地用一個舊數字跑下去。

每一項後面標了「量測值 / 暫定值」，量完一項就改標記。

**改完之後要人工重新檢查的東西**（它們是「由幾何推導、但含有判斷」的值，
沒有辦法自動跟著算，也不該自動跟著算）：

| 值 | 在哪裡 | 為什麼要重看 |
|----|--------|-------------|
| `headland_width` 0.70 | `mower_control.launch.py`、`f2c_server.cpp` | = 外接半徑 + xy_goal_tolerance 0.10 再**進位**。外接半徑變了就要重算、重新決定怎麼進位。**有啟動斷言**（階段 31）：headland_width < 外接半徑 + general_goal_checker 的 xy_goal_tolerance 時，`mower_control.launch.py` 在啟動任何節點前以「車輛幾何變更後 headland 未同步更新」失敗 |
| `acc_lim_x` 0.5 / `acc_lim_theta` 1.5 | `nav2_params.yaml` | 由 max_wheel_acceleration × 輪半徑、÷ 輪距推導後**保守取整**。屬於速度 / 加速度上限，由使用者決定 |
| `inflation_radius` 0.45 | `nav2_params.yaml` | 依內切半徑選的，內切半徑變了要重看 |
| 前後輪 x = ±0.35 | `car_wheels.xacro` | `wheelbase` 還是 TBD，URDF 暫時沒有讀它。量到之後要改成讀 `vehicle.yaml` |
| 輪寬 0.1 | `car_wheels.xacro` | 不在 `vehicle.yaml` 裡。目前 `footprint_width` 0.68 = 輪距 0.58 + 輪寬 0.1，兩者是分開填的。**有啟動斷言**（階段 31）：`footprint_width` < `wheel_separation` + 輪寬時，`robot_state_publisher.launch.py` 失敗（footprint 沒包住輪子）。比較寬不會觸發 —— 車殼比輪子寬時本來就該比較寬 |
| Phase O 夾具的 `O_FIXTURE_MARGIN` | `smoke_test.py` | 啟動斷言會以「夾具幾何問題」失敗來提醒，不會安靜地錯 |

---

## M0 N150 裝 Ubuntu 22.04 + ROS 2 Humble + 編譯 workspace

**安全前提**：不接馬達電。

### 指令

1. 用 Ubuntu 22.04 LTS Desktop 的 USB 開機碟裝系統（N150 的 BIOS 開機順序選 USB）。
   裝完之後：

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl software-properties-common build-essential cmake
```

2. 裝 ROS 2 Humble（官方 apt 來源）：

```bash
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
     ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joy \
     ros-humble-xacro ros-humble-tf2-tools python3-serial python3-opencv python3-pyqt5
```

（實車上**不需要 Gazebo**。要在 N150 上跑 `test/smoke_test.py` 的模擬 Phase 才需要
`ros-humble-gazebo-ros-pkgs`，那不是上線必要條件。）

3. 裝 Fields2Cover（F2C，割草線規劃用；開發機上用的是 **2.0.0**，裝在 `/usr/local`）。
   下面的相依套件清單是參考，**以 Fields2Cover 該版本 README 列的為準**（N150 上第一次編譯可能要 20 分鐘以上）：

```bash
sudo apt install -y libgeos-dev libgdal-dev libeigen3-dev libtbb-dev libboost-all-dev \
     libtinyxml2-dev nlohmann-json3-dev swig python3-matplotlib
cd ~ && git clone -b v2.0.0 https://github.com/Fields2Cover/Fields2Cover.git
cd Fields2Cover && mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON=OFF .. && make -j$(nproc) && sudo make install
sudo ldconfig
```

4. 拿 workspace 並編譯：

```bash
cd ~ && git clone <workspace 的 git 位址> mowerbot
cd ~/mowerbot
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
echo 'source /opt/ros/humble/setup.bash'  >> ~/.bashrc
echo 'source ~/mowerbot/install/setup.bash' >> ~/.bashrc
```

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
| `mowerbot_planner` 編譯失敗：`Could not find a package configuration file provided by "Fields2Cover"` | F2C 沒裝好或沒 `sudo make install` | 回第 3 步；確認 `/usr/local/lib/cmake/Fields2Cover/` 存在 |
| `rosdep install` 說某個 key 找不到（例如 `fields2cover`） | F2C 不在 rosdep 資料庫裡，是從原始碼裝的 | 正常，`-r` 會跳過；只要第 3 步裝好就能編 |
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

3. udev 規則：把裝置固定成 `/dev/c30d`，重開機或換 USB 孔都不會跑掉：

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m3 -E 'idVendor|idProduct|serial'
# 把上面看到的三個值填進去：
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="<idVendor>", ATTRS{idProduct}=="<idProduct>", ATTRS{serial}=="<serial>", SYMLINK+="c30d", MODE="0666"' \
     | sudo tee /etc/udev/rules.d/99-c30d.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/c30d                          # 應該指向 ttyUSB0（或 ACM0）
```

4. 先**不透過 ROS**，用最土法的方式讓一顆輪子轉起來（證明「電腦講的話板子聽得懂」）：

```bash
python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/c30d', <C30D 文件上的波特率>, timeout=0.5)
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
- 拔掉 USB 再插回去，`ls -l /dev/c30d` 仍然存在

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| `Permission denied: '/dev/ttyUSB0'` | 還沒登出再登入，`dialout` 沒生效 | 登出再登入；或暫時用 udev 規則裡的 `MODE="0666"` |
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

以 `base_link`（車體方塊的中心）為原點，右手座標：x 往車頭、y 往左、z 往上。

| 量什麼 | 怎麼量 |
|--------|--------|
| 光達掃描中心的 x | 從車體中心沿車頭方向量到光達的旋轉中心 |
| 光達掃描中心的 y | 往左為正 |
| 光達掃描中心的 z | 從 `base_link` 的高度（車體方塊中心）往上量到光達的掃描平面 |
| 光達的 0° 朝向 | 看光達外殼上的箭頭或纜線出口，對照型號文件的 0° 方向 |

### 指令

目前 URDF 裡雷達在 `car_radar.xacro`：`radar_joint_x = 0.0`、`radar_joint_y = 0.0`、`radar_joint_z = 0.45`。
**這是 URDF 的修改 —— 在 M-1 的結果交回來、凍結解除、使用者同意之前不要改**（見 `CLAUDE.md`）。
量好的數字先記錄下來交回來。

凍結解除之後：

```bash
nano ~/mowerbot/src/mowerbot_description/urdf/car_radar.xacro      # 改 radar_joint_x / y / z
cd ~/mowerbot && colcon build --packages-select mowerbot_description
ros2 launch mowerbot_description robot_state_publisher.launch.py use_sim_time:=false &
ros2 run tf2_ros tf2_echo base_link radar                          # 印出來的平移要等於量到的值
```

光達驅動節點填在 `bringup_real.launch.py` 的 TODO（型號確定之後），
`frame_id` 必須是 `radar`，裝置用 udev 固定成 `/dev/lidar`（做法同 M1 第 3 步）。

### 預期看到什麼

- `tf2_echo base_link radar` 的平移與量到的值差 < 1 cm
- 光達驅動起來之後 `ros2 topic hz /scan` 有穩定頻率
- RViz 裡（Fixed Frame 設 `base_link`）車頭前方的牆出現在車頭前方，不是側面或後面

### 不正常時

| 徵狀 | 原因 | 處理 |
|------|------|------|
| RViz 裡牆面在車子側面或後面 | 光達 0° 方向與車頭不一致 | 在 xacro 的 radar joint 加 yaw（`rpy="0 0 <角度>"`） |
| 車子原地轉時掃到的牆也跟著「甩」 | 光達的 x/y 偏移量錯，旋轉時掃描中心不在填的位置 | 重量 x、y |
| `/scan` 裡有一圈固定的近距離點 | 光達掃到車體自己（支架、相機） | 抬高光達或在驅動設定裡遮掉那個角度範圍 |

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
