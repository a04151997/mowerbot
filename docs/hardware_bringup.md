# 實車第一次接上時的步驟清單

**相關文件**：模擬端的驗證數據在 [`simulation_results.md`](simulation_results.md)，
系統架構與資料流圖在 [`architecture.md`](architecture.md)，
驅動板要查的資訊清單在 [`../src/mowerbot_bridge/mowerbot_bridge/drivers/README.md`](../src/mowerbot_bridge/mowerbot_bridge/drivers/README.md)。

這份文件是給「拿到驅動板、準備第一次讓 74 kg 的車子動起來」的自己看的。
每一步都寫了**怎麼確認這一步成功了**，沒有確認方式的步驟不算做完。

目前的狀態（寫這份文件時）：

| 項目 | 狀態 |
|------|------|
| 模擬端（SLAM / F2C / Nav2 / 安全機制） | 完成，28 項自動化測試全過，見 `simulation_results.md` |
| `bridge_node`（ROS ↔ 驅動板） | 骨架完成，用 loopback 假驅動測過（Phase H） |
| 里程計數學 | 完成，17 項單元測試全過（`mowerbot_bridge/test/test_odometry.py`） |
| 真實驅動實作 | **空白**，要等驅動板資訊 |
| 雷達驅動 | **空白**，要等雷達型號 |

---

## 安全：第一次通電測試時車子必須架高、輪子離地

**這不是客套話。** 74 kg 的機器，輪半徑 0.17 m、最高速 0.7 m/s。
接線接反、編碼器方向相反、或是速度單位換算差一個數量級，
車子會瞬間往某個方向衝出去，而你的手正放在它旁邊。

第一次通電到「步驟 5 全部通過」之前，車子一律架高、四輪離地。
另外：

- 電池先接一顆保險絲，或用可限流的電源供應器
- 急停開關放在手邊，不是放在車上
- 旁邊至少有另一個人
- 測試場地淨空半徑 2 公尺以上（步驟 6 之後才需要）

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
| `headland_width` 0.70 | `mower_control.launch.py`、`f2c_server.cpp` | = 外接半徑 + xy_goal_tolerance 0.10 再**進位**。外接半徑變了就要重算、重新決定怎麼進位 |
| `acc_lim_x` 0.5 / `acc_lim_theta` 1.5 | `nav2_params.yaml` | 由 max_wheel_acceleration × 輪半徑、÷ 輪距推導後**保守取整**。屬於速度 / 加速度上限，由使用者決定 |
| `inflation_radius` 0.45 | `nav2_params.yaml` | 依內切半徑選的，內切半徑變了要重看 |
| 前後輪 x = ±0.35 | `car_wheels.xacro` | `wheelbase` 還是 TBD，URDF 暫時沒有讀它。量到之後要改成讀 `vehicle.yaml` |
| 輪寬 0.1 | `car_wheels.xacro` | 不在 `vehicle.yaml` 裡。目前 `footprint_width` 0.68 = 輪距 0.58 + 輪寬 0.1，兩者是分開填的，改其中一個要確認另一個還對得上 |
| Phase O 夾具的 `O_FIXTURE_MARGIN` | `smoke_test.py` | 啟動斷言會以「夾具幾何問題」失敗來提醒，不會安靜地錯 |

---

## 步驟 1：確認驅動板通訊（先不透過 ROS）

**目標：用最土法的方式讓一顆輪子轉起來，證明「電腦講的話板子聽得懂」。**

先不要碰 ROS。用 `screen`、`minicom`、或十行以內的 Python 腳本，
照板子文件送一條「讓左輪以最低速轉」的指令。

需要先查到的資訊見 `mowerbot_bridge/drivers/README.md` 的 A 節與 B 節。

**怎麼確認成功：**

- 輪子真的轉了，而且送停止指令會停
- 讀得回板子的回應（不是只有送出去沒有回應）
- 拔掉線再插回去，同樣的腳本還能動（裝置路徑沒有跑掉）

**卡住的話：** 先確認鮑率與接線（TX/RX 有沒有反），
這兩個是最常見的。不要急著寫 ROS 節點。

---

## 步驟 2：實作 `drivers/` 下的真實驅動

照 `mowerbot_bridge/drivers/README.md` 的說明新增一個檔案，
把步驟 1 那幾行土法指令包成 `MotorDriver` 的五個方法。

**怎麼確認成功：**

```bash
# 不透過 ROS，直接在 python3 裡用你的 driver
python3 -c "
from mowerbot_bridge.drivers.<你的檔名> import <你的類別>
d = <你的類別>(ticks_per_rev=<實際值>)
d.connect()
print('encoders:', d.read_encoders())
d.set_wheel_velocities(0.5, 0.0)   # 只轉左輪
import time; time.sleep(2)
d.stop()
print('encoders:', d.read_encoders())
d.disconnect()
"
```

- 左輪轉、右輪不轉
- 兩次 `read_encoders()` 的左輪數值有變、右輪沒變
- `stop()` 之後輪子停住

---

## 步驟 3：方向檢查

**目標：確認「前進」真的是前進，「左轉」真的是左轉。**

車子仍然架高。啟動：

```bash
ros2 launch mowerbot_bringup bringup_real.launch.py \
     driver_type:=<你的 driver> encoder_ticks_per_rev:=<實際值>
```

另開一個終端機手動送指令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

**怎麼確認成功：**

| 指令 | 應該看到 |
|------|---------|
| `linear.x = 0.1` | 兩輪都往「使車輛前進」的方向轉 |
| `linear.x = -0.1` | 兩輪都反轉 |
| `angular.z = 0.3`（左轉） | 右輪比左輪快（原地左轉時右輪正轉、左輪反轉） |
| `angular.z = -0.3` | 左輪比右輪快 |

**方向錯了怎麼辦：改參數，不要改接線。**

```bash
ros2 param set /mower_bridge invert_left true     # 或 invert_right
```

確認之後把值寫進 `bringup_real.launch.py`。
改接線的話，程式與實物會對不起來，下次有人看程式會被誤導。

---

## 步驟 4：編碼器檢查

**目標：確認 `encoder_ticks_per_rev` 是對的、而且計數方向與轉向一致。**

車子仍然架高，節點不用啟動（或啟動也行）。

1. 在輪子上做一個記號，對準車體上的一個固定點
2. 記下目前的 tick 值
3. **用手**把輪子正轉整整一圈，回到記號對齊
4. 再記一次 tick 值

```bash
ros2 topic echo /motor_status --once
```

**怎麼確認成功：**

- 一圈的 tick 變化量 ≈ `encoder_ticks_per_rev`（差幾個 tick 是正常的，差一倍不是）
- **手往前轉，tick 要增加**。減少的話設 `invert_left` / `invert_right`
- 差一個固定倍數（2 或 4）的話，多半是四倍頻算進去沒有／重複算了
- 差的是減速比，那就是 `encoder_ticks_per_rev` 沒有把減速比乘進去

這一步做完，`/odom` 的尺度才有意義。

---

## 步驟 5：落地前的最後檢查（急停、deadman、watchdog）

**車子仍然架高。這三項全部通過才可以放到地上。**

### 5-1 急停（mode 4）

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"   # 讓它轉
ros2 service call /change_mower_mode mowerbot_interfaces/srv/SetDriveMode "{mode: 4}"
```

確認：輪子**立刻**停止；`ros2 topic echo /cmd_vel` 全是 0；
切回 mode 2 之後輪子**不會自己恢復轉動**。

### 5-2 deadman（搖桿）

不按 LB 時推搖桿 → 輪子不動。按住 LB 推搖桿 → 輪子轉。放開 LB → 立刻停。

### 5-3 watchdog（兩層）

- **bridge_node 這層**：送幾秒 `/cmd_vel` 之後**直接 Ctrl-C 停掉發布**，
  輪子要在 `cmd_vel_timeout`（預設 0.5 秒）內停下來，
  log 會印 `⏱️ 超過 0.50 秒沒收到 /cmd_vel`。
- **驅動板那層**（如果板子有，見 `drivers/README.md` 的 B5）：
  把 USB 線**拔掉**，輪子也要停。這一層是 bridge_node 掛掉時的最後防線。

拔線測試一定要做。軟體 watchdog 保護不了「電腦當機」這個情況。

---

## 步驟 6：里程計校正

**到這裡才可以落地。** 場地淨空半徑 2 公尺以上，手放在急停上。

```bash
# 先在模擬裡跑過一次，確認腳本本身是通的（會算出係數 ≈ 1.0）
python3 test/tools/calibrate_odometry.py --auto

# 實車：互動式，會問你「實際量到多少」
python3 test/tools/calibrate_odometry.py
```

三項實驗：

| 實驗 | 校正什麼 | 怎麼量 |
|------|---------|--------|
| (a) 直線 5 m | `wheel_radius_correction` | 地上貼兩條膠帶，用捲尺量車子實際走了多遠 |
| (b) 原地旋轉 360° x5 | `wheel_separation_correction` | 車體與地面各做一個對齊記號，看轉完差多少度 |
| (c) 2m x 2m 正方形 | 驗證累積誤差 | 量車子實際離出發點多遠 |

**(b) 是這三項裡最重要的。**
四輪 skid-steer 轉彎時輪胎一定會橫向滑動，有效輪距比幾何值（0.58 m）大，
通常落在 **1.3 ~ 1.8 倍**。沒有校正的話，車子實際轉 360 度時
里程計可能只累積出 240 度 —— scan matching 會直接發散，
症狀是「地圖轉一轉就糊掉」，但看起來像是 SLAM 的問題，很難查。

**這支腳本已經在 Gazebo 裡乾跑驗證過（2026-09-21）**，證明流程本身是對的：

| 實驗 | 對著 Gazebo 的完美里程計跑出來的結果 |
|------|--------------------------------|
| (a) 直線 5 m | `/odom` 報 5.0051 m，係數 **1.0000** |
| (b) 原地旋轉 5 圈 | `/odom` 報 1800.05 度（5.000 圈），係數 **1.0000** |
| (c) 2m 正方形 | 回原點誤差 **0.102 m**（佔總行程 1.3%），朝向誤差 3.5 度 |

係數算出 1.0000 是預期的 —— 模擬的里程計本來就是完美的，
這只證明「開車、記錄、計算、輸出」這條流程沒有寫錯，**不代表車子準**。

（(c) 一開始量到 1.478 m 的誤差，那不是里程計的問題，是腳本開環停止時
每個轉彎過衝約 11 度累積出來的。轉彎改成「停下來量、不足就補轉」之後
降到 0.102 m。實車上這個補轉一樣會生效。）

**怎麼確認成功：**

- (a) 的係數落在 0.9 ~ 1.1（差太多先回頭檢查步驟 4 的 tick 數）
- (b) 的係數落在 1.2 ~ 2.0（這是 skid-steer 的正常範圍）
- 把係數填進 `bringup_real.launch.py` 之後**重跑一次**，
  這次三項的係數都應該接近 1.0
- (c) 的回原點誤差 < 總行程的 5%

---

## 步驟 7：第一次實車 SLAM 建圖

雷達驅動要先填進 `bringup_real.launch.py`（見該檔案裡的 TODO）。

```bash
ros2 launch mowerbot_bringup bringup_real.launch.py \
     driver_type:=<你的 driver> encoder_ticks_per_rev:=<實際值>
```

先確認這三件事，再開始推車：

```bash
ros2 topic hz /scan                    # 雷達有資料
ros2 run tf2_ros tf2_echo odom base_footprint    # 里程計 TF 有在動
ros2 run tf2_ros tf2_echo base_link radar        # 雷達外參對得上 URDF
```

然後用搖桿（mode 2）慢慢繞場地一圈。

**怎麼確認成功：**

- RViz 裡的地圖沒有「重影」或「糊掉」
- 繞回起點時，地圖裡的起點牆面與第一次掃到的重疊（閉環正常）
- `ros2 run tf2_ros tf2_echo map odom` 的平移量沒有一直暴衝

**糊掉的話**：九成是步驟 6 的 `wheel_separation_correction` 還不夠準，
回去重做 (b)，圈數加到 10 圈提高解析度。

存圖：

```bash
ros2 run mowerbot_bringup save_map.sh <地圖名稱>
```

---

## 步驟 8：第一次實車 Nav2 循跡

**先在定位模式下確認車子知道自己在哪**，再談自動割草。

```bash
ros2 launch mowerbot_bringup localization.launch.py
ros2 launch mowerbot_bringup navigation.launch.py use_sim_time:=false
```

第一次不要直接跑整個覆蓋任務。照這個順序：

1. 手動送一條 2 公尺的直線路徑給 `/follow_path`，看車子會不會跟
2. 跟得上之後，再送一條有轉彎的
3. 都正常了，才切 mode 1 跑完整的覆蓋任務，而且**第一次要有人跟在旁邊**

**怎麼確認成功：**

- `controller_server` 進到 `active`（`ros2 lifecycle get /controller_server`）
- 車子跟著路徑走，橫向偏差在半個刀盤寬（0.25 m）之內
- 中途按急停，車子立刻停且不會自己恢復

**這裡的數字一定會比模擬差。**
模擬的里程計是完美的（沒有打滑、沒有漂移），
報告 7.5 節已經寫明模擬的循跡數字不能當作實車的預期值。
DWB 的權重、`xy_goal_tolerance`、跑道長度全部要在實車上重新實測決定 ——
**那些值不要照抄模擬的結果。**

---

## 附錄：需要從驅動板文件查到的資訊

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
| B5 | 板子自己有沒有指令逾時保護 |
| D3 | 有沒有硬體急停輸入腳位 |
| D6 | 斷電或斷線時馬達是自由滑行還是煞車 |
