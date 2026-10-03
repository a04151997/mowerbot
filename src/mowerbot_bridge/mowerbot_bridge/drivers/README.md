# 馬達驅動層

`bridge_node` 只透過 `base.py` 的 `MotorDriver` 介面跟驅動板講話。
**接上真實驅動板時，只要在這個目錄下新增一個實作，`bridge_node.py` 一行都不用改。**

目前的實作：

| 檔案 | 用途 |
|------|------|
| `base.py` | 抽象介面。所有驅動都要繼承 `MotorDriver` |
| `loopback.py` | 假驅動。把速度指令按時間積分成完美的編碼器計數，沒有硬體時用來測整條資料流 |
| `wheeltec.py` | 輪趣 C30D（階段 36）。暫用下面的接法 (a)，只對著 `test/tools/fake_c30d.py` 驗證過（smoke_test Phase R） |

> **C30D 協定狀態：待驗證（階段 38）。**
> 平台已改為 XLK X2RS，控制板為 STM32F407。OLED 顯示 DIFF/GYRO/ROS，與輪趣系高度相符，
> C30D 協定有相當機會可重用，但待學長 ROS 1 原始碼比對後確認。狀態：待驗證。
>
> （`wheeltec.py` 原樣保留、沒有改。學長的 ROS 1 驅動是 `/home/dreamworker/catkin_ws` 裡的
> `turn_on_mini3_robot`（`mini3_robot.cpp / .h`），比對對象就是它的封包格式。）
>
> **不要假設 `/odom` 由板子韌體計算。** 舊平台的 C30D 會回報車體速度、由韌體積分；
> STM32F407 這塊板子會回報什麼（編碼器計數？輪速？位姿？）未知。
> `bridge_node` 目前的設計是自己從編碼器 / 輪速積分里程計，輪子幾何
> （`rear_wheel_radius`、`wheel_separation`）由 `bringup_real.launch.py` 從
> `mowerbot_description/config/vehicle.yaml`（經 `vehicle_geometry.load()`）傳入 ——
> 不要寫任何依賴「板子會回傳位姿」的程式碼。
>
> 另外：X2RS 的前輪是腳輪，不是四輪 skid-steer。`bridge_node` 與 `bringup_real.launch.py`
> 註解裡「四輪 skid-steer 的有效輪距是幾何值 1.3 ~ 1.8 倍」的說法是舊平台的，
> 對後輪差速 + 腳輪不一定成立；`wheel_separation_correction` 仍要用
> `test/tools/calibrate_odometry.py` 實測，不要沿用那個範圍當預期值。

## 要接真實驅動板時怎麼做

1. 新增 `drivers/<板子名稱>.py`，繼承 `MotorDriver`，實作這五個方法：

   ```python
   connect()                                  # 開啟連線，失敗丟 DriverError
   disconnect()                               # 關閉連線，要能重複呼叫
   set_wheel_velocities(left_rad_s, right_rad_s)   # 單位固定是 rad/s
   read_encoders()                            # 回傳 (left_ticks, right_ticks) 或 None
   stop()                                     # 立刻停止，安全路徑，不要有重試迴圈
   read_status()                              # 選用，回傳 dict
   ```

2. 在 `__init__.py` 的 `DRIVERS` 字典加一行。
3. 啟動時用 `driver_type:=<板子名稱>` 切換。

### 三個不要

- **不要在驅動實作裡偷偷改方向的正負號。** 方向接反用 `bridge_node` 的
  `invert_left` / `invert_right` 參數處理，否則之後查「到底是哪一層反的」會很痛苦。
- **`read_encoders()` 讀失敗時不要回傳上一次的值**，要回傳 `None`。
  回傳舊值會讓里程計以為車子停著，但車子其實在動 —— 那比「這一拍沒資料」危險得多。
- **不要把通訊細節（封包格式、暫存器位址、逾時重試）洩漏到 `bridge_node`。**

## 需要從驅動板文件查到的資訊

**拿到板子之前，下面每一項都是未知的，不要用猜的填。**
這張表可以直接拿去問供應商或查手冊。

### A. 通訊

| # | 要查什麼 | 為什麼需要 |
|---|----------|-----------|
| A1 | 實體介面（USB / RS-232 / RS-485 / CAN / 板載 UART 腳位） | 決定用 pyserial、python-can 還是別的 |
| A2 | 鮑率 / 位元速率、資料位元、同位、停止位元 | 連線參數 |
| A3 | 協定格式（文字指令？二進位封包？Modbus？CANopen？） | 決定怎麼編碼指令 |
| A4 | 封包的框頭、長度欄位、校驗方式（checksum / CRC 的多項式） | 收送封包 |
| A5 | 有沒有「板子主動回報」的模式，還是一問一答 | 決定 `read_encoders()` 要不要自己輪詢 |
| A6 | 指令逾時與重試的建議值 | 驅動層的錯誤處理 |
| A7 | 裝置節點名稱是否固定（udev rule / by-id 路徑） | 避免重開機後 /dev/ttyUSB0 跑掉 |

### B. 速度指令

| # | 要查什麼 | 為什麼需要 |
|---|----------|-----------|
| B1 | 速度指令的單位（RPM？rad/s？PWM 佔空比？內部單位？） | `set_wheel_velocities` 的換算 |
| B2 | 指令值的範圍與飽和行為 | 超出範圍時板子是截斷還是報錯 |
| B3 | 是幾個獨立通道（2 個還是 4 個） | 四輪 skid-steer 同側兩輪要不要分別下指令 |
| B4 | 板子內部有沒有速度閉環（PID），參數要不要我們設 | 決定要不要在上位機做閉環 |
| B5 | 有沒有指令逾時保護（多久沒收到指令會自己停） | **安全相關**，與我們的 watchdog 是兩層 |
| B6 | 加減速斜坡是板子做還是要我們做 | 影響 74 kg 車體的起步衝擊 |

### C. 編碼器

| # | 要查什麼 | 為什麼需要 |
|---|----------|-----------|
| C1 | 每轉的 tick 數（要含減速比與四倍頻） | `encoder_ticks_per_rev`，**沒有這個值 bridge_node 會拒絕啟動** |
| C2 | 回報的是累計值還是增量 | `read_encoders()` 的語意 |
| C3 | 計數器的位元寬度（16 / 32 位元）與是否會回繞 | `DifferentialOdometry` 的 `encoder_wrap` |
| C4 | 計數方向與馬達轉向的關係 | 決定 `invert_left` / `invert_right` |
| C5 | 編碼器讀值的更新頻率 | 決定 `odom_rate` 設多少才有意義 |
| C6 | 讀取失敗時板子怎麼表示（回傳錯誤碼？逾時？） | `read_encoders()` 何時該回 None |

### D. 狀態與安全

| # | 要查什麼 | 為什麼需要 |
|---|----------|-----------|
| D1 | 讀得到電池電壓 / 電流嗎？單位與換算 | `MowerStatus` 的欄位 |
| D2 | 讀得到馬達或驅動溫度嗎？過熱門檻 | `is_overheated` |
| D3 | 有沒有硬體急停輸入腳位，狀態讀得到嗎 | `stop_active`，**安全相關** |
| D4 | 板子的故障碼清單（過流 / 堵轉 / 欠壓 ...） | 錯誤處理與 log |
| D5 | 上電後的預設狀態（是不是需要明確 enable 才會動） | `connect()` 要做什麼 |
| D6 | 斷電或斷線時馬達的行為（自由滑行還是煞車） | **安全相關**，74 kg 的車滑行距離要知道 |

### E. 機械

| # | 要查什麼 | 為什麼需要 |
|---|----------|-----------|
| E1 | 減速比（如果 C1 沒有含進去） | tick 換算 |
| E2 | 馬達額定轉速 → 換算成車速，對照現在的 `max_vel_x` 0.7 m/s | 確認參數設得合理 |

查到之後：C1 填進 `encoder_ticks_per_rev`，C3 填進 `encoder_wrap`，
C4 決定 `invert_*`，其餘寫進新的 driver 實作裡。
校正流程見 `docs/hardware_bringup.md`。

## 輪趣（Wheeltec）C30D：從 wheeltec_robot_ros2 讀到的協定（階段 34，**未經實機確認**）

輪趣沒有公開的 GitHub，程式碼是隨產品用網盤發的。下面是從三份 GitHub 鏡像讀的，
三份的協定定義完全一致（`turn_on_wheeltec_robot/src/wheeltec_robot.cpp`）：

| 鏡像 | 最後 commit |
|------|------------|
| `Aent-8/WHEELTEC_ROS2`（標示 ROS2-V3.5 humble） | b22a31c，2026-02-28 |
| `Yyote/wheeltec_robot_ros2` | 0ef28cf，2024-05-13 |
| `CarlDegio/turn_on_wheeltec_robot` | a48e305，2022-05-03 |

這是輪趣**自家車款**用的下位機協定。C30D 是不是跑同一套韌體，要看現場
`deploy/hw_probe.sh` 第 4 節：115200 baud 下有沒有 BCC 正確的 `7B … 7D` 24 byte 封包。

### 連線

- 序列埠 115200 8N1；輪趣慣例的裝置名 `/dev/wheeltec_controller`（我們改用 `/dev/mowerbot_base`，見 `deploy/99-mowerbot.rules`）
- USB 晶片 CP2102（10c4:ea60）或 CH9102（1a86:55d4），晶片序號燒成 `0002`
- 下位機**主動連續送**上行封包，上位機只在收到 `/cmd_vel` 時送下行封包

### 下行（上位機 → 下位機），11 bytes

| byte | 內容 |
|------|------|
| 0 | `0x7B` 框頭 |
| 1 | 旗標（0 = 一般；1/2/3 是自動回充用，我們一律 0）|
| 2 | 保留 0 |
| 3–4 | vx，int16 **大端**，單位 **mm/s** |
| 5–6 | vy，int16，mm/s（差速車固定 0）|
| 7–8 | wz，int16，單位 **mrad/s** |
| 9 | BCC：byte 0~8 逐位元 XOR |
| 10 | `0x7D` 框尾 |

### 上行（下位機 → 上位機），24 bytes

| byte | 內容 |
|------|------|
| 0 | `0x7B` |
| 1 | Flag_Stop（下位機的停止旗標，語意要查韌體）|
| 2–3 / 4–5 / 6–7 | vx / vy（mm/s）、wz（mrad/s），int16 大端 —— **下位機自己由編碼器算出的車體速度** |
| 8–13 | 加速度 x/y/z，int16，±2 g 量程（÷1671.84 → m/s²）|
| 14–19 | 角速度 x/y/z，int16，±500 °/s 量程（×0.00026644 → rad/s）|
| 20–21 | 電池電壓，uint16，mV |
| 22 | BCC：byte 0~21 XOR |
| 23 | `0x7D` |

### 對照上面 A~E 表：這份協定回答了什麼、還缺什麼

| # | 結果 |
|---|------|
| A1 A2 A3 A4 A5 | 已知（USB 序列、115200、二進位定長封包、XOR BCC、下位機主動送）|
| A7 | 已知（晶片序號 `0002`，待 hw_probe 確認）|
| B1 | **車體速度**（vx mm/s、wz mrad/s），**不是輪速** |
| B3 B4 | 左右輪分配與速度閉環都在下位機韌體裡做 |
| B5 | **未知**。`wheeltec_robot_node` 自己沒有 cmd_vel 逾時，只在收到 `/cmd_vel` 時送一包；韌體有沒有逾時停車看不到 |
| C1 C2 C3 | **不適用**：上行**沒有編碼器 tick**，只有下位機算好的車體速度 |
| C4 | 未知（看實機）|
| D1 | 已知（電壓，mV）|
| D2 D3 D4 D6 | 未知 |
| D5 | 程式裡沒有 enable 指令，開序列埠就能送 |
| E1 E2 | 未知；而且**下位機用來把 vx/wz 換成輪速的輪徑、輪距、減速比是寫在韌體裡的**，是不是我們這台車的值要問輪趣 |

### 建議：用我們的 `bridge_node`，新增 `drivers/wheeltec.py`；不要直接用 `wheeltec_robot_node`

理由（依重要性）：

1. **安全層數**。`bridge_node` 有 0.5 s 的 `cmd_vel_timeout`，跟 `mower_manager` 的 watchdog 是兩層獨立保護。
   `wheeltec_robot_node` 沒有逾時（只在 callback 裡送一次），manager 掛掉時最後一個速度會一直留在下位機上，
   能不能停只看韌體有沒有逾時（B5 未知）。換成它等於拿掉一層安全機制。
2. **TF**。`wheeltec_robot_node` 只發 `/odom`（frame 預設 `odom_combined`），**不發 odom → base_footprint 的 TF**，
   輪趣是另外用 `robot_pose_ekf` 補的。slam_toolbox 硬性要這個 TF，用它的話還要多掛一個 EKF。
3. **介面一致**。`/motor_status`（M-1 的 bag 要錄）、`invert_*`、校正係數、Phase H 的 loopback 測試，
   都建在 `bridge_node` 上。
4. **相依**。`turn_on_wheeltec_robot` 要一起編 `serial`（wjwwood 的 C++ 函式庫，Humble 沒有 apt 套件）、
   `wheeltec_robot_msg`，還 `depend` 了 `turtlesim`、`ackermann_msgs`、`nav2_msgs`；來源是網盤與不明鏡像。
5. 協定本身很小（11 / 24 byte、XOR 校驗），用 pyserial 寫大約 150 行，可以照 `loopback.py` 的方式做單元測試。

**要先決定的一件事**：`MotorDriver` 介面是輪速層級（`set_wheel_velocities` rad/s、`read_encoders` 累計 tick），
但這塊板子是車體速度層級、而且不回報 tick。兩種接法：

- (a) 不動 `bridge_node`：`wheeltec.py` 把左右輪速用同一組輪半徑/輪距換回 (vx, wz) 送出；
  把上行的 (vx, wz) 換回左右輪速後積分成「等效 tick」回報。線性換算、來回無損，
  但 tick 是算出來的，不是編碼器原始值。
- (b) 擴充介面，讓 driver 可以直接回報車體速度，`bridge_node` 對這種 driver 改成積分速度。
  比較誠實，但要改 `bridge_node` 的里程計路徑。

兩種都還要等 hw_probe 確認協定、以及 field_checklist 第 3 節確認**有沒有編碼器**
（沒有編碼器的話上行速度的意義要重新確認）之後再決定。

**階段 36 的現況**：為了在實車通電前先把硬體迴路測起來，`wheeltec.py` 先用 (a) 接上，
這個決定**還沒拍板**，現場確認之後再選 (a) 或 (b)。(a) 目前已知的副作用：

- `encoder_ticks_per_rev` 對這個驅動只是等效 tick 的解析度（Phase R 用 4096），沒有物理意義，
  但 `bridge_node` 仍然要求它 > 0。
- `wheel_radius_correction` / `wheel_separation_correction` 下行與上行會互相抵消，等於不起作用；
  `vehicle.yaml` 的輪半徑、輪距也一樣會抵消 —— 真正決定車速換算的是韌體裡的幾何（E1 E2）。
