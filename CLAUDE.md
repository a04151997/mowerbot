# mowerbot 專案規則

## 回報語言

所有回報一律用繁體中文（程式碼與 log 原文除外）。

## 硬性規則：ROS 1 與 ROS 2 的驅動節點不可同時執行

上位機上同時有學長的 ROS Melodic 原系統（`turn_on_mini3_robot`）與我們的 ROS 2 `bridge_node`。
兩者會搶同一個序列埠，下行指令互相覆蓋 —— 對上百公斤的載具這是安全問題。
**每次測試前先 `ps aux | grep -E "mini3|bridge_node|roslaunch|ros2"` 確認沒有殘留節點**，
有就先停掉再開始。原系統是「已知可用的參照系統」，不得覆蓋或重裝。

## 硬性規則：參數掃描前先證明參數生效

任何參數掃描開始之前，**必須先做一次對照，證明該參數確實生效**：
設一個極端值確認行為明顯改變，或確認某個原本會出現的現象消失（例：設 min_speed_theta = 0.3，
確認轉不動時的 wz 指令真的變成 >= 0.3）。**管道未經驗證之前量到的數字一律不算數。**
更一般地：判讀任何自動化結果之前，先確認量測管道本身是好的。
理由與四個前例（ros2 daemon 快取、Phase O 夾具、監看腳本 grep、參數注入）見 `docs/stage38_decisions.md` 15 節。

## 凍結條件

### 階段 38 解除的部分（2026-10-01）：車輛幾何、URDF

下列兩項**已解除**，可以依實測值修改：

- 車輛幾何（`src/mowerbot_description/config/vehicle.yaml`）
- URDF（`src/mowerbot_description/urdf/` 下所有 xacro，含輪子位置、前輪模型）

理由：

1. 平台已更換為 XLK X2RS，實車尺寸已實測取得。舊的幾何值不是暫定值而是錯誤值，保留它沒有意義。
2. 實車前輪經照片確認為「萬向輪（腳輪）」，不是固定輪。原本「四輪滑移轉向、草地可能轉不動」
   的風險依據不成立。這同時解釋了階段 30 摩擦力掃描「前輪 mu >= 1.0 時完全無法轉向」的結果
   —— 那是把前輪建成固定輪的建模假象，不是物理限制（`docs/simulation_results.md` 30.4、38 節）。

隨之而來的兩項**改為由幾何推導，不再是獨立的凍結值**（推導公式只在
`src/mowerbot_description/mowerbot_description/vehicle_geometry.py` 一處）：

- headland（`headland_width` = ceil((rotation_swept_radius + 0.12) / 0.05) x 0.05，目前 1.20 m）
- 掉頭 / 通過淨空門檻的**數值**（`rotation_swept_radius` 1.0616、`lateral_half_extent` 0.42）

vehicle.yaml 的每一個值都要在 `provenance` 標 measured / measured_coarse / provisional / derived；
measured_coarse 必須附 `uncertainty`；provisional 項目一律走 allow_provisional 閘門（實機預設擋下），
不得寫字面 TBD。跨參數一致性（例如光達掃描面不得落在車體方塊內）由 `vehicle_geometry.py` 啟動時斷言。

### 仍然凍結（等實機量到再動）

- 角速度上限（階段 38 起它有第二個理由：對正控制器的容忍值 0.10 rad 是在模擬不可信的低速旋轉行為下調出來的，
  實機原地旋轉的「命令 vs 實際角速度」與停止精度量到之前，它不是定案值 —— 見 `docs/stage38_decisions.md` 14 節）
- 速度 / 加速度限制（`nav2_params.yaml` 的 max_vel_* / acc_lim_* / decel_lim_*，URDF 的 max_wheel_acceleration）
- DWB critic 權重
- 安全機制行為（deadman、watchdog、急停、模式仲裁、佇列清空）
- 既有測試的通過標準
- 跟掉頭有關的**邏輯**（原地旋轉的方式、approach 的 goal checker、「依是否需要旋轉套用兩種門檻」的規則）
  —— 階段 38 只換了門檻的數值來源，沒有改邏輯

實機量測步驟見 `docs/hardware_bringup.md`，暫定值的量測工單見 `docs/measurement_worklist.md`。
