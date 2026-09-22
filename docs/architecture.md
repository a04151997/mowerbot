# mowerbot 系統架構與資料流

這份文件是報告要用的圖。圖用 mermaid 語法寫在 markdown 裡，
GitHub、VS Code 的預覽、以及多數 markdown 轉 PDF 的工具都直接看得到，
不需要另外維護圖片檔。

數據與詳細分析在 `simulation_results.md`，實車步驟在 `hardware_bringup.md`。

---

## 1. 套件關係

```mermaid
graph TD
    subgraph 介面["mowerbot_interfaces（介面定義）"]
        I1["SetDriveMode.srv"]
        I2["GenerateCoveragePath.srv"]
        I3["MotorStatus.msg / MowerStatus.msg"]
        I4["ObstaclePolygons.msg"]
    end

    subgraph 描述["mowerbot_description"]
        D1["car.xacro（URDF）"]
        D2["rviz 設定"]
    end

    subgraph 啟動["mowerbot_bringup"]
        L1["gazebo / mower_control / navigation"]
        L2["demo.launch.py（一鍵啟動）"]
        L3["bringup_real.launch.py（實車）"]
        L4["nav2_params.yaml / mapper_params.yaml"]
        L5["worlds（模擬場地）"]
    end

    subgraph 邏輯["mowerbot_action"]
        A1["mower_manager（模式仲裁 / 任務佇列 / 安全）"]
        A2["map_to_boundary（地圖 → 邊界 + 障礙物）"]
    end

    subgraph 規劃["mowerbot_planner"]
        P1["f2c_server（Fields2Cover 覆蓋路徑）"]
    end

    subgraph 介面層["mowerbot_hmi"]
        U1["hmi_node（PyQt5）<br/>模式切換 + 狀態顯示"]
    end

    subgraph 底盤["mowerbot_bridge"]
        B1["teleop_node（搖桿 → 速度）"]
        B2["bridge_node（ROS ↔ 驅動板）"]
        B3["drivers/（可抽換的驅動層）"]
        B4["odometry.py（里程計數學）"]
    end

    U1 --> I1
    U1 --> I3
    A1 --> I1
    A1 --> I2
    A2 --> I4
    B2 --> I3
    B2 --> B3
    B2 --> B4
    L1 --> D1
    L3 --> B2
```

---

## 2. 完整資料流（割草任務）

從雷射到輪子的一條龍。標在箭頭上的是實際的 topic / service 名稱。

```mermaid
graph LR
    LIDAR["雷達<br/>（模擬：Gazebo plugin）"] -->|"/scan"| SLAM["slam_toolbox"]
    ODO["里程計<br/>模擬：diff_drive plugin<br/>實車：bridge_node"] -->|"/odom + TF<br/>odom→base_footprint"| SLAM
    SLAM -->|"/map + TF<br/>map→odom"| B2B["map_to_boundary"]
    B2B -->|"/f2c_boundary<br/>（外輪廓）"| MGR["mower_manager"]
    B2B -->|"/f2c_obstacles<br/>（內部障礙物）"| MGR
    MGR -->|"generate_coverage_path<br/>（service）"| F2C["f2c_server<br/>Fields2Cover"]
    F2C -->|"coverage_path<br/>周邊環繞 + 弓字形"| MGR
    MGR -->|"一次一條<br/>follow_path（action）"| NAV["controller_server<br/>DWB"]
    NAV -->|"/cmd_vel_nav"| MGR
    JOY["搖桿"] -->|"/joy"| TEL["teleop_node"]
    TEL -->|"/cmd_vel_joy"| MGR
    MGR -->|"/cmd_vel<br/>（唯一出口）"| CHASSIS["底盤<br/>模擬：diff_drive plugin<br/>實車：bridge_node"]
    CHASSIS --> ODO
    MGR -->|"/mower_status<br/>/mission_status（5 Hz）"| HMI["hmi_node<br/>人機介面"]
    HMI -->|"change_mower_mode"| MGR
```

**這張圖最重要的一點：`/cmd_vel` 只有 `mower_manager` 一個發布者。**

Nav2 的輸出被 remap 成 `/cmd_vel_nav`、搖桿的輸出是 `/cmd_vel_joy`，
兩者都必須經過 manager 的模式仲裁才會變成 `/cmd_vel`。
這是安全設計：急停（mode 4）只要讓 manager 不轉發，
任何來源都到不了底盤。詳見 `simulation_results.md` 1.2 節與 N4 測試。

---

## 3. 模式仲裁

```mermaid
stateDiagram-v2
    [*] --> 手動模式
    手動模式 --> F2C模式: mode 1
    手動模式 --> 自動導航: mode 3
    F2C模式 --> 手動模式: mode 2（清空佇列）
    F2C模式 --> 急停: mode 4
    自動導航 --> 急停: mode 4
    手動模式 --> 急停: mode 4
    急停 --> 手動模式: mode 2（佇列已清空，不會自己恢復）

    note right of F2C模式
        切入時自動呼叫 F2C 規劃
        並把路徑切成一段一段循序執行
    end note

    note right of 急停
        取消當前 goal + 清空整個任務佇列
        解除後車子不會自己動起來（N6 驗證）
    end note
```

| mode | 名稱 | `/cmd_vel` 的來源 |
|------|------|------------------|
| 1 | F2C 割草 | `/cmd_vel_nav`（Nav2） |
| 2 | 手動 | `/cmd_vel_joy`（搖桿，且要按住 deadman） |
| 3 | 自動導航 | `/cmd_vel_nav`（Nav2），但不會自己規劃路徑 |
| 4 | 急停 | 無，全部攔截 |

---

## 4. 模擬與實車的差異

**只有兩個方塊不一樣**，其餘節點完全共用 —— 這是整個架構刻意維持的性質。

```mermaid
graph TB
    subgraph SIM["模擬（已完成，28 項測試全過）"]
        S1["gazebo.launch.py"] --> S2["Gazebo diff_drive plugin<br/>免費提供 /odom 與 TF"]
        S1 --> S3["Gazebo 雷達 plugin<br/>提供 /scan"]
    end
    subgraph REAL["實車（骨架完成，待硬體）"]
        R1["bringup_real.launch.py"] --> R2["bridge_node + drivers/<br/>要自己算 /odom 與 TF"]
        R1 --> R3["雷達驅動節點<br/>❌ 待填（要先知道型號）"]
    end
    SIM --> COMMON["共用：slam_toolbox / map_to_boundary /<br/>f2c_server / mower_manager / teleop_node / Nav2"]
    REAL --> COMMON
```

**實車最容易被忽略的一點**：Gazebo 的 diff_drive plugin 免費提供
`/odom` 與 `odom → base_footprint` 的 TF，實車上沒有任何東西會發這兩樣，
而 `slam_toolbox` 硬性要求它。沒有 `bridge_node`，實車上整條鏈路等於零。

---

## 5. 覆蓋率的改善（階段 8 → 10 → 13）

分母是「扣掉地頭的作業區」，數字來自
`python3 test/tools/coverage_analysis.py <log 目錄>`。

| 階段 | 做了什麼 | 作業區未覆蓋率 | 覆蓋落差最大 |
|------|---------|--------------|------------|
| 階段 8（封板） | 弓字形 + 跑道 + 重疊率 0.4 | 9.00 ~ 9.70 %（5 次） | 0.419 ~ 0.545 m |
| 階段 10 | 加上周邊環繞 | 1.99 % / 4.44 %（2 次） | 0.369 / 0.447 m |
| 階段 13 | 同配置的重複性驗證 | `mow_field` 1.99 % ± 0.04 (n=2)<br/>`demo_lawn` 4.62 % ± 1.74 (n=3) | — |

```mermaid
xychart-beta
    title "作業區未覆蓋率（越低越好）"
    x-axis ["階8-1", "階8-2", "階8-3", "階8-4", "階8-5", "階10-1", "階10-2", "階13-mf1", "階13-mf2", "階13-dl1", "階13-dl2", "階13-dl3"]
    y-axis "未覆蓋 %" 0 --> 12
    bar [9.00, 9.09, 9.09, 9.70, 5.22, 1.99, 4.44, 2.02, 1.96, 6.50, 4.28, 3.07]
```

（`mf` = `mow_field.world`，`dl` = `demo_lawn.world`。階段 13 還有一次
`mow_field` 的執行沒有畫進來：那一次任務根本沒啟動，未覆蓋 97.58%，
原因是 `/f2c_boundary` 的發布者競態，見 `simulation_results.md` 12.3 節。
不畫是因為它不是「割草品質」的資料點，但它沒有被從紀錄裡刪掉。）

（`xychart-beta` 需要較新的 mermaid；渲染不出來時看上面的表格即可。）

**剩下的未覆蓋面積 98 ~ 100% 是線間的條狀縫隙**，位置在作業區中段，
成因是循跡誤差而不是邊緣漏割 —— 周邊環繞解決不了它。
要再往下壓必須處理循跡誤差本身（DWB 的權重），
那些值在實車上都要重新實測，現在調等於白做。

---

## 6. 測試套件的組成

```mermaid
graph LR
    subgraph 不需要模擬器["不需要 Gazebo（幾十秒）"]
        A["Phase A 靜態檢查<br/>5 項"]
        B["Phase B 單元功能<br/>4 項"]
        H["Phase H 底盤橋接<br/>5 項"]
    end
    subgraph 需要模擬器["需要 Gazebo（約 15 分鐘）"]
        C["Phase C 模擬整合<br/>5 項"]
        D["Phase D 安全機制<br/>4 項"]
        L["Phase L 存圖與定位<br/>4 項"]
        N["Phase N Nav2 路徑跟隨<br/>6 項"]
        O["Phase O 障礙物容錯<br/>1 項"]
        P["Phase P 狀態發布<br/>6 項"]
    end
    A --> C
    B --> C
    H -.->|"不依賴模擬<br/>可單獨跑"| N
```

共 **40 項**。開發時只跑受影響的 Phase（改 F2C 跑 B + N、改 manager 跑 D + N、
改 bridge_node 跑 H），收尾才跑完整套件。

| Phase | 內容 | 需要 Gazebo |
|-------|------|:-----------:|
| A | build、launch 解析、執行檔、介面欄位、xacro | ✗ |
| B | F2C 路徑、邊界提取、內部障礙物挖洞與提取 | ✗ |
| C | 節點清單、topic 頻率、TF 鏈、服務、RTF | ✓ |
| D | deadman、watchdog、急停、模式仲裁 | ✓ |
| H | bridge_node + loopback：里程計、TF、watchdog、MotorStatus | ✗ |
| L | 建圖、存圖、定位模式 | ✓ |
| N | Nav2 生命週期、F2C 路徑、端到端覆蓋任務、remap、RTF、急停清佇列 | ✓ |
| O | 臨時障礙物擋住割草線時跳過該段並繼續（固定用 `demo_lawn_obstacle.world`） | ✓ |
| P | 狀態發布（`/mower_status`、`/mission_status`）、邊界防呆、HMI 無頭啟動 | ✓ |

另有 17 項純 Python 的里程計單元測試（`mowerbot_bridge/test/test_odometry.py`），
不需要 ROS，一秒內跑完：

```bash
python3 src/mowerbot_bridge/test/test_odometry.py
```

---

## 7. 專案進度

```mermaid
graph LR
    S1["模擬端<br/>✅ 完成"] --> S2["bridge_node 骨架<br/>✅ 完成（loopback 測過）"]
    S2 --> S3["真實 driver 實作<br/>⬜ 等驅動板資訊"]
    S3 --> S4["里程計校正<br/>⬜ 等實車"]
    S4 --> S5["實車 SLAM<br/>⬜"]
    S5 --> S6["實車 Nav2 循跡<br/>⬜"]
```

擋在路上的是**驅動板的規格資訊**，不是程式。
需要查什麼列在 `mowerbot_bridge/drivers/README.md`（26 項），
拿到之後的步驟在 `hardware_bringup.md`。

---

## 8. 圖片索引

| 檔案 | 內容 | 在報告裡的位置 |
|------|------|--------------|
| `coverage_overlap_000.png` | 重疊率 0 的覆蓋示意圖（灰=地頭、淺綠=已割、紅=沒掃到、藍=實際軌跡）。每兩條割草線之間都有月牙形的縫 | 4.6.4 節 |
| `coverage_overlap_040.png` | 重疊率 0.4（選定值）。月牙縫被隔壁那刀補掉，只剩開頭一塊 | 4.6.4 節 |

兩張圖都可以用下列指令重新產生：

```bash
python3 test/tools/coverage_analysis.py --figure test/logs/<timestamp>
```
