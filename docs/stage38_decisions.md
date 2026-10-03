# 階段 38 自行決定事項與寫死常數清查

依階段 38 修正 3 第 0 部分的原則自行決定的事項，一項一行，附理由。
原則編號：P1 幾何門檻一律從 vehicle.yaml 推導、單一來源；P2 三個半徑用途固定；
P3 隔離變數（非安全、會增加混淆的常數維持原值只做單一來源化）；P4 衝突取嚴格；
P5 不放寬測試標準（量測方式修正要標註）；P6 不確定值走 provisional；P7「要一致」註解的寫死常數改單一來源。

---

## 1. 寫死常數全面清查表

格式：檔案:行號（改完之後的行號）| 舊值 | 新來源 | 數值有無改變 | 理由

### 1.1 產品程式

| 檔案:行號 | 舊值 | 新來源 | 數值改變 | 理由 |
|---|---|---|---|---|
| `mowerbot_action/manager.py:154` | `LEAD_IN_BOUNDARY_CLEARANCE = footprint_width/2` = 0.34 | 參數 `lateral_half_extent` ← `vehicle_geometry` | **0.34 → 0.42** | P2：直線通過門檻 = 側向半寬（舊值 0.34 是側向半寬，與內接半徑相等是巧合） |
| `mowerbot_action/manager.py:156` | `ROTATION_CLEARANCE = hypot(L/2, W/2)` = 0.5841 | 參數 `rotation_swept_radius` | **0.5841 → 1.0616** | P2：繞後輪軸旋轉時車頭角的半徑；規格第 7 節 |
| `mowerbot_action/manager.py:158` | `LEAD_IN_OBSTACLE_CLEARANCE = 0.45`（類別常數，註明與 inflation_radius 一致） | 參數 `soft_inflation_radius` ← `vehicle_geometry.SOFT_INFLATION_RADIUS` | 否 | P7、修正 3 第 1 部分 (a) |
| `mowerbot_action/manager.py:24` | `overlap_ratio` 預設 0.4（manager、兩支 launch、smoke_test 各一份，註明要一起改） | 模組常數 `DEFAULT_OVERLAP_RATIO` | 否 | P7 |
| `mowerbot_action/manager.py:26` | `min_boundary_area` 預設 4.0（manager 與 HMI 各一份，註明一致） | 模組常數 `DEFAULT_MIN_BOUNDARY_AREA` | 否 | P7。數值本身列入待重新推導（見 4 節） |
| `mowerbot_action/manager.py:28` | 航點間距 0.1（manager 兩處、f2c_server 兩處，註明一致） | 模組常數 `WAYPOINT_SPACING`，f2c_server 由 launch 傳參數 | 否 | P7 |
| `mowerbot_action/manager.py:587` | `APPROACH_WAYPOINT_SPACING = 0.1` | `WAYPOINT_SPACING` | 否 | P7 |
| `mowerbot_action/manager.py:677` | `LEAD_IN_WAYPOINT_SPACING = 0.1` | `WAYPOINT_SPACING` | 否 | P7 |
| `mowerbot_action/manager.py:403` | `req.turning_radius = 1.0` | 不變 | 否 | f2c_server 完全沒有讀這個欄位（grep 確認），不影響行為；P3 |
| `mowerbot_action/manager.py:586` | `APPROACH_SKIP_DISTANCE = 0.5` | 不變 | 否 | 演算法參數（距離小於它不插 approach），不是車輛幾何；P3 |
| `mowerbot_action/manager.py:977` | `GAP_CUT_DISTANCE = 0.3` | 不變（測試端改讀它） | 否 | 演算法參數；它是測試與 coverage_analysis 的單一來源 |
| `mowerbot_action/manager.py:1182` | `APPROACH_TARGET_SEARCH_M = 1.0` | 不變 | 否 | 演算法參數；P3 |
| `mowerbot_action/manager.py:708` | `LEAD_IN_FALLBACKS = (0.3, 0.2, 0.0)` | 不變 | 否 | 演算法參數；P3 |
| `mowerbot_planner/src/f2c_server.cpp:38` | `headland_width` 預設 0.70 | 參數（無預設）← `vehicle_geometry.headland_width` | **0.70 → 1.20** | 規格第 6 節。公式只在 Python 算一次，f2c_server 只斷言 ≥ rotation_swept_radius + 0.10 |
| `mowerbot_planner/src/f2c_server.cpp:230` | `kPerimeterSpacing = 0.1` | 參數 `waypoint_spacing` | 否 | P7 |
| `mowerbot_planner/src/f2c_server.cpp:315` | `waypoint_spacing = 0.1` | 參數 `waypoint_spacing` | 否 | P7 |
| `mowerbot_bringup/launch/mower_control.launch.py` | launch 參數 `headland_width` 預設 0.70 | 刪除，改由 `vehicle_geometry` 推導 | **0.70 → 1.20** | 規格第 6 節「不得寫死」 |
| `mowerbot_bringup/config/nav2_params.yaml:169` | `inflation_radius: 0.45` | `navigation.launch.py` 注入 `SOFT_INFLATION_RADIUS`；yaml 再寫一份就拒絕啟動 | 否 | 修正 3 第 1 部分 (a) |
| `mowerbot_bringup/config/nav2_params.yaml:92` | `FollowPath.xy_goal_tolerance: 0.10`（註明與 general_goal_checker「兩邊都要改」） | `navigation.launch.py` 從 `general_goal_checker.xy_goal_tolerance` 注入 | 否 | P7 |
| `mowerbot_bringup/launch/navigation.launch.py` | footprint = ±L/2, ±W/2（對稱） | `vehicle_geometry.footprint`（真實四點） | **是**（前 0.975 / 後 0.155 / 側 0.42） | 規格第 2、5 節 |
| `mowerbot_bringup/config/nav2_params.yaml:119` | `PathAlign.forward_point_distance: 0.5`（註解「車長 0.95 m」） | 不變 | 否 | DWB critic 設定屬凍結項；註解的「車長 0.95」已過期，列入待重新推導 |
| `mowerbot_bringup/config/nav2_params.yaml:78` | `acc_lim_x 0.5` 等（由輪半徑 0.17 / 輪距 0.58 推導） | 不變 | 否 | 速度 / 加速度限制凍結；推導基準已改變，列入待重新推導（見 4 節） |
| `mowerbot_bringup/config/nav2_params.yaml:24` | `required_movement_radius: 0.5` | 不變 | 否 | progress checker，不是車輛幾何；P3 |
| `mowerbot_bringup/launch/bringup_real.launch.py:79` | `wheel_radius: vehicle['wheel_radius']` | `geom.rear_wheel_radius` | **0.17 → 0.155** | 實測值；驅動輪是後輪 |
| `mowerbot_bridge/bridge_node.py:97` | （無） | `vehicle_geometry.check_gate` | — | 修正 1 C 節第二道閘門 |
| `mowerbot_hmi/hmi_node.py:88` | `BOUNDARY_MIN_AREA_HINT = 4.0`（註明一致） | `DEFAULT_MIN_BOUNDARY_AREA` | 否 | P7 |
| `mowerbot_description/launch/robot_state_publisher.launch.py` | 「車寬要包住後輪」斷言用 `footprint_width` 與從 URDF 讀輪寬 | 移進 `vehicle_geometry.load()`，用 `body_width_total` 與 `rear_wheel_width` | 否 | 原 key 已不存在；輪寬依 P1/P6 進 vehicle.yaml |

### 1.2 URDF / xacro

| 檔案:行號 | 舊值 | 新來源 | 數值改變 | 理由 |
|---|---|---|---|---|
| `car_base.xacro` | base_link = 車體中心、離地 0.43 | base_link = 後輪軸中心，離地 `rear_wheel_radius` | **是** | 規格第 2、4 節 |
| `car_base.xacro:23` | `ground_height = 0.2`（車體方塊底） | vehicle.yaml `body_ground_clearance`（provisional） | 否 | P1 / P6 |
| `car_base.xacro:56` | 質心 `o_xyz="0 0 -0.18"`（車體中心下 0.18 = 離地 0.25） | vehicle.yaml `com_x` 0.41 / `com_height` 0.25（provisional） | 水平：仍在車體幾何中心；高度不變 | P1 / P6；質心決定後輪 / 腳輪載重分配 |
| `car_wheels.xacro:17` | 後輪 x = −0.35、前輪 x = +0.35（軸距 0.70 寫死） | 後輪 x = 0；腳輪滾動軸 x = `wheelbase` 0.875 | **是** | 規格第 4 節 |
| `car_wheels.xacro` | 輪寬 0.1（前後同） | `rear_wheel_width` 0.10 / `front_wheel_width` 0.05（provisional） | 後輪否、前輪是 | P1 / P6 |
| `car_wheels.xacro:67` | 前輪固定方向 continuous | 腳輪：swivel（z）+ roll（y）兩關節，trail 0.02 | **是** | 規格第 4 節 |
| `car_wheels.xacro` | 前輪 mu1 0.1 / mu2 0.0 | 腳輪 mu1 0.1 / mu2 0.05 | 是 | 規格第 4 節 |
| `car_wheels.xacro` | `wheel_mass 1.0`、swivel 0.2 kg | 不變 / 新增 | — | 模型質量，非幾何門檻；P3 |
| `car_radar.xacro:12,14` | 雷達 (0, 0, 0.45)（舊 base_link 座標，離地 0.88） | x = `lidar_x` 0.30（measured_coarse ±0.08）；z = `lidar_z_ground` 0.55 − `rear_wheel_radius`（由載入器推導） | **是**（離地 0.88 → 0.55，往前 0.30） | 最終版第 3、7 節 |
| `car_radar.xacro:4` | 雷達圓柱 r 0.1 / h 0.1 | 不變 | 否 | 感測器外殼模型，不參與任何門檻 |
| `car_camera.xacro:10` | 車體前緣 −0.02、車頂 | `front_extent − 0.02`、`body_top_z` | 位置隨車體改變 | 以車體方塊為準（推導），相機不參與導航 |
| `car_imu.xacro:19` | 車體頂面中心 | `body_center_x`、`body_top_z` | 位置隨車體改變 | 同上 |
| `car_engine.xacro`（新） | — | 中心 x = `blade_offset_x`（曲軸與刀盤同心，推導）；尺寸 / 高度 `engine_*`（provisional）；質量 0.1 kg 佔位 | — | 最終版第 7 節；Gazebo 會丟掉沒有 inertial 的 link 連同碰撞 |
| `car_base.xacro` | 刀片 link 不存在 | `blade_link` 於 x = `blade_offset_x`（0.375）、貼地 | — | 最終版第 7 節 |
| diff_drive plugin | `wheel_diameter = 2 × 0.17` | `2 × rear_wheel_radius` = 0.31 | **是** | 實測值 |

### 1.3 測試與工具

| 檔案:行號 | 舊值 | 新來源 | 數值改變 | 理由 |
|---|---|---|---|---|
| `test/smoke_test.py:106` | `VEHICLE_INSCRIBED = footprint_width/2` | `VEHICLE_LATERAL_HALF = GEOM.lateral_half_extent` | **0.34 → 0.42** | P2 |
| `test/smoke_test.py:107` | `VEHICLE_CIRCUMSCRIBED` 0.5841 | `VEHICLE_ROT_SWEPT = GEOM.rotation_swept_radius` | **→ 1.0616** | P2 |
| `test/smoke_test.py:1485` | `GAP_CUT_DISTANCE = 0.3`（註明一致） | `MowerManager.GAP_CUT_DISTANCE` | 否 | P7 |
| `test/smoke_test.py` | `DEFAULT_OVERLAP = 0.4`（註明同步修改） | `mowerbot_action.manager.DEFAULT_OVERLAP_RATIO` | 否 | P7 |
| `test/smoke_test.py:1889` | `TEST_BOUNDARY_CENTER = (-1.5, -1.5)` | `(-1.2, -1.2)` + N3 前的起點淨空斷言 | **是** | 夾具：舊中心下起點離邊界 1.00 < 1.06，manager 會拒絕開始（見 2 節） |
| `test/smoke_test.py:1893` | — | `N_START_MARGIN = 0.20` | — | 與 O_FIXTURE_MARGIN 同一理由 |
| `test/smoke_test.py:3463` | `O_BOUNDARY` 半邊長 3.0 | 3.65 + `o_min_half()` 公式斷言 | **是** | 規格第 8 節 |
| `test/smoke_test.py:3467` | `O_BODY_HALF_WIDTH = VEHICLE_INSCRIBED` | `VEHICLE_LATERAL_HALF` | **0.34 → 0.42** | P2 |
| `test/smoke_test.py:3474` | `O_FIXTURE_MARGIN = 0.20` | 不變 | 否 | 夾具餘裕的理由（2 格 + 0.10 循跡）與車輛無關；P3 |
| `test/smoke_test.py:4413` | `Q_INSCRIBED = VEHICLE_INSCRIBED` | `VEHICLE_LATERAL_HALF` | **0.34 → 0.42** | P2（Q2 的「通過點」門檻） |
| `test/smoke_test.py:4415` | `Q_INFLATION = 0.45`（註明一致） | `GEOM.soft_inflation_radius` | 否 | P7 |
| `test/smoke_test.py:4423` | `Q_CIRCUM = VEHICLE_CIRCUMSCRIBED` | `VEHICLE_ROT_SWEPT` | **→ 1.0616** | P2 |
| `test/smoke_test.py:4651` | Q3 `target_clear = 0.45`、往前開 | `(rear_extent + rotation_swept_radius)/2` = 0.608、倒車 | **是** | 夾具修正：車頭在前方 0.975，往前開到 0.45 會先撞牆（判定不變） |
| `test/smoke_test.py:1201` | C3 預期 radar z = 0.45 | `GEOM.lidar_z` | **0.45 → 0.70** | 量測方式修正（判定仍是「TF 的 z 與 URDF 設定差 < 0.01」） |
| `test/smoke_test.py:2313` | N3 `trim_radius=0.35` | 不變 | 否 | 量測參數，不影響判定；註解「xy_goal_tolerance 0.25」已過期 |
| `test/smoke_test.py:2548` | `H_WHEEL_RADIUS = VEHICLE['wheel_radius']` | `GEOM.rear_wheel_radius` | **0.17 → 0.155** | 實測值 |
| `test/tools/coverage_budget.py:68` | `BODY_HALF = footprint_width/2` 0.34 | `GEOM.lateral_half_extent` 0.42 | **是**（E1 帶寬 0.19 → 0.27） | P2；**量測方式改變**，另提供 `--body-half 0.34` 供對照 |
| `test/tools/coverage_budget.py:65` | `BLADE_HALF = blade_width/2` | `GEOM.blade_width/2` | 否 | 單一來源 |
| `test/tools/coverage_budget.py:72` | `ERODE = 0.10`（註明是 map_to_boundary 的腐蝕量） | `MapToBoundaryNode.BOUNDARY_ERODE_CELLS × 地圖解析度` | 否 | P7 |
| `test/tools/coverage_run.py:80` | — | 刀片偏移從 TF `base_footprint → blade_link` 查一次 | — | 規格第 9 節；查不到就停，不退回寫死的數字 |
| `test/tools/segment_endpoint.py:115` | 分箱 0.10 / 0.30 m | 不變 | 否 | 統計分箱，不是車輛幾何 |
| `test/tools/approach_guard_probe.py:52` | 傳 footprint_length / width | 傳三個門檻 + blade_width | 否（manager 參數改名） | 參數介面跟著 manager |
| `test/tools/o_failure_trace.py:35` | `HALF_L / HALF_W` 對稱、取樣格數 20 × 14 寫死 | `FRONT / REAR / HALF_W`，格數由 footprint 推導 | 是 | P1 |
| `test/tools/segment_failure.py:154` | 由 footprint 算外接 / 內切 | `rotation_swept_radius / lateral_half_extent` | 是 | P2 |
| `test/tools/coverage_analysis.py:48` | `HEADLAND_WIDTH = 0.5`、`BLADE_WIDTH = 0.5`、`TEST_BOUNDARY_*`、0.3（註明與 smoke_test 一致） | `GEOM`、smoke_test 原始碼（ast 取值）、`MowerManager.GAP_CUT_DISTANCE` | **headland 0.5 → 1.20**（舊值早就是錯的 0.70） | P7。舊工具，階段 18 起改用 coverage_budget |

### 1.4 數值確實改變的項目對基線的影響

| 項目 | 舊 → 新 | 對新舊基線對照的影響 |
|---|---|---|
| headland_width | 0.70 → 1.20 | 作業區每邊內縮多 0.50 m，割草線變少、外圈離牆更遠 → 覆蓋率下降（預期） |
| 直線通過門檻 | 0.34 → 0.42 | 跑道起點與佇列檢查更嚴 |
| 掉頭門檻 | 0.5841 → 1.0616 | 跑道 (lead-in) 大量不可行（主基線 34 條裡 26 條沒有跑道） |
| footprint | 對稱 0.95 × 0.68 → 前 0.975 / 後 0.155 / 側 0.42 | costmap inscribed 帶 0.34 → 0.155 |
| base_link | 車體中心 → 後輪軸 | 軌跡 (x, y) 的意義改變（量測方式），刀片在其前方 0.50 |
| E1 帶寬 | 0.19 → 0.27 | 「扣 E1 覆蓋率」的分母改變（量測方式）；對照表另附 0.34 版本 |
| 光達位置 / 引擎遮擋 | 新增 | 定位品質與建圖（見報告 38 節） |
| 前輪模型 | 固定輪 → 腳輪 | 原地旋轉不再受前輪側滑阻擋 |

---

## 2. 自行決定事項（一項一行）

1. vehicle.yaml 除了規格列的 4 個暫定值，另新增 14 個 provisional 項（lidar_x/z、engine_* 5 項、com_x/height、caster_track、caster_trail、rear/front_wheel_width、body_ground_clearance）—— P1：原本寫死在 xacro 的幾何值要有單一來源；P6：它們都沒量過。
2. 新增項目的 impact：lidar_z / engine_top_z / engine_width = high（決定遮擋）、lidar_x / engine_center_x / engine_length / com_x = medium、其餘 low —— 依「改了會不會改變模擬結論」排序。
3. `front_wheel_type: caster` 的 provenance 標 measured —— 規格寫「照片確認」。
4. 光達 `lidar_z = 0.70` 解讀為**相對 base_link**（URDF 慣例），離地 0.855 m，高於鋁架上緣 0.70 —— 規格寫「鋁架上方」，若是離地 0.70 則光達中心剛好在鋁架上緣、半個外殼在架子裡。
5. 引擎中心 x 取 0.50（= blade_offset_x 的目測值）、尺寸 0.40 × 0.45、離地 0.45 ~ 0.90 —— 修正 1 寫「刀片在引擎下方」；頂端 0.90 > 光達掃描面 0.855，符合「高度相近、會遮擋」。
6. 引擎 link 質量 0.1 kg —— Gazebo 會把沒有 inertial 的固定 link 連同碰撞一起丟掉；整車質量仍以 vehicle.yaml 的 mass 為準（修正 1 E 節）。
7. 質心水平位置取車體幾何中心 (0.41)、高度沿用舊模型離地 0.25 —— 沒有量過；放進 provisional，不另猜「引擎在前所以偏前」（P3）。
8. 腳輪轉軸高度 = 滾動軸上方 front_wheel_radius（叉架高度）、swivel link 0.2 kg —— 建模值，只影響腳輪動力學；沒有 trail 以外的阻尼。
9. 車體方塊從離地 0.20（舊 ground_height）到 body_height —— 方塊只影響外觀與慣量，P3。
10. 相機放在車頭、車頂（以新車體方塊為準）—— 沿用舊模型「車頭前緣 −0.02、車頂」的相對關係；相機不參與導航。
11. 幾何載入器放在 `mowerbot_description/mowerbot_description/vehicle_geometry.py`（ament_cmake_python 安裝）—— 與 vehicle.yaml 同一個套件，launch / 節點 / 測試都 import 它。
12. headland 公式只在 Python 算一次；f2c_server（C++）收 `headland_width`、`rotation_swept_radius`、`headland_min_margin` 三個參數並在建構時斷言、不滿足就 throw（程序以非零結束）—— 規格要 f2c_server「從 vehicle.yaml 推導」，但 C++ 再讀一次 yaml 等於公式有兩份，違反 P1。
13. 刪掉 `mower_control.launch.py` 的 `headland_width` launch 參數 —— 它的預設值 0.70 就是規格說的「寫死」；目前沒有任何測試或腳本覆寫它（grep 確認）。
14. 保留階段 31 的 `check_headland`（headland ≥ rotation_swept_radius + nav2 的 xy_goal_tolerance），改用推導出來的 headland —— 它看的是 nav2 實際的 xy_goal_tolerance，f2c_server 的斷言看的是固定 0.10；兩道不重複（P4）。
15. 三個跨檔預設值（重疊率 0.4、最小邊界面積 4.0、航點間距 0.1）放在 `mowerbot_action.manager` 的模組常數 —— manager 是它們真正被使用的地方；launch / HMI / 測試 import。
16. HMI 新增 `exec_depend mowerbot_action`、bridge 與 action 新增 `exec_depend mowerbot_description` —— 為了 import 單一來源。
17. `FollowPath.xy_goal_tolerance` 由 `navigation.launch.py` 注入 —— P7（「兩邊都要改」的註解）。數值 0.10 不變。
18. `geometry_guard` 同時放在 `demo.launch.py`（模擬，allow_provisional 預設 true）與 `bringup_real.launch.py`（實機，預設 false）—— 修正 1 C 節要求「模擬 launch 預設 true」，模擬端需要同一個節點印橫幅；檢查邏輯仍只有 `vehicle_geometry.check_gate` 一份，其他節點沒有重複檢查。
19. bridge_node 的第二道閘門放在「以 read_only=false 啟動時、建立驅動之前」—— `read_only` 目前只是啟動參數，沒有執行中從 true 轉 false 的路徑；新增執行中切換等於新增介面（停止條件 3），所以不做。
20. 閘門不通過時 launch 以例外結束（結束碼 1）—— 實測只送 Shutdown 事件時 `ros2 launch` 結束碼是 0，腳本會以為啟動成功。
21. Phase H、Phase R 的 bridge / bringup_real 啟動加 `allow_provisional:=true` —— 它們對著 loopback / 假 C30D，等同模擬；不加的話閘門會擋下（正確行為），但那不是這兩個 Phase 要測的東西。
22. smoke_test 新增結果狀態 `FIXTURE`（夾具失效）：不算通過、結束碼 1，摘要單獨列出 —— 規格第 8 節「標記為 fixture 失效而非測試失敗」。Phase O 原本執行中的夾具檢查（階段 26）也從 FAIL 改記為 FIXTURE，判定（不通過）不變。
23. Phase O 場地公式除了「環繞直線段離箱子 ≥ 半寬 + 0.20」，另加「環繞四個轉角離箱子 ≥ rotation_swept_radius + 0.20」—— 環繞每段開頭會原地轉 90°（P4 取較嚴格）；直線條件下限 3.47、加轉角條件後 3.61，取 3.65。
24. Phase N 改用挪中心而不是放大邊界 —— 面積不變、障礙物淨空仍 ≥ 1.5（1.81），N3 少一個混淆變數（P3）。
25. Phase Q3 改成背對邊界倒車 —— 往前開會先撞牆；target 取 rear_extent 與 rotation_swept_radius 的中點（推導，不寫死）。
26. coverage_budget 的 E1 帶寬改用 lateral_half_extent（0.27），並提供 `--body-half` 讓對照表能算「舊量法」—— P2；量測方式的差異要能單獨拆出來。
27. CSV 蓋章用 `# PROVISIONAL: ...` 註解行；所有讀 CSV 的工具一律濾掉 `#` 開頭的行 —— 修正 1 D 節第 2 點。
28. 蓋章字串依 impact (high → low) 再依鍵名排序 —— 最重要的未知數排最前面。
29. `coverage_run.py` 另存 `_mapodom.csv`（map → odom 每秒一筆）—— 模擬 odom 是完美的，map → odom 的偏離就是定位漂移，用來回答「引擎遮擋對建圖 / 定位的可觀察影響」。
30. `min_boundary_area` 數值維持 4.0 —— P3；但它的理由（「4.0 在幾何下限 1.96 之上」）已不成立，幾何下限變成 (2 × 1.20)² = 5.76 m²，列入待重新推導。
31. `PathAlign.forward_point_distance`、`acc_lim_*` 的過期註解不改 —— 凍結範圍，避免讓人以為數值有被重新推導過。
33. 蓋章的檔名後綴（_PROVISIONAL）用在本輪產生的每一份報告檔（budget / corner / overshoot / endpoint / heatmap / suite 輸出 / smoke_test 摘要）；`docs/simulation_results.md` 是跨階段的總報告，不改檔名，改成在 38 節開頭放蓋章行 —— 改名會讓所有既有的「見報告 N 節」引用失效。
32. 摩擦掃描除了規格的 0.1 / 0.3 / 0.5 / 1.0（mu1 = mu2），另跑一次現況 0.1 / 0.05 當對照。

---

## 3. 規格與實作的差異（照實記錄）

* CSV 原本就是 10 欄（階段 29 加了 yaw），不是 9 欄；blade_x / blade_y 是第 11、12 欄，前 10 欄不變。
* 規格第 10 節寫「F2C 2.0.0」：開發機實際安裝 `ros-humble-fields2cover 2.0.0-11jammy`，與階段 32 基線相同（階段 35 的 2.1.0 是部署目標，不是開發機）。
* 規格第 4 節「光達 TF 保留現有相對位置」由修正 3 第 4 部分取代（改用 provisional 值）。

---

## 4. 待重新推導清單

| 項目 | 目前值 | 為什麼要重新推導 | 等什麼 |
|---|---|---|---|
| `soft_inflation_radius` | 0.45 | 只比 lateral_half_extent 0.42 多 0.03；照舊公式的意義應為 0.42 + 0.11 = 0.53 | 第 6 部分角點偏移量測（本報告）+ 使用者決定 |
| 直線通過門檻 | 0.420（不疊餘裕） | 偏航 θ 時車頭角側向 = 0.42 cos θ + 0.975 sin θ | 同上 |
| `TURN_MARGIN` | 0.12 | 沿用舊的絕對餘裕；新車掉頭半徑大、循跡誤差的側向代價翻倍 | 實機原地旋轉量測 |
| `min_boundary_area` | 4.0 m² | 幾何下限已是 5.76 m² | 使用者決定 |
| `acc_lim_x` / `acc_lim_theta` | 0.5 / 1.5 | 由 0.17 / 0.58 推導；新值 3.0 × 0.155 = 0.465 m/s² < 0.5（DWB 的加速度上限比模擬輪子能給的還大） | 凍結（速度 / 加速度限制） |
| `PathAlign.forward_point_distance` | 0.5 | 註解依據是舊車長 0.95 | 凍結（DWB critic） |
| `wheel_separation_correction` 預期範圍 | 1.3 ~ 1.8（註解） | 那是四輪 skid-steer 的經驗值，腳輪平台不一定成立 | 實機 calibrate_odometry |
| Phase N `trim_radius` | 0.35 | 註解依據 xy_goal_tolerance 0.25，現在是 0.10 | 不影響判定，低優先 |
| `body_ground_clearance` / `com_*` / `caster_*` / 輪寬 | 見 vehicle.yaml | provisional | `docs/measurement_worklist.md` |


---

## 5. 最終版（2026-10-02）相對前一版的變更

| 項目 | 前一版 | 最終版 | 理由 |
|---|---|---|---|
| provenance 種類 | measured / provisional / derived | 加 `measured_coarse`，必須附 `uncertainty` | 最終版第 3、4 節 |
| blade_width / mass | provisional | measured（原廠規格） | 最終版第 3 節 |
| blade_offset_x | 0.50 provisional | 0.375 ± 0.025 measured_coarse | 實測範圍 0.35 ~ 0.40 |
| lidar_x | −0.05 provisional | 0.30 ± 0.08 measured_coarse | 實測 30 ~ 32 cm，基準點未確認 |
| lidar_z → lidar_z_ground | 0.70（相對 base_link） | 0.55（**離地**），base_link 相對高度由載入器減 rear_wheel_radius | 最終版第 3、7 節 |
| body_height | 0.70 | **0.50**（自洽性修正，見 6 節） | 使用者修正 |
| engine_center_x | 0.50 provisional | 刪除，改由 blade_offset_x 推導 | 曲軸與刀盤同心：同一個物理量只能有一個來源（原則 1） |
| engine_top / bottom | 0.90 / 0.45（離地） | 0.50 / 0.25（`engine_*_z_ground`） | 現場目視光達未被引擎擋住 → 引擎頂取低於光達 |
| 閘門 | provisional 擋下 | provisional 擋下；measured_coarse 放行但印誤差 | 最終版第 4 節 |
| 蓋章 | 一行 PROVISIONAL | PROVISIONAL / COARSE / DERIVED-RANGE 三行，動態產生 | 最終版第 13 節 |
| 啟動斷言 | rotation / inscribed | 加 `lateral_half_extent` 0.420；加誤差傳遞的 headland 穩定性；加跨參數一致性（6 節） | 最終版第 5、6 節 |
| smoke_test | 61 項 | **62 項**（新增 A6：淨空比較不引用 costmap_inscribed_radius） | 最終版第 5 節 |
| mass 探測 | 95 | 85 | 最終版第 14 節 |
| 敏感度 | blade_width 0.45 / 0.55 | 另加 blade_offset_x 0.35 / 0.40 | 最終版第 14 節 |

自行決定（最終版）：

34. A6 的規則：除了定義它的 `vehicle_geometry.py`，任何 .py / .cpp 程式碼行（非註解）出現 `costmap_inscribed_radius` 就違規 —— 它唯一合法的消費者是 Nav2 costmap，而 costmap 自己從 footprint 算；用名字掃比用語意判斷「是不是淨空比較」可靠。先植入違規檔確認會紅，再對實際程式庫確認是綠（報告 38 節）。
35. 誤差傳遞用區間端點（所有衍生量對輸入單調，端點組合就是精確上下界），不用蒙地卡羅 —— 結果可以手算核對。
36. 蓋章的 COARSE 行裡數字用最短的兩位以上小數（0.84±0.005）；規格範例寫 0.840，數值相同。
37. 引擎方塊的前後中心不設獨立值，取 blade_offset_x —— 規格第 16 節「刀盤直接鎖在引擎垂直曲軸上，兩者同心」。
38. 修 rclpy 的「同一個呼叫點不能換嚴重度」：閘門回傳同時有 info 與 warn 之後，bridge_node 與 geometry_guard 會在第一行 warn 時 ValueError（Phase H 0/5 抓到），改成 warn / info 兩個呼叫點。不是行為改變。
39. 中途曾為了「光達被車體方塊包住」新增 `body_solid_top_z_ground`（碰撞方塊只到原廠車身 0.46），使用者指出根因是兩個目測值矛盾後**撤回**，碰撞方塊回到 body_height，只保留一個來源。
40. 上位機是 Ubuntu 18.04（使用者提供），ROS 2 Humble 需要 22.04 —— **已結案（2026-10-02）**：使用者決定用 chroot（22.04 rootfs，學長的 18.04 / Melodic / 5.16 核心完全不動），寫進 hardware_bringup.md「實機資訊」，從未決清單移除。

---

## 6. 兩個獨立目測值互相矛盾的偵測

**發生了什麼。** `body_height` 0.70 與 `lidar_z_ground` 0.55 是兩次獨立的照片目測。各自看都合理，
放在一起卻表示「光達掃描面在車體方塊裡面」。模擬中 360 條射線全部打在 0.40 ~ 0.80 m 的方塊內壁上，
**光達 360° 全瞎**。

**為什麼危險。** 這類錯誤**不會讓任何既有測試變紅**：Phase A 的 URDF 解析、C3 的 TF 檢查都照樣通過，
只會讓 SLAM 與 costmap 悄悄變差，最後以「覆蓋率怪怪的」「定位會飄」的形式出現，很難追回根因。
唯一能抓到它的是**跨參數的一致性斷言**。

**修正。** 使用者把 body_height 改成 0.50（鋁架甲板），光達在方塊上方 0.05 m；兩個仍是 provisional。
`vehicle_geometry._consistency()` 在啟動時檢查，物理上不可能的組態直接 `GeometryError`，
允許但必須被看見的情況進 `warnings`（印在橫幅與閘門 log 裡）：

| # | 關係 | 處理 | 以前會不會被抓到 |
|---|---|---|---|
| 1 | 光達掃描面不得落在車體方塊高度範圍 [body_ground_clearance, body_height] 內 | **拒絕** | 不會（這次就是） |
| 2 | body_ground_clearance < body_height；0 < engine_bottom < engine_top | 拒絕 | 不會 |
| 3 | 光達不得落在引擎方塊體積內 | 拒絕 | 不會 |
| 4 | 引擎頂高於光達掃描面（光達在引擎前 / 後） | **警告**並報預期遮擋角（2·atan(半寬 / 距離)） | 不會 |
| 5 | lidar_x 在 footprint 前後範圍內 | 拒絕 | 不會 |
| 6 | 引擎方塊、刀盤都在 footprint 內（前後與左右） | 拒絕 | 不會 |
| 7 | 質心 com_x 在後輪軸與腳輪軸之間（否則靜態會翻） | 拒絕 | 不會 |
| 8 | 0 < com_height < body_height | 拒絕 | 不會 |
| 9 | 腳輪外緣（caster_track/2 + 輪寬/2）在車寬內 | 拒絕 | 不會 |
| 10 | 腳輪轉 180° 時輪子前緣超出 footprint 2 × caster_trail | 警告（目前 0.040 m） | 不會 |
| 11 | 車寬包住後輪（wheel_separation + rear_wheel_width ≤ body_width_total） | 拒絕 | 階段 31 已有 |
| 12 | soft_inflation_radius > lateral_half_extent | 拒絕 | 階段 38 前一版 |
| 13 | headland 在量測誤差範圍內唯一 | 拒絕 | 新增（最終版第 6 節） |

紅 / 綠驗證：舊值 0.70 / 0.55 → 拒絕（訊息指出兩個值至少一個錯）；引擎頂 0.60 → 拒絕（光達在引擎體積內）；
引擎頂 0.60 且 lidar_x 0.15 → 放行並警告「預期遮擋 154.9°，車頭方向」；com_x −0.05 → 拒絕；
blade_offset_x 0.80 → 拒絕（刀盤與引擎都超出 footprint）；現值 → 放行，警告只有第 10 項。

Gazebo 對照（demo_lawn 原點，`scan_fov.py`）：

| 組態 | 被遮擋 | 來源 |
|---|---|---|
| 修正前 body 0.70 / 光達 0.55 | **360 / 360 條（全瞎）**，距離 0.40 ~ 0.80 m | 車體方塊（光達在方塊內） |
| 修正後 body 0.50 / 光達 0.55 | **0 條，360° 全開**（< 1.2 m 與 < 2.0 m 兩種門檻都是 0） | 無 |
| 參考：前一版（光達離地 0.855、引擎頂 0.90） | 66 條，−32.6° ~ +32.6° | 引擎 |

---

## 7. 最終版執行過程中的決定

41. C 第 1 趟結束時批次已自動開始 C2；依使用者指示立即停止批次（C2 啟動 10 秒、尚未開始任務，目錄改名 `run2_STOPPED_no_data`），trap 還原 vehicle.yaml / xacro 並逐位元組確認一致。
42. C1 沒有錄 DWB 評估資料（只能在執行當下錄），為了撞牆簡報另跑一趟與 C1 同設定的擷取趟並錄 bag（1.2 GB，只錄簡報需要的 topic，不錄相機）。這是量測，不是改動；碰撞位置與 C1 相差 0.04 m。
43. 完整套件用 `STEPS="A"` 單獨重啟同一支批次腳本（同一個 orig 備份流程），不另寫腳本。
44. `swath_overshoot.py` 在 0 條完成的割草線時 `max()` 空序列例外 —— 結果本來就是「量不到」，未改工具（只記錄）。
45. 撞牆簡報的角點「越界深度」同時報兩種：模擬實際穿透量（0.0004 m，被接觸擋住）與 DWB 選中軌跡的預測穿牆量（0.067 m）—— 前者是物理引擎的結果，後者才回答「控制器打算開到哪裡」。

---

## 8. 「修正 A 引入 B」：rclpy 的嚴重度限制（Phase H 抓到的）

**A（修正）**：最終版第 4 節要求閘門在 measured_coarse 時「放行但印警告」，`check_gate()` 因此改成
同一次呼叫會回傳 info（誤差傳遞報告）與 warn（橫幅、一致性警告）兩種訊息。

**B（引入的錯誤）**：`bridge_node` 與 `geometry_guard` 原本用同一行
`(log.warn if level == 'warn' else log.info)(line)` 印訊息。rclpy 的 logger 會記住每一個**呼叫點**第一次用的嚴重度，
同一個呼叫點換嚴重度就丟 `ValueError: Logger severity cannot be changed between calls.`。
以前閘門每次只回傳一種 level，這一行從來沒有換過嚴重度，所以一直沒事；A 讓它第一次同時出現兩種。

**怎麼被抓到的**：改完之後照「只跑受影響的 Phase」跑 A/B/D/H。Phase H 用 loopback 驅動以 `read_only=false`
啟動 bridge_node —— 這正是第二道閘門會執行的路徑 —— bridge_node 在印第一行 warn 時崩潰，H1 ~ H5 全部 0/5。
沒有 Phase H 的話，這個錯誤會等到實車第一次以 read_only=false 啟動 bridge_node 才出現，
而且看起來像「驅動起不來」，不像「log 寫法」。geometry_guard 的同一個錯誤則會讓模擬與實機 launch 都在第一個節點就失敗。

**修正**：warn / info 分成兩個呼叫點（`if level == 'warn': log.warn(line) else: log.info(line)`），兩個檔案都改。
重跑 Phase H 5/5、閘門的擋下 / 放行 / 強制通行三種情況各驗一次。

**教訓**：改了一個函式的**回傳內容**（不是介面），呼叫端的假設（「每次只有一種 level」）也一起失效。
這類問題只有實際走過呼叫端那條路徑的測試抓得到 —— 單獨測 `check_gate()` 的回傳值不會發現。

---

## 9. 決定 2 / 3、R3（2026-10-02）

46. Phase P 的測試邊界改用 Phase N 的 `TEST_BOUNDARY_CENTER / HALF`（P3 與 P5 共三處），P3 開始前加起點淨空的夾具前提，不成立記 FIXTURE —— 使用者決定 2；P5 的「保留 25 m²」判準不變（大小沒變）。StatusRig 為此多訂閱 /odom（只讀）。
47. R3 判準從寫死 0.5 s 改成 `cmd_vel_timeout + 1 / odom_rate`（0.533 s），兩個值與送給 bridge_node 的參數是同一組常數 —— 使用者指示。這是**判準的修正**：watchdog 本來就以 odom_rate 週期檢查，0.5 s 不是可達成的上限。
48. 新增 B5「周邊環繞有效長度」（第 63 項）：用進版控的 stage29_lawn 地圖跑 map_to_boundary → F2C → manager 自己的切段函式；起點 = 終點的段落算 0。先在舊 headland 0.70 下確認是綠（3 段、有效 100 %），再在現行 headland 1.20 下確認是紅（1 段、有效 0 %）。修法沒有實作（使用者決定 3），所以 B5 目前會紅 —— 那是真的缺陷。
49. perimeter_probe / blade_sweep 的訂閱端先印 READY、shell 等到才起 map_to_boundary：/f2c_boundary 只在收到 /map 時發一次（volatile），固定 sleep 4 s 在 import 比較慢時會錯過（實測一次）。

---

## 10. 「量測工具本身也會說謊」：smoke test 清掉了我的錄製器（O1 第一次重跑）

**發生了什麼**：為了查 O1，我在 smoke test 的 ROS_DOMAIN_ID (77) 上另外起了一個 `ros2 bag record`，
然後跑 `smoke_test.py --phases=O`。O1 照樣失敗、重現了，但 bag 只有 28 KB —— 一筆資料都沒有。

**原因**：smoke test 開始時的防呆會找出「同一個 ROS_DOMAIN_ID 上、不是自己祖先的所有 ROS 行程」並全部清掉
（`smoke_test.py` 的殘留節點檢查），用意是不讓上一趟的殘留污染這一趟。我的錄製器正好符合條件，在測試開始的那一刻就被殺了。
錄製器沒有報錯（被殺的行程不會留下「我被殺了」的訊息），bag 檔也照樣存在 —— 從外面看起來像「錄了，只是那段時間沒有資料」。

**修正**：錄製器延後 40 秒啟動（在清理之後、Gazebo 起來之前），第二次錄到 238 秒、12 MB。

**同一類問題**：這是「測試基礎設施干擾量測」的實例，與下列兩件同屬一類 —— **量測工具本身也會說謊**：
* 階段 32 / 33：`ros2 node list` 讀到 ros2 daemon 的過期快取，C1 偶發失敗（改成 `--no-daemon`）。
* 階段 26：Phase O 夾具在 headland 改變後失效，O1 卻因為判準太鬆一路綠燈超過一天。
三件事的共同點：工具回報了一個「看起來正常」的結果，但它量的不是我們以為的東西。
對策是同一個：每一個新的量測管道，先用一個已知結果的情況確認它真的量得到（這次是看 bag 的筆數，不是只看檔案存在）。

## 11. 決定 3 實作與決定 1-A 實驗（2026-10-02）

50. 環繞轉角偵測：弧長窗口 = body_length_total（1.13 m）、累積轉角門檻 = body_length_total / rotation_swept_radius（1.064 rad ≈ 61.0°），由 vehicle_geometry 推導 —— 使用者的 60° 理由（曲率半徑約 1.08 m ≈ rotation_swept_radius）寫成公式就是這個；掃描表見報告 38.13。
51. 不採用「單步轉角也切」的補強：它能消掉兩個 S 形 / 凹口的漏切，但 61° 時多出 2 段 < 0.5 m 的極短段（過度切段）。兩處漏切記為已知限制。
52. 不變量 ① 實作在 split_perimeter_into_edges（閉合段在離起點最遠的航點切兩段）；不變量 ② 抽成 drop_closed_segments()（行為不變，為了能單獨測）。判定門檻 = nav2 general_goal_checker 的 xy_goal_tolerance，由 launch 傳入（單一來源）。
53. B6 / B7 的對照組用「goal_xy_tolerance = −1」關掉不變量 —— 不需要另寫一份錯誤的實作；正常組乾淨且對照組被抓到才 PASS（與 O3 的 A/B/C 同一個設計）。
54. 決定 1-A 的實驗用一份實驗專用參數檔（`test/tools/align_experiment.sh` 產生），**沒有改 repo 的 nav2_params.yaml**。實驗沒有 manager，量測端自己把 /cmd_vel_nav 轉到 /cmd_vel。

---

## 12. 幾何推導出的規格必須再通過一次「執行器能否達成」的檢查

**規則：幾何推導出的規格必須再通過一次「執行器能否達成」的檢查，否則會訂出系統物理上做不到的數字。**

這一輪犯了兩次同類錯誤，兩次都是使用者的規格、由驗證 / 實驗抓到：

1. **approach_goal_checker 的 3.15 rad 等於不檢查朝向**（決定 1 原提案）。規格只看了「有一個 goal checker 可以重用」，
   沒看它在「目標就在當下位置」這個情境下會怎樣 —— 送出的瞬間就假成功。
2. **對正容忍值 0.048 rad**（由 (headland − rotation_swept_radius) / 3 推導）。幾何上合理，
   但實驗顯示這個模擬的**停止精度約 0.15 rad**：成功之後 1 s 的最終誤差，容忍值 0.03 / 0.05 / 0.10 分別是
   0.16~0.17 / 0.15~0.17 / 0.09~0.12 —— **收緊容忍值沒有讓最終誤差變小，反而變大**（越緊越容易在低速區停滯、
   成功時角速度越大、停下來衝得越遠）。容忍值訂在停止精度以下沒有意義。

檢查的方法：一個由幾何算出來的控制規格（容忍值、門檻、速度），在寫進程式之前，先用最便宜的實驗確認
執行器在那個尺度上「停得住、量得到、重複得了」。這次的實驗只花了十幾分鐘，比直接跑整趟再回頭查便宜得多。

## 13. 決定 1 實作的自行決定事項

55. **對正容忍值 0.10 rad，實驗值不是推導值**（使用者定案）。寫在 nav2_params.yaml 的 align_goal_checker，附依據與「實機必須重新定案」警告。
    幾何推導公式 (headland − rotation_swept)/3 從程式裡移除（它算出 0.048，系統做不到）。
56. **AlignController 的參數不寫在 nav2_params.yaml**，由 navigation.launch.py 在啟動時把 FollowPath 整塊複製、只改 max_vel_x = 0。
    規格寫「其餘參數完全複製」：用程式複製比在 yaml 貼一份可靠 —— 貼一份等於兩份權重，下一次有人調 FollowPath 就會分岔（原則 7）。
    yaml 裡若有人自己寫 AlignController，launch 拒絕啟動。
57. align_goal_checker 的 xy 容忍由 launch 從 approach_goal_checker 注入（規格「沿用 approach_goal_checker 的 0.10」，單一來源）。
58. 啟動警告由 navigation.launch.py 印（每次起 Nav2 都會看到）：lateral(0.10) = 0.515 m，超出 lateral_half_extent 0.095 m，佔轉彎餘裕 0.1384 m 的 **69 %**。
    （使用者訊息裡的 lateral(0.12) = 0.534 / 82 % 是以 0.12 計算；容忍值實際是 0.10。若以實驗的最終誤差 0.12 計，就是 82 %。）
59. **approach 段落也先對正**：規格寫「每段開始前」；C1 撞牆之後的兩次 approach 也是邊走邊轉撞牆（失敗模式 7）。
60. 對正期間 MissionStatus.current_label = 「對正 → <段落>」：coverage_budget / corner_offset 用 label 前綴歸類，對正期間的原地旋轉不會被算進割草線或周邊環繞。
61. 查不到車子位置時不對正、直接送那一段（與 maybe_insert_approach 的既有處理一致：警告並放行）。
62. 對正失敗（ABORTED）在 log 標「⏱️ 對正超時」，跳過清單的原因寫 ALIGN_TIMEOUT；之後完全走既有的跳過邏輯與 max_consecutive_failures，沒有新增機制。
    急停 / 離開 mode 1 時的取消走原本的取消分支，行為與訊息不變（安全機制不動）。
63. N7 / N8 放在 Phase N（有 Gazebo + Nav2 + manager）：對 AlignController 直接送 goal，用 mode 3 讓 manager 轉發 /cmd_vel_nav（mode 3 不規劃任務）。
    對照組 = 同一個 goal 改用 approach_goal_checker（yaw 3.15）—— 不需要另一份設定就能植入違規；它必須被 N7 的檢查抓到，N7 才算 PASS。
64. 死區診斷（決定 1-D）多跑了一次「腳輪預先轉向」的對照，用來檢驗「死區 = 腳輪換向卡住」的假說 —— 假說被推翻（預先轉向後死區反而更大），兩條曲線都記錄。

## 14. 限制的層級：對正參數是在不可信的模擬低速行為下調出來的

**對正控制器的參數是在「模擬的低角速度行為」下調出來的，而該行為已知不可信：**

* **低速死區**：直接對 /cmd_vel 下固定 wz（繞過 DWB）時，從靜止起轉的最小命令在 0.02 ~ 0.28 rad/s 之間，**取決於前一個動作**（同一個命令值，不同時刻量到的實際角速度可以差 10 倍以上）。
* **停不住**：controller 送了零，車子仍以 0.19 rad/s 轉 1 s 以上。
* **旋轉中心漂移**：原地旋轉時後輪軸平移 0.07 ~ 0.10 m。

三者都是模擬模型（ODE 的輪地接觸、腳輪關節）的產物。**實車的控制板有編碼器閉環，PID 會推過靜摩擦，很可能沒有這個死區** ——
也就是說，**模擬在低速旋轉上是悲觀的**。

因此：
* 對正方案的**機制**是對的，可以實作（已實作）。
* **0.10 rad 這個值不是定案值**，不是設計值；它是「這個模擬能做到的最好」。
* 凍結條件「等實機原地旋轉角速度」現在有**第二個理由**：它不只決定 headland，也決定對正階段的可行性與容忍值。
65. 決定 1-C 的車角判準（前 2 m 最大側向偏移 < 0.558 m）**無法判定**：C1 在第一條割草線開始之前就因連續 3 次對正超時中止，沒有任何割草線取樣。沒有調整容忍值（使用者指示）。
66. O1 對正之後的新失敗原因 = Phase O 場地公式缺「approach 直線離箱子」的條件（實測 0.336 m < 0.42）。是夾具缺陷，只記錄、沒有改 o_min_half（使用者指示 O1 查明不修）。
67. 完整套件的等待腳本這次在 1 h 上限被停掉，但套件其實 16:57 就跑完了：batch_A.log 含有 grep 判成二進位的字元，`grep -q` 的比對失效。結果以 suite_PROVISIONAL.out 為準（62/67）。—— 又一個「量測工具說謊」的小實例（10 節）。

---

## 15. 量測工具本身也會說謊（四個案例與一條常規）

四個案例的形狀完全相同：**看起來在量，實際上量的不是你以為的東西。** 最後一欄說明「先驗證管道」這條常規能省多少時間。

| 案例 | 看起來在量什麼 | 實際量到什麼 | 怎麼被發現 | 多久才發現 |
|---|---|---|---|---|
| ros2 daemon 快取（階段 32，C1） | C1：所有節點是否都在線（`ros2 node list`） | domain 77 的 daemon 從 00:57 起的舊圖形快取：列出已結束的節點、重複列節點，也可能漏掉新節點 | 03:43 第 3 趟跑到 Phase D 時當場對照「經過 daemon」與 `--no-daemon` 兩次查詢（`test/logs/stage32/daemon_diag.txt`） | 約 2 h 46 min；6 趟批次中 3 趟 C1 紅了才去查。用 `--no-daemon` 對照一次就看得出差異 |
| Phase O 夾具（階段 26；階段 38 再補一條） | O1：割草線被箱子擋住時 manager 是否正確跳過 | headland 改成 0.70 之後，周邊環繞直接撞進箱子，O1 要測的情境根本沒發生；階段 38 又發現第一段 approach 直線擦過箱角（0.336 m < 0.42），O1 量到的是 approach 的產品缺陷 | 逐段計算佇列與箱子的距離（26.4、38.15） | 綠燈超過一天；approach 那條在對正方案改變路徑形狀之後才浮現 |
| 監看腳本的 grep（階段 38） | 等待腳本：套件跑完了沒 | log 裡的字元讓 grep 把檔案當成二進位，`grep -q` 永遠比對不到，等待腳本只會等到超時 | 直接看輸出檔的時間戳：套件 16:57 就跑完了 | 1 h（等待腳本超時）。先拿一份已知跑完的 log 試一次就會發現 |
| 參數注入（階段 38 後續 4，min_speed_theta 掃描） | AlignController 的 min_speed_theta 設成 0.2 / 0.3 / 0.4 時對正的收斂行為 | 三組都是 min_speed_theta = 0.0：navigation.launch.py 從 repo 的 FollowPath 整塊複製出 AlignController，用排在後面的暫存參數檔蓋掉實驗值 | 三組前兩趟「最後 2 s 平均 wz 指令」到小數第 4 位完全相同（0.0616 / 0.1895）→ 讀 navigation.launch.py 確認注入蓋掉實驗值。**更正（後續 5 對照）**：當時另一條論據「0.06 的指令在 min_speed_theta = 0.3 下依 DWB 規則不可能出現」是錯的，見下方註 | 15 趟、約 6 分鐘的模擬（18:14 ~ 18:19）加上讀結果；做一次對照（設 0.3，看轉不動時的指令是否 ≥ 0.3）第 1 趟、約 2 分鐘就會發現 —— 而且會同時發現下方註的第二層問題 |

**註：第 4 案例其實有兩層，對照才抓到第二層。** 修好注入之後做的對照（min_speed_theta = 0.3、1 趟，`test/logs/stage38/align_ctrl/`）：
啟動 log 確認 0.3 有進到節點（「保留參數檔的設定：min_speed_theta=0.3」），但轉不動時的 wz 指令**仍然是 0.0616** —— 對照失敗。
讀 DWB Humble 原始碼 `dwb_plugins/src/xy_theta_iterator.cpp` 的 `isValidSpeed()`：取樣被剔除的條件是
`平移² < min_speed_xy²` **而且** `|θ| < min_speed_theta`，兩者同時成立；`min_speed_xy = 0` 時前者永遠不成立，
**min_speed_theta 單獨設定完全沒有作用**。我之前依標頭檔註解判斷「0.06 不可能出現」是讀錯了。
也就是說：即使沒有注入 bug，那 15 趟也會量到同樣的數字。
第二次對照（實驗檔另設 `min_speed_xy = 0.01`，AlignController 的 max_vel_x = 0 所以平移永遠 < 0.01）：轉不動時的 wz 指令變成 **0.308** —— 參數生效，管道驗證通過。

（另外兩個同類的小例子：smoke test 開場清理殺掉另外起的 bag 錄製器 —— bag 檔存在但沒有資料（10 節）；
等待條件寫錯導致「看起來沒事」的監看。）

**常規（已寫進 CLAUDE.md）：任何參數掃描開始之前，先做一次對照，證明該參數確實生效**
（設一個極端值確認行為明顯改變，或確認某個原本會出現的現象消失）。**管道未經驗證之前量到的數字一律不算數。**
更一般的說法：判讀任何自動化結果之前，先確認量測管道本身是好的。具體做法：
* 新的量測管道（工具、錄製、等待腳本、夾具、參數注入）第一次用時，先拿一個**已知答案**的情況跑一次
  （例：bag 看筆數不是看檔案存在；測試先植入違規確認會紅，如 A6 / B5 / B6 / B7 / N7 的對照組）。
* 「沒有輸出」「超時」「綠燈」「三組數字一樣」都是結果，不是證據 —— 先問「這個工具這次真的量到了東西嗎」。
* 自動化結果和直覺不一致時，先懷疑管道，再懷疑系統。

---

## 16. AlignController 注入機制改成「只寫自己負責的鍵、覆寫一律出聲」（階段 38 後續 5）

| # | 決定 | 理由 |
|---|---|---|
| 68 | 注入負責的鍵 = `critics`、所有 `<Critic>.<參數>`（權重與 critic 參數）、`max_vel_x`（= 0）、`xy_goal_tolerance`（= general_goal_checker） | 「兩個控制器的權重只有一份來源」是原設計的目的，保留。`xy_goal_tolerance` 原本就由注入提供（與 FollowPath 同一來源），一起歸注入管 |
| 69 | 其他鍵（速度 / 加速度 / 取樣 / min_speed_theta ...）：參數檔的 AlignController 有寫就用它的，沒寫才從 FollowPath 繼承 | 使用者指示「不在集合內的鍵不得覆寫」。繼承不算覆寫：參數檔沒寫的鍵本來就沒有值可以被蓋 |
| 70 | 注入值與參數檔既有值不同時印 WARN「注入覆寫 align_controller.<鍵>：<舊值> → <新值>」；參數檔沒寫的鍵，「既有值」取它會從 FollowPath 繼承的值 | 這樣正常情況下也一定會印 `max_vel_x 0.7 → 0.0`，覆寫永遠看得見。全部列出 39 個繼承鍵反而會淹沒真正的覆寫 |
| 71 | 參數檔 AlignController 裡被保留的非注入鍵，印一行 INFO「保留參數檔的設定」 | 實驗者能在啟動 log 直接看到自己的設定有沒有進去 —— 本身就是一次管道對照 |
| 72 | 注入改為讀「這次實際傳入的 params_file」（launch 執行時用 OpaqueFunction 解析），不再固定讀 repo 的 nav2_params.yaml | 舊版讀的是 repo 檔，實驗檔的 FollowPath / goal checker 改動對注入完全不可見，是這次 bug 的另一半 |
| 73 | 移除「nav2_params.yaml 不得自行設定 AlignController」的斷言；同一原則套用到 FollowPath / align_goal_checker 的 `xy_goal_tolerance`：被注入蓋掉時也印 WARN | 有了「只寫負責的鍵 + 覆寫出聲」之後，參數檔寫 AlignController（例如決定 1 定案後的 min_speed_theta）是合法的。repo 檔對 xy_goal_tolerance 的禁止斷言不動 |
| 74 | 離線驗證（不開 Gazebo）：repo 檔 → 只印 `max_vel_x`、`xy_goal_tolerance` 兩行 WARN；實驗檔設 `min_speed_theta: 0.3` 並植入 `PathAlign.scale: 999` → min_speed_theta 保留（INFO）、999 被還原成 32 並印 WARN | 植入違規的對照：注入該蓋的蓋得掉、不該蓋的蓋不掉，兩邊都要成立 |

---

## 17. 一個看起來是邊界、實際上不是邊界的東西

這一節和第 15 節是**兩個不同的模式**：第 15 節是「量測工具說謊」—— 我們以為在量 X，實際量到 Y；
這一節是「防護邊界有洞」—— 我們以為某個東西擋住了 X，實際上它只擋住 X 的一部分，其餘照樣穿過去。

| 項目 | 看起來保護了什麼 | 實際保護範圍 | 沒被發現的話後果是什麼 | 怎麼被發現的 |
|---|---|---|---|---|
| DWB 的 `BaseObstacle` critic | 候選軌跡會不會讓車子碰到障礙物 | 只查軌跡上每個位姿**中心點**那一格的 costmap 值；車身其餘部分 (車頭伸出 0.975 m) 碰到什麼它不知道 | 割草線起步時邊走邊轉的弧線讓車頭掃進牆，BaseObstacle 給 0 分照選 (DWB 自己預測的軌跡穿牆 0.067 m 仍被選中)；實車上就是車頭撞牆 | 階段 38 的中止全部都是這個：錄 bag 對照「costmap 在車頭角的值 99」與「BaseObstacle 分數 0」(`stage38_collision_brief.md`) |
| manager 的 `build_approach_path()` | approach 直線從車子到下一段起點這一路的淨空 | 只檢查**終點**能不能原地掉頭 (`approach_target()` / `lead_in_point_unsafe`)；中間整條線不查邊界、不查障礙物 | 直線擦過障礙物角落時車身側面直接撞上；舊版邊走邊轉的弧線「剛好繞開」，與階段 32 的 2 cm 餘裕同一種運氣 | 對正方案把路徑改成「先轉再直走」之後 O1 失敗，逐段算出 approach 直線離箱角 0.336 m < 0.42，再讀程式碼確認 (38.15) |
| chroot 的 `--rbind` 掛載 | 把 chroot 的影響關在 rootfs 裡面 | rbind 進來的 `/dev`、`/run` 預設跟主機是 **shared** 傳播：在 chroot 這邊 `umount` 會傳播回主機，卸掉主機真正的掛載 | 實驗室那台小電腦 (學長的 18.04，不可取代的參照系統) 執行 `exit-humble.sh` 時，`umount -R` 會連帶卸掉主機的 `/dev/pts` 等；終端機開不起來、圖形環境出問題，要重新開機 | 第一次打包 (19:09) 第 9 階段 `umount rootfs/dev/pts` 回報 busy —— 它其實是在卸主機的 `/dev/pts`，只因為主機上有終端機開著才失敗。我們運氣好 |

**第三項特別註明：這是【防護機制本身】有洞。** 我們為了防 `rm -rf` 沿著 bind mount 刪掉主機的 `/dev`，
在 `remove-humble.sh` 加了「還有掛載就拒絕刪除」的檢查 —— 那個檢查是對的，但主機還是會從**另一條路**
(卸載傳播) 受傷，而且正好發生在大家以為最安全的那一步「先執行 exit-humble.sh 卸載」。
**加了檢查不等於問題解決了**：檢查只擋住了我們想到的那條路。

處理 (2026-10-02)：
* `enter-humble.sh`、`enter_build.sh`、`build_rootfs.sh`：每個 `--rbind` 後面緊接 `mount --make-rslave` (主機的變化傳得進來，chroot 這邊的卸載傳不出去)。
* `exit-humble.sh`：卸載**之前**先對每個掛載點做一次 `--make-rslave` (明天現場可能有舊版腳本建立的 shared 掛載)。
* **主機掛載的前後驗證** (把「靠運氣發現」變成「腳本會講」)：`enter-humble.sh` 進入前把主機掛載清單寫進
  `/tmp/.mowerbot_host_mounts_before`；`exit-humble.sh` 卸載後重新比對，主機原本有、現在沒有的掛載點會用醒目警告列出，
  並說明「主機的掛載被連帶卸載了，請重新開機。這不會損壞硬碟上的資料。」；完全一致時印一行確認。
  `build_rootfs.sh` 第 9 階段做同樣的比對，不一致就不打包。--make-rslave 「應該」能防住，但「應該」不夠。
  比對邏輯做過對照：植入一個消失的掛載點會紅 (rc 1)、完全一致會綠 (rc 0)。
* `remove-humble.sh` 不動：它的掛載檢查是對的。

---

## 18. 今晚自己決定的事（階段 38 後續 6：讓模擬跑完並錄影）

| # | 決定 | 理由 |
|---|---|---|
| 75 | AlignController 加 `min_speed_xy: 0.01`（使用者原文寫進註解）+ 啟動斷言：設了 min_speed_xy 時 max_vel_x 必須 = 0，否則 abort | 使用者批准 A。斷言做過對照：正常設定通過；把注入的 max_vel_x 改成 0.3 → abort 並說明原因 |
| 76 | 第一層只跑對正時每個值跑 5 趟，不是 3 趟 | 判準是「>= 4/5」，3 趟判不出來 |
| 77 | 完整 C1 從最寬鬆的 0.25 開始往緊的試，第一個四項全過的就是答案 | 使用者要「最寬鬆的通過值」 |
| 78 | 0.20 不跑完整 C1 | 0.25 與 0.15 都在 ③ 失敗，拆解顯示 ③ 由起步側向差主導（偏航 = 0 也超過），0.20 的偏航比 0.15 大，不可能比 0.15 好 |
| 79 | 碰撞改用幾何量法（新工具 footprint_collision.py），舊的「頂住 > 2 s」一起報 | 舊量法在已知撞牆趟回報 0 次 —— 它漏抓真碰撞。新工具先用已知紅 / 已知綠兩趟做過管道對照 |
| 80 | 【量測方式修正】corner_offset.py 預設把軌跡用 map→odom 轉到 map 座標再算（`--frame odom` 保留舊量法） | 軌跡是 odom 座標、割草線是 map 座標，兩者差約 0.29 m。對照：割草線中段 \|側向\| 0.300 → 0.110 m，反向轉換 0.500 m。判準 0.558 不變 |
| 81 | 先錄影、後做第二層 | 錄影是今晚真正的交付物；有一趟 DONE、無碰撞的設定 (0.25) 時先把它錄下來，避免時間不夠 |
| 82 | 錄影用 RViz 螢幕錄影 + 由紀錄重畫的俯視動畫，不開 gzclient | 沒有 ffmpeg（要 sudo），改 Pillow + OpenCV；gzclient 在這台 VM 會閃退並拖垮整組（demo.launch.py 記載） |
| 83 | 第二層只跑 tol 0.10 的對正（0/5），沒有在 0.15 ~ 0.25 重跑 | 第二層的目的是讓更緊的容忍值收斂；0.10 都收斂不了，而 ③ 的失敗與旋轉無關 |
| 84 | **第三層（AlignController 角加速度 2.0 / 3.0）沒有執行** | 它是凍結清單的例外，只在「可能有幫助」時才值得動用；③ 的失敗來自起步位置（APPROACH_SKIP_DISTANCE），角加速度碰不到；①②④ 已經達成 |
| 85 | **第四層（demo_lawn_small）沒有跑**；世界檔 `demo_lawn_small.world` 已做好但**未驗證**（沒建圖、沒跑過） | 第四層是為了「大場地跑不完」；實測大場地已經 DONE（p = 0.991）。③ 是每一條割草線起步的問題，場地大小碰不到。p^n 曲線改用實測的 p 算 |
| 86 | repo 的 `align_goal_checker.yaw_goal_tolerance` **維持 0.10**，不改成展示用的 0.25 | N7 的判準直接讀這個值，改成 0.25 等於把 N7 從 0.10 放寬到 0.25 —— 放寬測試標準，不得碰 |
| 87 | repo 的 `AlignController.min_speed_theta` 設回 **0.0**（不作用，行為與加入前完全相同）；min_speed_xy 開關與斷言保留 | 在 repo 的 0.10 下，0.3 收斂 1/5、0.5 收斂 0/5，都未達使用者訂的採用標準 (>= 4/5)，且比不設 (9/15) 差。展示設定用 `ALIGN_MST=0.3 test/tools/stage38_demo_run.sh 0.25` 重現 |
| 88 | `ab_run.sh` 加 `AB_DEMO_ARGS`、`coverage_run.py` 加 `COV_MAP_SIDE_S`（預設值行為不變） | 錄影要開 RViz；小場地建圖的方塊要縮小。都是測試工具 |

**提案（未做，等使用者決定）**：③ 要過，需要讓每條割草線從自己的起點出發。可選：
(a) `APPROACH_SKIP_DISTANCE` 從 0.5 降到割草線間距以下（例如 0.15），每次掉頭多一段短 approach（多兩次對正，時間變長）；
(b) 割草線之間改用規劃好的轉彎路徑。兩者都是 CLAUDE.md 凍結的「跟掉頭有關的邏輯」。

### 18 節補充（同晚，使用者改訂判準之後）

| # | 決定 / 紀錄 | 理由 |
|---|---|---|
| 89 | **驗收第 3 項改訂（使用者決定）**：由「前 2 m 車頭角相對割草線中心線的側向偏移 < 0.558 m」改成「車身四角到任何牆面 / 障礙物的最小幾何距離 >= 0.15 m」，用 `footprint_collision.py` 量。舊的偏移量保留為**覆蓋品質指標**，不再當安全判準 | 草坪中間的割草線兩側都是草，偏離中心線 0.87 m 只是跑到隔壁那條線上 —— 那是覆蓋品質，不是安全。理由獨立於結果成立 |
| 90 | **參數之間互相矛盾、但沒有任何檢查在盯的例子**：`APPROACH_SKIP_DISTANCE = 0.5`（離下一段起點 0.5 m 內不插 approach）大於割草線間距 0.30 m，所以相鄰割草線之間 approach **永遠不會插入**，每條割草線都帶著約 0.30 m 的側向差起步。兩個值各自看都合理，放在一起就讓一段邏輯永遠不會執行，而沒有任何東西會告訴你 | 加啟動檢查：`APPROACH_SKIP_DISTANCE >= 割草線間距` 時 manager 印 WARN（先不 abort：現有設定就是這個情況，abort 會起不來）。對照：間距 0.30 / 0.50 → 印；間距 0.60 → 不印。常數本身**未改** |
| 91 | 新判準的「95 百分位」報兩種：每一筆取樣最近距離的**第 5 百分位**（95 % 的時間比它遠）、以及**每一段最小距離**的分布與低於門檻的段數 | 使用者要知道 0.088 m 是個案還是常態，只看單一百分位分不出來 |

### 第三次訂錯代理指標（使用者的自我更正）

| 次 | 代理指標 | 它量的是 | 真正該量的 |
|---|---|---|---|
| 1 | 內接半徑（costmap_inscribed_radius） | 車身最窄處能不能塞進去 | 整個 footprint 會不會碰到東西 |
| 2 | yaw 容忍值 | 對正停在多準 | 起步後車身有沒有超出安全範圍 |
| 3 | 偏離割草線中心線 | 循跡跟計畫差多少 | 車身離牆 / 障礙物多遠 |

**通則：安全判準必須直接量「離危險多遠」，不要量「偏離計畫多遠」。**
偏離計畫可以是覆蓋品質指標，但它和安全之間隔著「計畫本身離危險多遠」這一層假設，那一層一變，指標就失效。
