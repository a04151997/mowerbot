# mowerbot_ws/ (工作空間根目錄)
# └── src/
#     ├── mowerbot_bringup/          <-- 整合啟動包 (MENT_PYTHON)
#     │   ├── launch/
#     │   │   ├── full_system.launch.py   <-- 總開關：一次啟動 Teleop, Manager, RSP, Lidar
#     │   │   └── gazebo_sim.launch.py    <-- 啟動模擬環境
#     │   ├── config/
#     │   │   ├── teleop_params.yaml      <-- 存放 scale、按鍵對應參數
#     │   │   └── mower_manager.yaml      <-- 存放 Watchdog 超時時間等參數
#     │   ├── package.xml
#     │   └── setup.py                    <-- 關鍵：需配置 data_files 包含 launch/ 和 config/
#     │
#     ├── mowerbot_action/           <-- 決策包 (AMENT_PYTHON)
#     │   ├── mowerbot_action/
#     │   │   ├── manager.py              <-- 大腦節點：話題仲裁 (Mux), Watchdog
#     │   │   └── __init__.py
#     │   ├── package.xml                 <-- 依賴: mowerbot_interfaces, geometry_msgs
#     │   └── setup.py
#     │
#     ├── mowerbot_bridge/           <-- 硬體/通訊橋接包 (AMENT_PYTHON 或 AMENT_CMAKE)
#     │   ├── mowerbot_bridge/
#     │   │   ├── teleop_node.py          <-- 手把節點：LB組合鍵邏輯 (你剛寫好的)
#     │   │   ├── motor_bridge.py         <-- (待寫) 對接實體馬達驅動器的節點
#     │   │   └── __init__.py
#     │   ├── package.xml                 <-- 依賴: geometry_msgs, sensor_msgs
#     │   └── setup.py
#     │
#     ├── mowerbot_description/      <-- 模型描述包 (AMENT_CMAKE)
#     │   ├── urdf/
#     │   │   └── mowerbot.urdf.xacro     <-- 機器人物理模型             
#     │   ├── launch/
#     │   │   └── upload_robot.launch.py   <-- 啟動 RSP 載入模型
#     │   └── package.xml
#     │
#     └── mowerbot_interfaces/       <-- 自定義介面包 (AMENT_CMAKE)
#         ├── srv/
#         │   └── SetDriveMode.srv        <-- 請求: int8 mode, 回應: bool success
#         └── package.xml