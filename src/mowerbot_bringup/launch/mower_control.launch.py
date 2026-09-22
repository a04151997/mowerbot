import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
# 注意：SetParameter 位於 launch_ros.actions 之中
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # 1. 取得各個套件的路徑
    bringup_dir = get_package_share_directory('mowerbot_bringup')
    description_dir = get_package_share_directory('mowerbot_description')
    bridge_dir = get_package_share_directory('mowerbot_bridge')

    # 2. 宣告 Launch 參數
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. 定義節點與包含的 Launch 檔案

    # 機器人描述檔與狀態發布器
    # 模擬時 gazebo.launch.py 已經啟動 robot_state_publisher
    # (spawn_entity 需要從 /robot_description 讀 URDF)，
    # 這裡再啟動一次會出現兩個同名節點同時對 /tf 與 /robot_description 發布相同內容，
    # 產生大量 TF_REPEATED_DATA 警告，所以預設 false；
    # 之後跑實體車 (沒有 Gazebo) 時要用 start_rsp:=true 啟動它。
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('start_rsp'))
    )

    # 關節狀態發布器 (解決前輪不顯示的關鍵)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM 建圖
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 手把硬體驅動
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            os.path.join(bridge_dir, 'config', 'joystick.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 邊界提取節點
    boundary_node = Node(
        package='mowerbot_action',
        executable='map_to_boundary',
        name='map_to_boundary',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 手把控制邏輯
    teleop_node = Node(
        package='mowerbot_bridge',
        executable='teleop_node',
        name='mower_teleop',
        output='screen',
        parameters=[
            os.path.join(bridge_dir, 'config', 'joystick.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # F2C 路徑規劃伺服器 (C++)
    f2c_server_node = Node(
        package='mowerbot_planner',
        executable='f2c_server',
        name='f2c_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 地頭寬度，預設與跑道長度 L 相同，讓跑道剛好落在地頭裡
            'headland_width': LaunchConfiguration('headland_width'),
        }]
    )

    # 模式管理與 Watchdog
    manager_node = Node(
        package='mowerbot_action',
        executable='mower_manager',
        name='mower_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 跑道 (lead-in) 長度，讓實測可以掃不同數值而不用重建
            'lead_in_length': LaunchConfiguration('lead_in_length'),
            # 割草線重疊率，讓實測可以掃不同數值而不用重建
            'overlap_ratio': LaunchConfiguration('overlap_ratio'),
        }]
    )

    # 4. 回傳 LaunchDescription
    return LaunchDescription([
        # 模擬時預設不啟動 robot_state_publisher (由 gazebo.launch.py 負責)
        DeclareLaunchArgument(
            'start_rsp',
            default_value='false',
            description='Start robot_state_publisher. Keep false under Gazebo '
                        '(gazebo.launch.py already starts it); set true on the real robot.'
        ),

        # 每條割草線前面的跑道 (lead-in) 長度，單位公尺，0 代表停用
        DeclareLaunchArgument(
            'lead_in_length',
            default_value='0.5',
            description='Lead-in (headland run-up) length in metres prepended to '
                        'each mowing swath. 0 disables it.'
        ),

        # 割草線重疊率：割草線間距 = blade_width x (1 - overlap_ratio)。
        # 0 代表零重疊 (間距 = 刀盤寬)。
        DeclareLaunchArgument(
            'overlap_ratio',
            default_value='0.4',
            description='Swath overlap ratio. Swath spacing = blade_width * '
                        '(1 - overlap_ratio). 0 means no overlap.'
        ),

        # F2C 地頭 (headland) 寬度，單位公尺，0 代表停用地頭。
        # 階段 21 從 0.5 改成 0.70：地頭要能容納車子原地掉頭，
        # 而外接半徑就有 0.5841 m (見 f2c_server.cpp 的推導)。
        DeclareLaunchArgument(
            'headland_width',
            default_value='0.70',
            description='Headland width in metres. Swaths are generated on the '
                        'field shrunk by this margin. 0 disables the headland.'
        ),

        # 【核心修正】全域設定模擬時間參數
        SetParameter(name='use_sim_time', value=use_sim_time),

        robot_description_launch,
        joint_state_publisher,
        slam_launch,
        joy_node,
        teleop_node,
        f2c_server_node,
        manager_node,
        boundary_node,
    ])