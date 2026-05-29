import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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
        parameters=[{'deadzone': 0.05, 'use_sim_time': use_sim_time}]
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
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 模式管理與 Watchdog
    manager_node = Node(
        package='mowerbot_action',
        executable='mower_manager',
        name='mower_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. 回傳 LaunchDescription
    return LaunchDescription([
        # 【核心修正】全域設定模擬時間參數
        SetParameter(name='use_sim_time', value=use_sim_time),

        robot_description_launch,
        joint_state_publisher,
        slam_launch,
        joy_node,
        teleop_node,
        manager_node,
        boundary_node,
    ])