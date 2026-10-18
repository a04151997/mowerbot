import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('mowerbot_bringup')
    
    # 獲取參數檔路徑
    params_file = os.path.join(pkg_path, 'config', 'mapper_params.yaml')

    # 定義是否使用模擬時間 (在 Gazebo 裡必須為 True)
    use_sim_time = LaunchConfiguration('use_sim_time')
    # 光達話題：預設 /scan (mapper_params.yaml)。實車按 LB + A 才開始建圖時是 /scan_slam，
    # 由 slam_scan_gate 轉送 (見 mowerbot_action/slam_scan_gate.py)。
    # 用 remap 不用參數：這裡的 dict 參數 launch_ros 寫成 /** 萬用字元，比不過 mapper_params.yaml
    # 裡指名 slam_toolbox 的同名參數 (實測 scan_topic 還是 /scan)。
    scan_topic = LaunchConfiguration('scan_topic')

    # 啟動 slam_toolbox 節點
    start_async_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/scan', scan_topic)],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='slam_toolbox 讀的光達話題。實車按 LB + A 才建圖時由 mower_control 改成 /scan_slam。'),
        start_async_slam_toolbox_node
    ])