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

    # 啟動 slam_toolbox 節點
    start_async_slam_toolbox_node = Node(
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
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
        start_async_slam_toolbox_node
    ])