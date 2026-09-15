import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """只啟動 RViz2 並載入 mowerbot.rviz。

    與 display.launch.py 的差別：這支不會啟動 robot_state_publisher 與
    joint_state_publisher，專門給「模擬或實車已經在跑」的時候開來看畫面用。
    display.launch.py 會自己起一份 robot_state_publisher，跟 gazebo.launch.py
    起的那份同時對 /tf 與 /robot_description 發布相同內容，會產生大量
    TF_REPEATED_DATA 警告。

    Fixed Frame 是 map，所以要先有 SLAM (建圖或定位模式) 在發布 map -> odom。
    """
    pkg_share = get_package_share_directory('mowerbot_description')
    default_rviz = os.path.join(pkg_share, 'rviz', 'mowerbot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Full path to the RViz config file to load'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
