# -*- coding: utf-8 -*-
"""對正控制器實驗用 (階段 38 決定 1-A)：在指定位姿 spawn 車子 + navigation (實驗用參數檔)。

只用在 test/tools/align_experiment.sh。不是產品 launch：沒有 SLAM、沒有 manager。
FollowPath 的路徑用 odom 座標，所以不需要 map。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('mowerbot_bringup')
    pkg_description = get_package_share_directory('mowerbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    x, y, yaw = (LaunchConfiguration(k) for k in ('x', 'y', 'yaw'))
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world', default_value='demo_lawn.world'),
        DeclareLaunchArgument('params_file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': PathJoinSubstitution(
                [pkg_bringup, 'worlds', LaunchConfiguration('world')]), 'gui': 'false'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
             arguments=['-topic', 'robot_description', '-entity', 'mowerbot',
                        '-x', x, '-y', y, '-z', '0.03', '-Y', yaw]),
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')),
            launch_arguments={'use_sim_time': 'true',
                              'params_file': LaunchConfiguration('params_file')}.items())]),
    ])
