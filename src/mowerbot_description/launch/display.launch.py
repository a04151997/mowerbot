import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'mowerbot_description'
    
    # 1. 宣告參數：讓這個 Launch 檔可以接收 use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # 預設改為 true，因為你現在都在模擬器跑
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. 取得 xacro 檔案路徑並解析
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # RViz 設定檔 (Fixed Frame = map，含覆蓋路徑/邊界/costmap 等 display)
    rviz_config = os.path.join(get_package_share_directory(pkg_name), 'rviz', 'mowerbot.rviz')

    # 3. 定義節點
    return LaunchDescription([
        declare_use_sim_time,

        # 1. 必備節點：Joint State Publisher (負責處理 continuous 關節)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 2. 機器人狀態發布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config,
                'use_sim_time': use_sim_time
            }]
        ),

        # 3. RViz2 (載入 mowerbot.rviz)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])