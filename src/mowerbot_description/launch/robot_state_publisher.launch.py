import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 取得套件路徑
    pkg_path = get_package_share_directory('mowerbot_description')

    # 宣告 use_sim_time 參數 (讓其他 launch 檔可以透過 launch_arguments 傳入)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 讀取並處理 Xacro (指向你的主檔案 car.xacro)
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # 定義發布器節點
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher
    ])
