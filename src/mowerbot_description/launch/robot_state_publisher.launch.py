import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from mowerbot_description import vehicle_geometry

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

    # 讀取並處理 Xacro (指向你的主檔案 car.xacro)。
    # 衍生幾何量 (軸距、前後伸出量、側向半寬) 由 vehicle_geometry 算好當 xacro 參數傳入 (階段 38)，
    # 載入時也一併做完所有幾何斷言 (含階段 31「車寬要包住後輪」，改用 vehicle.yaml 的 rear_wheel_width)。
    geom = vehicle_geometry.load()
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings=geom.xacro_mappings())
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
