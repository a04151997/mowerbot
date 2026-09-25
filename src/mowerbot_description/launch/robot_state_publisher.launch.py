import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

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

    # 斷言 footprint_width >= wheel_separation + 輪寬 (階段 31)：footprint 必須把後輪整個包住。
    # 不用算的而用斷言：footprint_width 本身是要實測的值 (輪外緣到輪外緣)，
    # 由它與「輪距 + 輪寬」兩條獨立量測互相核對；而且車殼或刀盤座比輪子寬時
    # footprint 本來就該比較寬，用等式算反而會把它蓋掉。
    # 輪寬還在 car_wheels.xacro (沒有進 vehicle.yaml)，所以從展開後的 URDF 讀後輪碰撞圓柱的長度。
    with open(os.path.join(pkg_path, 'config', 'vehicle.yaml')) as fh:
        vehicle = yaml.safe_load(fh)
    wheel_width = None
    for link in robot_description_config.getElementsByTagName('link'):
        if link.getAttribute('name') == 'left_rear_wheel':
            collision = link.getElementsByTagName('collision')[0]
            cylinder = collision.getElementsByTagName('cylinder')[0]
            wheel_width = float(cylinder.getAttribute('length'))
    wheel_span = vehicle['wheel_separation'] + wheel_width
    # 1e-9 只吸收浮點誤差 (0.58 + 0.1 = 0.6799999999999999)，不是量測容忍
    if vehicle['footprint_width'] < wheel_span - 1e-9:
        raise RuntimeError(
            'vehicle.yaml 的 footprint_width = %.4f m 比後輪外緣寬度還窄：'
            'wheel_separation %.4f m + 輪寬 %.4f m (car_wheels.xacro) = %.4f m。'
            'footprint 沒有包住輪子，淨空檢查與 costmap 會低估車寬。'
            % (vehicle['footprint_width'], vehicle['wheel_separation'],
               wheel_width, wheel_span))
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
