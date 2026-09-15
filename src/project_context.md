.
├── joy_tester
│   └── joy_tester
│       ├── CHANGELOG.rst
│       ├── CONTRIBUTING.md
│       ├── joy_tester
│       │   ├── __init__.py
│       │   └── test_joy.py
│       ├── LICENSE
│       ├── package.xml
│       ├── README.md
│       ├── resource
│       │   └── joy_tester
│       ├── setup.cfg
│       ├── setup.py
│       └── test
│           ├── test_copyright.py
│           ├── test_flake8.py
│           └── test_pep257.py
├── mowerbot_action
│   ├── mowerbot_action
│   │   ├── __init__.py
│   │   ├── manager.py
│   │   └── map_to_boundary.py
│   ├── package.xml
│   ├── resource
│   │   └── mowerbot_action
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── mowerbot_bridge
│   ├── config
│   │   └── joystick_yaml
│   ├── mowerbot_bridge
│   │   ├── bridge_node.py
│   │   ├── __init__.py
│   │   └── teleop.py
│   ├── package.xml
│   ├── resource
│   │   └── mowerbot_bridge
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── mowerbot_bringup
│   ├── config
│   │   └── mapper_params.yaml
│   ├── launch
│   │   ├── gazebo.launch.py
│   │   ├── mapping.launch.py
│   │   └── mower_control.launch.py
│   ├── mowerbot_bringup
│   │   └── __init__.py
│   ├── package.xml
│   ├── resource
│   │   └── mowerbot_bringup
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── mowerbot_description
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── display.launch.py
│   │   └── robot_state_publisher.launch.py
│   ├── package.xml
│   ├── rviz
│   └── urdf
│       ├── car_base.xacro
│       ├── car_camera.xacro
│       ├── car_imu.xacro
│       ├── car_inertia.xacro
│       ├── car_radar.xacro
│       ├── car_wheels.xacro
│       └── car.xacro
├── mowerbot_interfaces
│   ├── action
│   ├── CMakeLists.txt
│   ├── msg
│   │   ├── MotorStatus.msg
│   │   └── MowerStatus.msg
│   ├── package.xml
│   └── srv
│       ├── GenerateCoveragePath.srv
│       └── SetDriveMode.srv
├── mowerbot_planner
│   ├── CMakeLists.txt
│   ├── include
│   │   └── mowerbot_planner
│   ├── package.xml
│   └── src
│       └── f2c_server.cpp
└── project_context.md

32 directories, 67 files


=== FILE CONTENTS ===


--- FILE: ./mowerbot_bringup/setup.py ---
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mowerbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a0987747836@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

--- FILE: ./mowerbot_bringup/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_bringup</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <depend>mowerbot_action</depend>
  <depend>mowerbot_description</depend>
  <depend>joy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

--- FILE: ./mowerbot_bringup/mowerbot_bringup/__init__.py ---

--- FILE: ./mowerbot_bringup/test/test_flake8.py ---
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

--- FILE: ./mowerbot_bringup/test/test_copyright.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'

--- FILE: ./mowerbot_bringup/test/test_pep257.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'

--- FILE: ./mowerbot_bringup/config/mapper_params.yaml ---

slam_toolbox:
  ros__parameters:
    # 核心參數
    use_sim_time: true
    solver_plugin: solver_plugins::CeresSolver
    ceres_loss_function: None
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function_type: HuberLoss

    # 坐標系定義 (與你的 URDF 對齊)
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint # 使用底盤投影點作為基準
    scan_topic: /scan # 訂閱你 radar.xacro 發布的話題

    # 物理與感測器限制
    max_laser_range: 12.0 # 配合你雷達外掛程式的設定
    minimum_time_interval: 0.1 
    transform_timeout: 1.0
    tf_buffer_duration: 30.
    stack_size_to_use: 40000000 # 避免大場地時記憶體崩潰

    # 建圖精度
    resolution: 0.05 # 5公分解析度
    max_range: 12.0
    min_range: 0.15 # 避開車體干擾  
--- FILE: ./mowerbot_bringup/launch/mapping.launch.py ---
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
--- FILE: ./mowerbot_bringup/launch/mower_control.launch.py ---
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
--- FILE: ./mowerbot_bringup/launch/gazebo.launch.py ---
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得相關路徑
    pkg_description = get_package_share_directory('mowerbot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. 定義參數：是否啟動模擬時間 (在 Gazebo 裡必須為 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. 引入 robot_state_publisher (發布 URDF)
    # 這裡直接呼叫你之前寫好的描述檔 launch
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. 啟動 Gazebo 伺服器 (gzserver) 與 客戶端 (gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # 5. 呼叫 gazebo_ros 的節點來「生成」機器人
    # 它會從 robot_description 話題讀取 URDF 並放入 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mowerbot',
                   '-x', '0', '-y', '0', '-z', '0.03'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_state_publisher,
        gazebo,
        spawn_entity
    ])
--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_py/mowerbot_interfaces/__init__.py ---

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_py/mowerbot_interfaces/msg/__init__.py ---
from mowerbot_interfaces.msg._motor_status import MotorStatus  # noqa: F401
from mowerbot_interfaces.msg._mower_status import MowerStatus  # noqa: F401

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_py/mowerbot_interfaces/msg/_mower_status.py ---
# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mowerbot_interfaces:msg/MowerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MowerStatus(type):
    """Metaclass of message 'MowerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mowerbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mowerbot_interfaces.msg.MowerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mower_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mower_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mower_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mower_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mower_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MowerStatus(metaclass=Metaclass_MowerStatus):
    """Message class 'MowerStatus'."""

    __slots__ = [
        '_battery_voltage',
        '_battery_current',
        '_battery_percentage',
        '_bumper_pressed',
        '_stop_active',
        '_is_overheated',
        '_mode',
    ]

    _fields_and_field_types = {
        'battery_voltage': 'float',
        'battery_current': 'float',
        'battery_percentage': 'float',
        'bumper_pressed': 'boolean',
        'stop_active': 'boolean',
        'is_overheated': 'boolean',
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.battery_current = kwargs.get('battery_current', float())
        self.battery_percentage = kwargs.get('battery_percentage', float())
        self.bumper_pressed = kwargs.get('bumper_pressed', bool())
        self.stop_active = kwargs.get('stop_active', bool())
        self.is_overheated = kwargs.get('is_overheated', bool())
        self.mode = kwargs.get('mode', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.battery_current != other.battery_current:
            return False
        if self.battery_percentage != other.battery_percentage:
            return False
        if self.bumper_pressed != other.bumper_pressed:
            return False
        if self.stop_active != other.stop_active:
            return False
        if self.is_overheated != other.is_overheated:
            return False
        if self.mode != other.mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def battery_current(self):
        """Message field 'battery_current'."""
        return self._battery_current

    @battery_current.setter
    def battery_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_current = value

    @builtins.property
    def battery_percentage(self):
        """Message field 'battery_percentage'."""
        return self._battery_percentage

    @battery_percentage.setter
    def battery_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_percentage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_percentage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_percentage = value

    @builtins.property
    def bumper_pressed(self):
        """Message field 'bumper_pressed'."""
        return self._bumper_pressed

    @bumper_pressed.setter
    def bumper_pressed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'bumper_pressed' field must be of type 'bool'"
        self._bumper_pressed = value

    @builtins.property
    def stop_active(self):
        """Message field 'stop_active'."""
        return self._stop_active

    @stop_active.setter
    def stop_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop_active' field must be of type 'bool'"
        self._stop_active = value

    @builtins.property
    def is_overheated(self):
        """Message field 'is_overheated'."""
        return self._is_overheated

    @is_overheated.setter
    def is_overheated(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_overheated' field must be of type 'bool'"
        self._is_overheated = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_py/mowerbot_interfaces/msg/_motor_status.py ---
# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mowerbot_interfaces:msg/MotorStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorStatus(type):
    """Metaclass of message 'MotorStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mowerbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mowerbot_interfaces.msg.MotorStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorStatus(metaclass=Metaclass_MotorStatus):
    """Message class 'MotorStatus'."""

    __slots__ = [
        '_left_front_rpm',
        '_right_front_rpm',
        '_left_behind_rpm',
        '_right_behind_rpm',
        '_left_front_encoder',
        '_right_front_encoder',
        '_left_behind_encoder',
        '_right_behind_encoder',
    ]

    _fields_and_field_types = {
        'left_front_rpm': 'float',
        'right_front_rpm': 'float',
        'left_behind_rpm': 'float',
        'right_behind_rpm': 'float',
        'left_front_encoder': 'int32',
        'right_front_encoder': 'int32',
        'left_behind_encoder': 'int32',
        'right_behind_encoder': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_front_rpm = kwargs.get('left_front_rpm', float())
        self.right_front_rpm = kwargs.get('right_front_rpm', float())
        self.left_behind_rpm = kwargs.get('left_behind_rpm', float())
        self.right_behind_rpm = kwargs.get('right_behind_rpm', float())
        self.left_front_encoder = kwargs.get('left_front_encoder', int())
        self.right_front_encoder = kwargs.get('right_front_encoder', int())
        self.left_behind_encoder = kwargs.get('left_behind_encoder', int())
        self.right_behind_encoder = kwargs.get('right_behind_encoder', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.left_front_rpm != other.left_front_rpm:
            return False
        if self.right_front_rpm != other.right_front_rpm:
            return False
        if self.left_behind_rpm != other.left_behind_rpm:
            return False
        if self.right_behind_rpm != other.right_behind_rpm:
            return False
        if self.left_front_encoder != other.left_front_encoder:
            return False
        if self.right_front_encoder != other.right_front_encoder:
            return False
        if self.left_behind_encoder != other.left_behind_encoder:
            return False
        if self.right_behind_encoder != other.right_behind_encoder:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_front_rpm(self):
        """Message field 'left_front_rpm'."""
        return self._left_front_rpm

    @left_front_rpm.setter
    def left_front_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_front_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'left_front_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._left_front_rpm = value

    @builtins.property
    def right_front_rpm(self):
        """Message field 'right_front_rpm'."""
        return self._right_front_rpm

    @right_front_rpm.setter
    def right_front_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_front_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'right_front_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._right_front_rpm = value

    @builtins.property
    def left_behind_rpm(self):
        """Message field 'left_behind_rpm'."""
        return self._left_behind_rpm

    @left_behind_rpm.setter
    def left_behind_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_behind_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'left_behind_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._left_behind_rpm = value

    @builtins.property
    def right_behind_rpm(self):
        """Message field 'right_behind_rpm'."""
        return self._right_behind_rpm

    @right_behind_rpm.setter
    def right_behind_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_behind_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'right_behind_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._right_behind_rpm = value

    @builtins.property
    def left_front_encoder(self):
        """Message field 'left_front_encoder'."""
        return self._left_front_encoder

    @left_front_encoder.setter
    def left_front_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_front_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_front_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_front_encoder = value

    @builtins.property
    def right_front_encoder(self):
        """Message field 'right_front_encoder'."""
        return self._right_front_encoder

    @right_front_encoder.setter
    def right_front_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_front_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_front_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_front_encoder = value

    @builtins.property
    def left_behind_encoder(self):
        """Message field 'left_behind_encoder'."""
        return self._left_behind_encoder

    @left_behind_encoder.setter
    def left_behind_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_behind_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_behind_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_behind_encoder = value

    @builtins.property
    def right_behind_encoder(self):
        """Message field 'right_behind_encoder'."""
        return self._right_behind_encoder

    @right_behind_encoder.setter
    def right_behind_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_behind_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_behind_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_behind_encoder = value

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_cpp/mowerbot_interfaces/msg/motor_status__type_support.cpp ---
// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _MotorStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorStatus_type_support_ids_t;

static const _MotorStatus_type_support_ids_t _MotorStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MotorStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorStatus_type_support_symbol_names_t _MotorStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MotorStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MotorStatus)),
  }
};

typedef struct _MotorStatus_type_support_data_t
{
  void * data[2];
} _MotorStatus_type_support_data_t;

static _MotorStatus_type_support_data_t _MotorStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorStatus_message_typesupport_map = {
  2,
  "mowerbot_interfaces",
  &_MotorStatus_message_typesupport_ids.typesupport_identifier[0],
  &_MotorStatus_message_typesupport_symbol_names.symbol_name[0],
  &_MotorStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorStatus_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorStatus_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MotorStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_cpp::MotorStatus_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mowerbot_interfaces, msg, MotorStatus)() {
  return get_message_type_support_handle<mowerbot_interfaces::msg::MotorStatus>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_cpp/mowerbot_interfaces/msg/mower_status__type_support.cpp ---
// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _MowerStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MowerStatus_type_support_ids_t;

static const _MowerStatus_type_support_ids_t _MowerStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MowerStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MowerStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MowerStatus_type_support_symbol_names_t _MowerStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MowerStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MowerStatus)),
  }
};

typedef struct _MowerStatus_type_support_data_t
{
  void * data[2];
} _MowerStatus_type_support_data_t;

static _MowerStatus_type_support_data_t _MowerStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MowerStatus_message_typesupport_map = {
  2,
  "mowerbot_interfaces",
  &_MowerStatus_message_typesupport_ids.typesupport_identifier[0],
  &_MowerStatus_message_typesupport_symbol_names.symbol_name[0],
  &_MowerStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MowerStatus_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MowerStatus_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MowerStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_cpp::MowerStatus_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, mowerbot_interfaces, msg, MowerStatus)() {
  return get_message_type_support_handle<mowerbot_interfaces::msg::MowerStatus>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_introspection_cpp/mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_introspection_cpp.hpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__rosidl_typesupport_introspection_cpp.h.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_


#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO(dirk-thomas) these visibility macros should be message package specific
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MotorStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_introspection_cpp/mowerbot_interfaces/msg/detail/motor_status__type_support.cpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mowerbot_interfaces::msg::MotorStatus(_init);
}

void MotorStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mowerbot_interfaces::msg::MotorStatus *>(message_memory);
  typed_message->~MotorStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorStatus_message_member_array[8] = {
  {
    "left_front_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_front_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_front_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_front_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_behind_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_behind_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_behind_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_behind_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_front_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_front_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_front_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_front_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_behind_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_behind_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_behind_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_behind_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorStatus_message_members = {
  "mowerbot_interfaces::msg",  // message namespace
  "MotorStatus",  // message name
  8,  // number of fields
  sizeof(mowerbot_interfaces::msg::MotorStatus),
  MotorStatus_message_member_array,  // message members
  MotorStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MotorStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MotorStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_introspection_cpp/mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_introspection_cpp.hpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__rosidl_typesupport_introspection_cpp.h.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_


#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO(dirk-thomas) these visibility macros should be message package specific
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MowerStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_introspection_cpp/mowerbot_interfaces/msg/detail/mower_status__type_support.cpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MowerStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mowerbot_interfaces::msg::MowerStatus(_init);
}

void MowerStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mowerbot_interfaces::msg::MowerStatus *>(message_memory);
  typed_message->~MowerStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerStatus_message_member_array[7] = {
  {
    "battery_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_voltage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_current),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_percentage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_percentage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bumper_pressed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, bumper_pressed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stop_active",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, stop_active),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_overheated",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, is_overheated),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerStatus_message_members = {
  "mowerbot_interfaces::msg",  // message namespace
  "MowerStatus",  // message name
  7,  // number of fields
  sizeof(mowerbot_interfaces::msg::MowerStatus),
  MowerStatus_message_member_array,  // message members
  MowerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MowerStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MowerStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MowerStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MowerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/CMakeFiles/3.22.1/CompilerIdCXX/CMakeCXXCompilerId.cpp ---
/* This source file must have a .cpp extension so that all C++ compilers
   recognize the extension without flags.  Borland does not know .cxx for
   example.  */
#ifndef __cplusplus
# error "A C compiler has been selected for C++."
#endif

#if !defined(__has_include)
/* If the compiler does not have __has_include, pretend the answer is
   always no.  */
#  define __has_include(x) 0
#endif


/* Version number components: V=Version, R=Revision, P=Patch
   Version date components:   YYYY=Year, MM=Month,   DD=Day  */

#if defined(__COMO__)
# define COMPILER_ID "Comeau"
  /* __COMO_VERSION__ = VRR */
# define COMPILER_VERSION_MAJOR DEC(__COMO_VERSION__ / 100)
# define COMPILER_VERSION_MINOR DEC(__COMO_VERSION__ % 100)

#elif defined(__INTEL_COMPILER) || defined(__ICC)
# define COMPILER_ID "Intel"
# if defined(_MSC_VER)
#  define SIMULATE_ID "MSVC"
# endif
# if defined(__GNUC__)
#  define SIMULATE_ID "GNU"
# endif
  /* __INTEL_COMPILER = VRP prior to 2021, and then VVVV for 2021 and later,
     except that a few beta releases use the old format with V=2021.  */
# if __INTEL_COMPILER < 2021 || __INTEL_COMPILER == 202110 || __INTEL_COMPILER == 202111
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_COMPILER/100)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_COMPILER/10 % 10)
#  if defined(__INTEL_COMPILER_UPDATE)
#   define COMPILER_VERSION_PATCH DEC(__INTEL_COMPILER_UPDATE)
#  else
#   define COMPILER_VERSION_PATCH DEC(__INTEL_COMPILER   % 10)
#  endif
# else
#  define COMPILER_VERSION_MAJOR DEC(__INTEL_COMPILER)
#  define COMPILER_VERSION_MINOR DEC(__INTEL_COMPILER_UPDATE)
   /* The third version component from --version is an update index,
      but no macro is provided for it.  */
#  define COMPILER_VERSION_PATCH DEC(0)
# endif
# if defined(__INTEL_COMPILER_BUILD_DATE)
   /* __INTEL_COMPILER_BUILD_DATE = YYYYMMDD */
#  define COMPILER_VERSION_TWEAK DEC(__INTEL_COMPILER_BUILD_DATE)
# endif
# if defined(_MSC_VER)
   /* _MSC_VER = VVRR */
#  define SIMULATE_VERSION_MAJOR DEC(_MSC_VER / 100)
#  define SIMULATE_VERSION_MINOR DEC(_MSC_VER % 100)
# endif
# if defined(__GNUC__)
#  define SIMULATE_VERSION_MAJOR DEC(__GNUC__)
# elif defined(__GNUG__)
#  define SIMULATE_VERSION_MAJOR DEC(__GNUG__)
# endif
# if defined(__GNUC_MINOR__)
#  define SIMULATE_VERSION_MINOR DEC(__GNUC_MINOR__)
# endif
# if defined(__GNUC_PATCHLEVEL__)
#  define SIMULATE_VERSION_PATCH DEC(__GNUC_PATCHLEVEL__)
# endif

#elif (defined(__clang__) && defined(__INTEL_CLANG_COMPILER)) || defined(__INTEL_LLVM_COMPILER)
# define COMPILER_ID "IntelLLVM"
#if defined(_MSC_VER)
# define SIMULATE_ID "MSVC"
#endif
#if defined(__GNUC__)
# define SIMULATE_ID "GNU"
#endif
/* __INTEL_LLVM_COMPILER = VVVVRP prior to 2021.2.0, VVVVRRPP for 2021.2.0 and
 * later.  Look for 6 digit vs. 8 digit version number to decide encoding.
 * VVVV is no smaller than the current year when a version is released.
 */
#if __INTEL_LLVM_COMPILER < 1000000L
# define COMPILER_VERSION_MAJOR DEC(__INTEL_LLVM_COMPILER/100)
# define COMPILER_VERSION_MINOR DEC(__INTEL_LLVM_COMPILER/10 % 10)
# define COMPILER_VERSION_PATCH DEC(__INTEL_LLVM_COMPILER    % 10)
#else
# define COMPILER_VERSION_MAJOR DEC(__INTEL_LLVM_COMPILER/10000)
# define COMPILER_VERSION_MINOR DEC(__INTEL_LLVM_COMPILER/100 % 100)
# define COMPILER_VERSION_PATCH DEC(__INTEL_LLVM_COMPILER     % 100)
#endif
#if defined(_MSC_VER)
  /* _MSC_VER = VVRR */
# define SIMULATE_VERSION_MAJOR DEC(_MSC_VER / 100)
# define SIMULATE_VERSION_MINOR DEC(_MSC_VER % 100)
#endif
#if defined(__GNUC__)
# define SIMULATE_VERSION_MAJOR DEC(__GNUC__)
#elif defined(__GNUG__)
# define SIMULATE_VERSION_MAJOR DEC(__GNUG__)
#endif
#if defined(__GNUC_MINOR__)
# define SIMULATE_VERSION_MINOR DEC(__GNUC_MINOR__)
#endif
#if defined(__GNUC_PATCHLEVEL__)
# define SIMULATE_VERSION_PATCH DEC(__GNUC_PATCHLEVEL__)
#endif

#elif defined(__PATHCC__)
# define COMPILER_ID "PathScale"
# define COMPILER_VERSION_MAJOR DEC(__PATHCC__)
# define COMPILER_VERSION_MINOR DEC(__PATHCC_MINOR__)
# if defined(__PATHCC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__PATHCC_PATCHLEVEL__)
# endif

#elif defined(__BORLANDC__) && defined(__CODEGEARC_VERSION__)
# define COMPILER_ID "Embarcadero"
# define COMPILER_VERSION_MAJOR HEX(__CODEGEARC_VERSION__>>24 & 0x00FF)
# define COMPILER_VERSION_MINOR HEX(__CODEGEARC_VERSION__>>16 & 0x00FF)
# define COMPILER_VERSION_PATCH DEC(__CODEGEARC_VERSION__     & 0xFFFF)

#elif defined(__BORLANDC__)
# define COMPILER_ID "Borland"
  /* __BORLANDC__ = 0xVRR */
# define COMPILER_VERSION_MAJOR HEX(__BORLANDC__>>8)
# define COMPILER_VERSION_MINOR HEX(__BORLANDC__ & 0xFF)

#elif defined(__WATCOMC__) && __WATCOMC__ < 1200
# define COMPILER_ID "Watcom"
   /* __WATCOMC__ = VVRR */
# define COMPILER_VERSION_MAJOR DEC(__WATCOMC__ / 100)
# define COMPILER_VERSION_MINOR DEC((__WATCOMC__ / 10) % 10)
# if (__WATCOMC__ % 10) > 0
#  define COMPILER_VERSION_PATCH DEC(__WATCOMC__ % 10)
# endif

#elif defined(__WATCOMC__)
# define COMPILER_ID "OpenWatcom"
   /* __WATCOMC__ = VVRP + 1100 */
# define COMPILER_VERSION_MAJOR DEC((__WATCOMC__ - 1100) / 100)
# define COMPILER_VERSION_MINOR DEC((__WATCOMC__ / 10) % 10)
# if (__WATCOMC__ % 10) > 0
#  define COMPILER_VERSION_PATCH DEC(__WATCOMC__ % 10)
# endif

#elif defined(__SUNPRO_CC)
# define COMPILER_ID "SunPro"
# if __SUNPRO_CC >= 0x5100
   /* __SUNPRO_CC = 0xVRRP */
#  define COMPILER_VERSION_MAJOR HEX(__SUNPRO_CC>>12)
#  define COMPILER_VERSION_MINOR HEX(__SUNPRO_CC>>4 & 0xFF)
#  define COMPILER_VERSION_PATCH HEX(__SUNPRO_CC    & 0xF)
# else
   /* __SUNPRO_CC = 0xVRP */
#  define COMPILER_VERSION_MAJOR HEX(__SUNPRO_CC>>8)
#  define COMPILER_VERSION_MINOR HEX(__SUNPRO_CC>>4 & 0xF)
#  define COMPILER_VERSION_PATCH HEX(__SUNPRO_CC    & 0xF)
# endif

#elif defined(__HP_aCC)
# define COMPILER_ID "HP"
  /* __HP_aCC = VVRRPP */
# define COMPILER_VERSION_MAJOR DEC(__HP_aCC/10000)
# define COMPILER_VERSION_MINOR DEC(__HP_aCC/100 % 100)
# define COMPILER_VERSION_PATCH DEC(__HP_aCC     % 100)

#elif defined(__DECCXX)
# define COMPILER_ID "Compaq"
  /* __DECCXX_VER = VVRRTPPPP */
# define COMPILER_VERSION_MAJOR DEC(__DECCXX_VER/10000000)
# define COMPILER_VERSION_MINOR DEC(__DECCXX_VER/100000  % 100)
# define COMPILER_VERSION_PATCH DEC(__DECCXX_VER         % 10000)

#elif defined(__IBMCPP__) && defined(__COMPILER_VER__)
# define COMPILER_ID "zOS"
  /* __IBMCPP__ = VRP */
# define COMPILER_VERSION_MAJOR DEC(__IBMCPP__/100)
# define COMPILER_VERSION_MINOR DEC(__IBMCPP__/10 % 10)
# define COMPILER_VERSION_PATCH DEC(__IBMCPP__    % 10)

#elif defined(__ibmxl__) && defined(__clang__)
# define COMPILER_ID "XLClang"
# define COMPILER_VERSION_MAJOR DEC(__ibmxl_version__)
# define COMPILER_VERSION_MINOR DEC(__ibmxl_release__)
# define COMPILER_VERSION_PATCH DEC(__ibmxl_modification__)
# define COMPILER_VERSION_TWEAK DEC(__ibmxl_ptf_fix_level__)


#elif defined(__IBMCPP__) && !defined(__COMPILER_VER__) && __IBMCPP__ >= 800
# define COMPILER_ID "XL"
  /* __IBMCPP__ = VRP */
# define COMPILER_VERSION_MAJOR DEC(__IBMCPP__/100)
# define COMPILER_VERSION_MINOR DEC(__IBMCPP__/10 % 10)
# define COMPILER_VERSION_PATCH DEC(__IBMCPP__    % 10)

#elif defined(__IBMCPP__) && !defined(__COMPILER_VER__) && __IBMCPP__ < 800
# define COMPILER_ID "VisualAge"
  /* __IBMCPP__ = VRP */
# define COMPILER_VERSION_MAJOR DEC(__IBMCPP__/100)
# define COMPILER_VERSION_MINOR DEC(__IBMCPP__/10 % 10)
# define COMPILER_VERSION_PATCH DEC(__IBMCPP__    % 10)

#elif defined(__NVCOMPILER)
# define COMPILER_ID "NVHPC"
# define COMPILER_VERSION_MAJOR DEC(__NVCOMPILER_MAJOR__)
# define COMPILER_VERSION_MINOR DEC(__NVCOMPILER_MINOR__)
# if defined(__NVCOMPILER_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__NVCOMPILER_PATCHLEVEL__)
# endif

#elif defined(__PGI)
# define COMPILER_ID "PGI"
# define COMPILER_VERSION_MAJOR DEC(__PGIC__)
# define COMPILER_VERSION_MINOR DEC(__PGIC_MINOR__)
# if defined(__PGIC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__PGIC_PATCHLEVEL__)
# endif

#elif defined(_CRAYC)
# define COMPILER_ID "Cray"
# define COMPILER_VERSION_MAJOR DEC(_RELEASE_MAJOR)
# define COMPILER_VERSION_MINOR DEC(_RELEASE_MINOR)

#elif defined(__TI_COMPILER_VERSION__)
# define COMPILER_ID "TI"
  /* __TI_COMPILER_VERSION__ = VVVRRRPPP */
# define COMPILER_VERSION_MAJOR DEC(__TI_COMPILER_VERSION__/1000000)
# define COMPILER_VERSION_MINOR DEC(__TI_COMPILER_VERSION__/1000   % 1000)
# define COMPILER_VERSION_PATCH DEC(__TI_COMPILER_VERSION__        % 1000)

#elif defined(__CLANG_FUJITSU)
# define COMPILER_ID "FujitsuClang"
# define COMPILER_VERSION_MAJOR DEC(__FCC_major__)
# define COMPILER_VERSION_MINOR DEC(__FCC_minor__)
# define COMPILER_VERSION_PATCH DEC(__FCC_patchlevel__)
# define COMPILER_VERSION_INTERNAL_STR __clang_version__


#elif defined(__FUJITSU)
# define COMPILER_ID "Fujitsu"
# if defined(__FCC_version__)
#   define COMPILER_VERSION __FCC_version__
# elif defined(__FCC_major__)
#   define COMPILER_VERSION_MAJOR DEC(__FCC_major__)
#   define COMPILER_VERSION_MINOR DEC(__FCC_minor__)
#   define COMPILER_VERSION_PATCH DEC(__FCC_patchlevel__)
# endif
# if defined(__fcc_version)
#   define COMPILER_VERSION_INTERNAL DEC(__fcc_version)
# elif defined(__FCC_VERSION)
#   define COMPILER_VERSION_INTERNAL DEC(__FCC_VERSION)
# endif


#elif defined(__ghs__)
# define COMPILER_ID "GHS"
/* __GHS_VERSION_NUMBER = VVVVRP */
# ifdef __GHS_VERSION_NUMBER
# define COMPILER_VERSION_MAJOR DEC(__GHS_VERSION_NUMBER / 100)
# define COMPILER_VERSION_MINOR DEC(__GHS_VERSION_NUMBER / 10 % 10)
# define COMPILER_VERSION_PATCH DEC(__GHS_VERSION_NUMBER      % 10)
# endif

#elif defined(__SCO_VERSION__)
# define COMPILER_ID "SCO"

#elif defined(__ARMCC_VERSION) && !defined(__clang__)
# define COMPILER_ID "ARMCC"
#if __ARMCC_VERSION >= 1000000
  /* __ARMCC_VERSION = VRRPPPP */
  # define COMPILER_VERSION_MAJOR DEC(__ARMCC_VERSION/1000000)
  # define COMPILER_VERSION_MINOR DEC(__ARMCC_VERSION/10000 % 100)
  # define COMPILER_VERSION_PATCH DEC(__ARMCC_VERSION     % 10000)
#else
  /* __ARMCC_VERSION = VRPPPP */
  # define COMPILER_VERSION_MAJOR DEC(__ARMCC_VERSION/100000)
  # define COMPILER_VERSION_MINOR DEC(__ARMCC_VERSION/10000 % 10)
  # define COMPILER_VERSION_PATCH DEC(__ARMCC_VERSION    % 10000)
#endif


#elif defined(__clang__) && defined(__apple_build_version__)
# define COMPILER_ID "AppleClang"
# if defined(_MSC_VER)
#  define SIMULATE_ID "MSVC"
# endif
# define COMPILER_VERSION_MAJOR DEC(__clang_major__)
# define COMPILER_VERSION_MINOR DEC(__clang_minor__)
# define COMPILER_VERSION_PATCH DEC(__clang_patchlevel__)
# if defined(_MSC_VER)
   /* _MSC_VER = VVRR */
#  define SIMULATE_VERSION_MAJOR DEC(_MSC_VER / 100)
#  define SIMULATE_VERSION_MINOR DEC(_MSC_VER % 100)
# endif
# define COMPILER_VERSION_TWEAK DEC(__apple_build_version__)

#elif defined(__clang__) && defined(__ARMCOMPILER_VERSION)
# define COMPILER_ID "ARMClang"
  # define COMPILER_VERSION_MAJOR DEC(__ARMCOMPILER_VERSION/1000000)
  # define COMPILER_VERSION_MINOR DEC(__ARMCOMPILER_VERSION/10000 % 100)
  # define COMPILER_VERSION_PATCH DEC(__ARMCOMPILER_VERSION     % 10000)
# define COMPILER_VERSION_INTERNAL DEC(__ARMCOMPILER_VERSION)

#elif defined(__clang__)
# define COMPILER_ID "Clang"
# if defined(_MSC_VER)
#  define SIMULATE_ID "MSVC"
# endif
# define COMPILER_VERSION_MAJOR DEC(__clang_major__)
# define COMPILER_VERSION_MINOR DEC(__clang_minor__)
# define COMPILER_VERSION_PATCH DEC(__clang_patchlevel__)
# if defined(_MSC_VER)
   /* _MSC_VER = VVRR */
#  define SIMULATE_VERSION_MAJOR DEC(_MSC_VER / 100)
#  define SIMULATE_VERSION_MINOR DEC(_MSC_VER % 100)
# endif

#elif defined(__GNUC__) || defined(__GNUG__)
# define COMPILER_ID "GNU"
# if defined(__GNUC__)
#  define COMPILER_VERSION_MAJOR DEC(__GNUC__)
# else
#  define COMPILER_VERSION_MAJOR DEC(__GNUG__)
# endif
# if defined(__GNUC_MINOR__)
#  define COMPILER_VERSION_MINOR DEC(__GNUC_MINOR__)
# endif
# if defined(__GNUC_PATCHLEVEL__)
#  define COMPILER_VERSION_PATCH DEC(__GNUC_PATCHLEVEL__)
# endif

#elif defined(_MSC_VER)
# define COMPILER_ID "MSVC"
  /* _MSC_VER = VVRR */
# define COMPILER_VERSION_MAJOR DEC(_MSC_VER / 100)
# define COMPILER_VERSION_MINOR DEC(_MSC_VER % 100)
# if defined(_MSC_FULL_VER)
#  if _MSC_VER >= 1400
    /* _MSC_FULL_VER = VVRRPPPPP */
#   define COMPILER_VERSION_PATCH DEC(_MSC_FULL_VER % 100000)
#  else
    /* _MSC_FULL_VER = VVRRPPPP */
#   define COMPILER_VERSION_PATCH DEC(_MSC_FULL_VER % 10000)
#  endif
# endif
# if defined(_MSC_BUILD)
#  define COMPILER_VERSION_TWEAK DEC(_MSC_BUILD)
# endif

#elif defined(__VISUALDSPVERSION__) || defined(__ADSPBLACKFIN__) || defined(__ADSPTS__) || defined(__ADSP21000__)
# define COMPILER_ID "ADSP"
#if defined(__VISUALDSPVERSION__)
  /* __VISUALDSPVERSION__ = 0xVVRRPP00 */
# define COMPILER_VERSION_MAJOR HEX(__VISUALDSPVERSION__>>24)
# define COMPILER_VERSION_MINOR HEX(__VISUALDSPVERSION__>>16 & 0xFF)
# define COMPILER_VERSION_PATCH HEX(__VISUALDSPVERSION__>>8  & 0xFF)
#endif

#elif defined(__IAR_SYSTEMS_ICC__) || defined(__IAR_SYSTEMS_ICC)
# define COMPILER_ID "IAR"
# if defined(__VER__) && defined(__ICCARM__)
#  define COMPILER_VERSION_MAJOR DEC((__VER__) / 1000000)
#  define COMPILER_VERSION_MINOR DEC(((__VER__) / 1000) % 1000)
#  define COMPILER_VERSION_PATCH DEC((__VER__) % 1000)
#  define COMPILER_VERSION_INTERNAL DEC(__IAR_SYSTEMS_ICC__)
# elif defined(__VER__) && (defined(__ICCAVR__) || defined(__ICCRX__) || defined(__ICCRH850__) || defined(__ICCRL78__) || defined(__ICC430__) || defined(__ICCRISCV__) || defined(__ICCV850__) || defined(__ICC8051__) || defined(__ICCSTM8__))
#  define COMPILER_VERSION_MAJOR DEC((__VER__) / 100)
#  define COMPILER_VERSION_MINOR DEC((__VER__) - (((__VER__) / 100)*100))
#  define COMPILER_VERSION_PATCH DEC(__SUBVERSION__)
#  define COMPILER_VERSION_INTERNAL DEC(__IAR_SYSTEMS_ICC__)
# endif


/* These compilers are either not known or too old to define an
  identification macro.  Try to identify the platform and guess that
  it is the native compiler.  */
#elif defined(__hpux) || defined(__hpua)
# define COMPILER_ID "HP"

#else /* unknown compiler */
# define COMPILER_ID ""
#endif

/* Construct the string literal in pieces to prevent the source from
   getting matched.  Store it in a pointer rather than an array
   because some compilers will just produce instructions to fill the
   array rather than assigning a pointer to a static array.  */
char const* info_compiler = "INFO" ":" "compiler[" COMPILER_ID "]";
#ifdef SIMULATE_ID
char const* info_simulate = "INFO" ":" "simulate[" SIMULATE_ID "]";
#endif

#ifdef __QNXNTO__
char const* qnxnto = "INFO" ":" "qnxnto[]";
#endif

#if defined(__CRAYXT_COMPUTE_LINUX_TARGET)
char const *info_cray = "INFO" ":" "compiler_wrapper[CrayPrgEnv]";
#endif

#define STRINGIFY_HELPER(X) #X
#define STRINGIFY(X) STRINGIFY_HELPER(X)

/* Identify known platforms by name.  */
#if defined(__linux) || defined(__linux__) || defined(linux)
# define PLATFORM_ID "Linux"

#elif defined(__MSYS__)
# define PLATFORM_ID "MSYS"

#elif defined(__CYGWIN__)
# define PLATFORM_ID "Cygwin"

#elif defined(__MINGW32__)
# define PLATFORM_ID "MinGW"

#elif defined(__APPLE__)
# define PLATFORM_ID "Darwin"

#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
# define PLATFORM_ID "Windows"

#elif defined(__FreeBSD__) || defined(__FreeBSD)
# define PLATFORM_ID "FreeBSD"

#elif defined(__NetBSD__) || defined(__NetBSD)
# define PLATFORM_ID "NetBSD"

#elif defined(__OpenBSD__) || defined(__OPENBSD)
# define PLATFORM_ID "OpenBSD"

#elif defined(__sun) || defined(sun)
# define PLATFORM_ID "SunOS"

#elif defined(_AIX) || defined(__AIX) || defined(__AIX__) || defined(__aix) || defined(__aix__)
# define PLATFORM_ID "AIX"

#elif defined(__hpux) || defined(__hpux__)
# define PLATFORM_ID "HP-UX"

#elif defined(__HAIKU__)
# define PLATFORM_ID "Haiku"

#elif defined(__BeOS) || defined(__BEOS__) || defined(_BEOS)
# define PLATFORM_ID "BeOS"

#elif defined(__QNX__) || defined(__QNXNTO__)
# define PLATFORM_ID "QNX"

#elif defined(__tru64) || defined(_tru64) || defined(__TRU64__)
# define PLATFORM_ID "Tru64"

#elif defined(__riscos) || defined(__riscos__)
# define PLATFORM_ID "RISCos"

#elif defined(__sinix) || defined(__sinix__) || defined(__SINIX__)
# define PLATFORM_ID "SINIX"

#elif defined(__UNIX_SV__)
# define PLATFORM_ID "UNIX_SV"

#elif defined(__bsdos__)
# define PLATFORM_ID "BSDOS"

#elif defined(_MPRAS) || defined(MPRAS)
# define PLATFORM_ID "MP-RAS"

#elif defined(__osf) || defined(__osf__)
# define PLATFORM_ID "OSF1"

#elif defined(_SCO_SV) || defined(SCO_SV) || defined(sco_sv)
# define PLATFORM_ID "SCO_SV"

#elif defined(__ultrix) || defined(__ultrix__) || defined(_ULTRIX)
# define PLATFORM_ID "ULTRIX"

#elif defined(__XENIX__) || defined(_XENIX) || defined(XENIX)
# define PLATFORM_ID "Xenix"

#elif defined(__WATCOMC__)
# if defined(__LINUX__)
#  define PLATFORM_ID "Linux"

# elif defined(__DOS__)
#  define PLATFORM_ID "DOS"

# elif defined(__OS2__)
#  define PLATFORM_ID "OS2"

# elif defined(__WINDOWS__)
#  define PLATFORM_ID "Windows3x"

# elif defined(__VXWORKS__)
#  define PLATFORM_ID "VxWorks"

# else /* unknown platform */
#  define PLATFORM_ID
# endif

#elif defined(__INTEGRITY)
# if defined(INT_178B)
#  define PLATFORM_ID "Integrity178"

# else /* regular Integrity */
#  define PLATFORM_ID "Integrity"
# endif

#else /* unknown platform */
# define PLATFORM_ID

#endif

/* For windows compilers MSVC and Intel we can determine
   the architecture of the compiler being used.  This is because
   the compilers do not have flags that can change the architecture,
   but rather depend on which compiler is being used
*/
#if defined(_WIN32) && defined(_MSC_VER)
# if defined(_M_IA64)
#  define ARCHITECTURE_ID "IA64"

# elif defined(_M_ARM64EC)
#  define ARCHITECTURE_ID "ARM64EC"

# elif defined(_M_X64) || defined(_M_AMD64)
#  define ARCHITECTURE_ID "x64"

# elif defined(_M_IX86)
#  define ARCHITECTURE_ID "X86"

# elif defined(_M_ARM64)
#  define ARCHITECTURE_ID "ARM64"

# elif defined(_M_ARM)
#  if _M_ARM == 4
#   define ARCHITECTURE_ID "ARMV4I"
#  elif _M_ARM == 5
#   define ARCHITECTURE_ID "ARMV5I"
#  else
#   define ARCHITECTURE_ID "ARMV" STRINGIFY(_M_ARM)
#  endif

# elif defined(_M_MIPS)
#  define ARCHITECTURE_ID "MIPS"

# elif defined(_M_SH)
#  define ARCHITECTURE_ID "SHx"

# else /* unknown architecture */
#  define ARCHITECTURE_ID ""
# endif

#elif defined(__WATCOMC__)
# if defined(_M_I86)
#  define ARCHITECTURE_ID "I86"

# elif defined(_M_IX86)
#  define ARCHITECTURE_ID "X86"

# else /* unknown architecture */
#  define ARCHITECTURE_ID ""
# endif

#elif defined(__IAR_SYSTEMS_ICC__) || defined(__IAR_SYSTEMS_ICC)
# if defined(__ICCARM__)
#  define ARCHITECTURE_ID "ARM"

# elif defined(__ICCRX__)
#  define ARCHITECTURE_ID "RX"

# elif defined(__ICCRH850__)
#  define ARCHITECTURE_ID "RH850"

# elif defined(__ICCRL78__)
#  define ARCHITECTURE_ID "RL78"

# elif defined(__ICCRISCV__)
#  define ARCHITECTURE_ID "RISCV"

# elif defined(__ICCAVR__)
#  define ARCHITECTURE_ID "AVR"

# elif defined(__ICC430__)
#  define ARCHITECTURE_ID "MSP430"

# elif defined(__ICCV850__)
#  define ARCHITECTURE_ID "V850"

# elif defined(__ICC8051__)
#  define ARCHITECTURE_ID "8051"

# elif defined(__ICCSTM8__)
#  define ARCHITECTURE_ID "STM8"

# else /* unknown architecture */
#  define ARCHITECTURE_ID ""
# endif

#elif defined(__ghs__)
# if defined(__PPC64__)
#  define ARCHITECTURE_ID "PPC64"

# elif defined(__ppc__)
#  define ARCHITECTURE_ID "PPC"

# elif defined(__ARM__)
#  define ARCHITECTURE_ID "ARM"

# elif defined(__x86_64__)
#  define ARCHITECTURE_ID "x64"

# elif defined(__i386__)
#  define ARCHITECTURE_ID "X86"

# else /* unknown architecture */
#  define ARCHITECTURE_ID ""
# endif

#elif defined(__TI_COMPILER_VERSION__)
# if defined(__TI_ARM__)
#  define ARCHITECTURE_ID "ARM"

# elif defined(__MSP430__)
#  define ARCHITECTURE_ID "MSP430"

# elif defined(__TMS320C28XX__)
#  define ARCHITECTURE_ID "TMS320C28x"

# elif defined(__TMS320C6X__) || defined(_TMS320C6X)
#  define ARCHITECTURE_ID "TMS320C6x"

# else /* unknown architecture */
#  define ARCHITECTURE_ID ""
# endif

#else
#  define ARCHITECTURE_ID
#endif

/* Convert integer to decimal digit literals.  */
#define DEC(n)                   \
  ('0' + (((n) / 10000000)%10)), \
  ('0' + (((n) / 1000000)%10)),  \
  ('0' + (((n) / 100000)%10)),   \
  ('0' + (((n) / 10000)%10)),    \
  ('0' + (((n) / 1000)%10)),     \
  ('0' + (((n) / 100)%10)),      \
  ('0' + (((n) / 10)%10)),       \
  ('0' +  ((n) % 10))

/* Convert integer to hex digit literals.  */
#define HEX(n)             \
  ('0' + ((n)>>28 & 0xF)), \
  ('0' + ((n)>>24 & 0xF)), \
  ('0' + ((n)>>20 & 0xF)), \
  ('0' + ((n)>>16 & 0xF)), \
  ('0' + ((n)>>12 & 0xF)), \
  ('0' + ((n)>>8  & 0xF)), \
  ('0' + ((n)>>4  & 0xF)), \
  ('0' + ((n)     & 0xF))

/* Construct a string literal encoding the version number. */
#ifdef COMPILER_VERSION
char const* info_version = "INFO" ":" "compiler_version[" COMPILER_VERSION "]";

/* Construct a string literal encoding the version number components. */
#elif defined(COMPILER_VERSION_MAJOR)
char const info_version[] = {
  'I', 'N', 'F', 'O', ':',
  'c','o','m','p','i','l','e','r','_','v','e','r','s','i','o','n','[',
  COMPILER_VERSION_MAJOR,
# ifdef COMPILER_VERSION_MINOR
  '.', COMPILER_VERSION_MINOR,
#  ifdef COMPILER_VERSION_PATCH
   '.', COMPILER_VERSION_PATCH,
#   ifdef COMPILER_VERSION_TWEAK
    '.', COMPILER_VERSION_TWEAK,
#   endif
#  endif
# endif
  ']','\0'};
#endif

/* Construct a string literal encoding the internal version number. */
#ifdef COMPILER_VERSION_INTERNAL
char const info_version_internal[] = {
  'I', 'N', 'F', 'O', ':',
  'c','o','m','p','i','l','e','r','_','v','e','r','s','i','o','n','_',
  'i','n','t','e','r','n','a','l','[',
  COMPILER_VERSION_INTERNAL,']','\0'};
#elif defined(COMPILER_VERSION_INTERNAL_STR)
char const* info_version_internal = "INFO" ":" "compiler_version_internal[" COMPILER_VERSION_INTERNAL_STR "]";
#endif

/* Construct a string literal encoding the version number components. */
#ifdef SIMULATE_VERSION_MAJOR
char const info_simulate_version[] = {
  'I', 'N', 'F', 'O', ':',
  's','i','m','u','l','a','t','e','_','v','e','r','s','i','o','n','[',
  SIMULATE_VERSION_MAJOR,
# ifdef SIMULATE_VERSION_MINOR
  '.', SIMULATE_VERSION_MINOR,
#  ifdef SIMULATE_VERSION_PATCH
   '.', SIMULATE_VERSION_PATCH,
#   ifdef SIMULATE_VERSION_TWEAK
    '.', SIMULATE_VERSION_TWEAK,
#   endif
#  endif
# endif
  ']','\0'};
#endif

/* Construct the string literal in pieces to prevent the source from
   getting matched.  Store it in a pointer rather than an array
   because some compilers will just produce instructions to fill the
   array rather than assigning a pointer to a static array.  */
char const* info_platform = "INFO" ":" "platform[" PLATFORM_ID "]";
char const* info_arch = "INFO" ":" "arch[" ARCHITECTURE_ID "]";



#if defined(__INTEL_COMPILER) && defined(_MSVC_LANG) && _MSVC_LANG < 201403L
#  if defined(__INTEL_CXX11_MODE__)
#    if defined(__cpp_aggregate_nsdmi)
#      define CXX_STD 201402L
#    else
#      define CXX_STD 201103L
#    endif
#  else
#    define CXX_STD 199711L
#  endif
#elif defined(_MSC_VER) && defined(_MSVC_LANG)
#  define CXX_STD _MSVC_LANG
#else
#  define CXX_STD __cplusplus
#endif

const char* info_language_standard_default = "INFO" ":" "standard_default["
#if CXX_STD > 202002L
  "23"
#elif CXX_STD > 201703L
  "20"
#elif CXX_STD >= 201703L
  "17"
#elif CXX_STD >= 201402L
  "14"
#elif CXX_STD >= 201103L
  "11"
#else
  "98"
#endif
"]";

const char* info_language_extensions_default = "INFO" ":" "extensions_default["
/* !defined(_MSC_VER) to exclude Clang's MSVC compatibility mode. */
#if (defined(__clang__) || defined(__GNUC__) ||                               \
     defined(__TI_COMPILER_VERSION__)) &&                                     \
  !defined(__STRICT_ANSI__) && !defined(_MSC_VER)
  "ON"
#else
  "OFF"
#endif
"]";

/*--------------------------------------------------------------------------*/

int main(int argc, char* argv[])
{
  int require = 0;
  require += info_compiler[argc];
  require += info_platform[argc];
#ifdef COMPILER_VERSION_MAJOR
  require += info_version[argc];
#endif
#ifdef COMPILER_VERSION_INTERNAL
  require += info_version_internal[argc];
#endif
#ifdef SIMULATE_ID
  require += info_simulate[argc];
#endif
#ifdef SIMULATE_VERSION_MAJOR
  require += info_simulate_version[argc];
#endif
#if defined(__CRAYXT_COMPUTE_LINUX_TARGET)
  require += info_cray[argc];
#endif
  require += info_language_standard_default[argc];
  require += info_language_extensions_default[argc];
  (void)argv;
  return require;
}

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_c/mowerbot_interfaces/msg/motor_status__type_support.cpp ---
// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.h"
#include "mowerbot_interfaces/msg/detail/motor_status__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _MotorStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorStatus_type_support_ids_t;

static const _MotorStatus_type_support_ids_t _MotorStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorStatus_type_support_symbol_names_t _MotorStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mowerbot_interfaces, msg, MotorStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mowerbot_interfaces, msg, MotorStatus)),
  }
};

typedef struct _MotorStatus_type_support_data_t
{
  void * data[2];
} _MotorStatus_type_support_data_t;

static _MotorStatus_type_support_data_t _MotorStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorStatus_message_typesupport_map = {
  2,
  "mowerbot_interfaces",
  &_MotorStatus_message_typesupport_ids.typesupport_identifier[0],
  &_MotorStatus_message_typesupport_symbol_names.symbol_name[0],
  &_MotorStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorStatus_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorStatus_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mowerbot_interfaces, msg, MotorStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_c::MotorStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_c/mowerbot_interfaces/msg/mower_status__type_support.cpp ---
// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.h"
#include "mowerbot_interfaces/msg/detail/mower_status__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _MowerStatus_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MowerStatus_type_support_ids_t;

static const _MowerStatus_type_support_ids_t _MowerStatus_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MowerStatus_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MowerStatus_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MowerStatus_type_support_symbol_names_t _MowerStatus_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mowerbot_interfaces, msg, MowerStatus)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mowerbot_interfaces, msg, MowerStatus)),
  }
};

typedef struct _MowerStatus_type_support_data_t
{
  void * data[2];
} _MowerStatus_type_support_data_t;

static _MowerStatus_type_support_data_t _MowerStatus_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MowerStatus_message_typesupport_map = {
  2,
  "mowerbot_interfaces",
  &_MowerStatus_message_typesupport_ids.typesupport_identifier[0],
  &_MowerStatus_message_typesupport_symbol_names.symbol_name[0],
  &_MowerStatus_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MowerStatus_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MowerStatus_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, mowerbot_interfaces, msg, MowerStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_c::MowerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_cpp/mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_fastrtps_cpp.hpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MowerStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MowerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MowerStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_cpp/mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_fastrtps_cpp.hpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MotorStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MotorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MotorStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_cpp/mowerbot_interfaces/msg/detail/dds_fastrtps/motor_status__type_support.cpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice
#include "mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: left_front_rpm
  cdr << ros_message.left_front_rpm;
  // Member: right_front_rpm
  cdr << ros_message.right_front_rpm;
  // Member: left_behind_rpm
  cdr << ros_message.left_behind_rpm;
  // Member: right_behind_rpm
  cdr << ros_message.right_behind_rpm;
  // Member: left_front_encoder
  cdr << ros_message.left_front_encoder;
  // Member: right_front_encoder
  cdr << ros_message.right_front_encoder;
  // Member: left_behind_encoder
  cdr << ros_message.left_behind_encoder;
  // Member: right_behind_encoder
  cdr << ros_message.right_behind_encoder;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MotorStatus & ros_message)
{
  // Member: left_front_rpm
  cdr >> ros_message.left_front_rpm;

  // Member: right_front_rpm
  cdr >> ros_message.right_front_rpm;

  // Member: left_behind_rpm
  cdr >> ros_message.left_behind_rpm;

  // Member: right_behind_rpm
  cdr >> ros_message.right_behind_rpm;

  // Member: left_front_encoder
  cdr >> ros_message.left_front_encoder;

  // Member: right_front_encoder
  cdr >> ros_message.right_front_encoder;

  // Member: left_behind_encoder
  cdr >> ros_message.left_behind_encoder;

  // Member: right_behind_encoder
  cdr >> ros_message.right_behind_encoder;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: left_front_rpm
  {
    size_t item_size = sizeof(ros_message.left_front_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_front_rpm
  {
    size_t item_size = sizeof(ros_message.right_front_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_behind_rpm
  {
    size_t item_size = sizeof(ros_message.left_behind_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_behind_rpm
  {
    size_t item_size = sizeof(ros_message.right_behind_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_front_encoder
  {
    size_t item_size = sizeof(ros_message.left_front_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_front_encoder
  {
    size_t item_size = sizeof(ros_message.right_front_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_behind_encoder
  {
    size_t item_size = sizeof(ros_message.left_behind_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: right_behind_encoder
  {
    size_t item_size = sizeof(ros_message.right_behind_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MotorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: left_front_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: right_front_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: left_behind_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: right_behind_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: left_front_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: right_front_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: left_behind_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: right_behind_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mowerbot_interfaces::msg::MotorStatus;
    is_plain =
      (
      offsetof(DataType, right_behind_encoder) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MotorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mowerbot_interfaces::msg::MotorStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MotorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mowerbot_interfaces::msg::MotorStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MotorStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mowerbot_interfaces::msg::MotorStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MotorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MotorStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MotorStatus__callbacks = {
  "mowerbot_interfaces::msg",
  "MotorStatus",
  _MotorStatus__cdr_serialize,
  _MotorStatus__cdr_deserialize,
  _MotorStatus__get_serialized_size,
  _MotorStatus__max_serialized_size
};

static rosidl_message_type_support_t _MotorStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MotorStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mowerbot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MotorStatus>()
{
  return &mowerbot_interfaces::msg::typesupport_fastrtps_cpp::_MotorStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MotorStatus)() {
  return &mowerbot_interfaces::msg::typesupport_fastrtps_cpp::_MotorStatus__handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_cpp/mowerbot_interfaces/msg/detail/dds_fastrtps/mower_status__type_support.cpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice
#include "mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: battery_voltage
  cdr << ros_message.battery_voltage;
  // Member: battery_current
  cdr << ros_message.battery_current;
  // Member: battery_percentage
  cdr << ros_message.battery_percentage;
  // Member: bumper_pressed
  cdr << (ros_message.bumper_pressed ? true : false);
  // Member: stop_active
  cdr << (ros_message.stop_active ? true : false);
  // Member: is_overheated
  cdr << (ros_message.is_overheated ? true : false);
  // Member: mode
  cdr << ros_message.mode;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MowerStatus & ros_message)
{
  // Member: battery_voltage
  cdr >> ros_message.battery_voltage;

  // Member: battery_current
  cdr >> ros_message.battery_current;

  // Member: battery_percentage
  cdr >> ros_message.battery_percentage;

  // Member: bumper_pressed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.bumper_pressed = tmp ? true : false;
  }

  // Member: stop_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.stop_active = tmp ? true : false;
  }

  // Member: is_overheated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_overheated = tmp ? true : false;
  }

  // Member: mode
  cdr >> ros_message.mode;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: battery_current
  {
    size_t item_size = sizeof(ros_message.battery_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: battery_percentage
  {
    size_t item_size = sizeof(ros_message.battery_percentage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bumper_pressed
  {
    size_t item_size = sizeof(ros_message.bumper_pressed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: stop_active
  {
    size_t item_size = sizeof(ros_message.stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_overheated
  {
    size_t item_size = sizeof(ros_message.is_overheated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mode
  {
    size_t item_size = sizeof(ros_message.mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MowerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: battery_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: battery_percentage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bumper_pressed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: stop_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: is_overheated
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mowerbot_interfaces::msg::MowerStatus;
    is_plain =
      (
      offsetof(DataType, mode) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _MowerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mowerbot_interfaces::msg::MowerStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MowerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mowerbot_interfaces::msg::MowerStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MowerStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mowerbot_interfaces::msg::MowerStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MowerStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_MowerStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _MowerStatus__callbacks = {
  "mowerbot_interfaces::msg",
  "MowerStatus",
  _MowerStatus__cdr_serialize,
  _MowerStatus__cdr_deserialize,
  _MowerStatus__get_serialized_size,
  _MowerStatus__max_serialized_size
};

static rosidl_message_type_support_t _MowerStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MowerStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mowerbot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MowerStatus>()
{
  return &mowerbot_interfaces::msg::typesupport_fastrtps_cpp::_MowerStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MowerStatus)() {
  return &mowerbot_interfaces::msg::typesupport_fastrtps_cpp::_MowerStatus__handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/mower_status__type_support.hpp ---
// generated from rosidl_generator_cpp/resource/idl__type_support.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_

#include "rosidl_typesupport_interface/macros.h"

#include "mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_cpp,
  mowerbot_interfaces,
  msg,
  MowerStatus
)();
#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/motor_status__traits.hpp ---
// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mowerbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_front_rpm
  {
    out << "left_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_rpm, out);
    out << ", ";
  }

  // member: right_front_rpm
  {
    out << "right_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_rpm, out);
    out << ", ";
  }

  // member: left_behind_rpm
  {
    out << "left_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_rpm, out);
    out << ", ";
  }

  // member: right_behind_rpm
  {
    out << "right_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_rpm, out);
    out << ", ";
  }

  // member: left_front_encoder
  {
    out << "left_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_encoder, out);
    out << ", ";
  }

  // member: right_front_encoder
  {
    out << "right_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_encoder, out);
    out << ", ";
  }

  // member: left_behind_encoder
  {
    out << "left_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_encoder, out);
    out << ", ";
  }

  // member: right_behind_encoder
  {
    out << "right_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_encoder, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_front_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_rpm, out);
    out << "\n";
  }

  // member: right_front_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_rpm, out);
    out << "\n";
  }

  // member: left_behind_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_rpm, out);
    out << "\n";
  }

  // member: right_behind_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_rpm, out);
    out << "\n";
  }

  // member: left_front_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_encoder, out);
    out << "\n";
  }

  // member: right_front_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_encoder, out);
    out << "\n";
  }

  // member: left_behind_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_encoder, out);
    out << "\n";
  }

  // member: right_behind_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_encoder, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mowerbot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mowerbot_interfaces::msg::MotorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mowerbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mowerbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mowerbot_interfaces::msg::MotorStatus & msg)
{
  return mowerbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mowerbot_interfaces::msg::MotorStatus>()
{
  return "mowerbot_interfaces::msg::MotorStatus";
}

template<>
inline const char * name<mowerbot_interfaces::msg::MotorStatus>()
{
  return "mowerbot_interfaces/msg/MotorStatus";
}

template<>
struct has_fixed_size<mowerbot_interfaces::msg::MotorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mowerbot_interfaces::msg::MotorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mowerbot_interfaces::msg::MotorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/mower_status__traits.hpp ---
// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mowerbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MowerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_current
  {
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << ", ";
  }

  // member: battery_percentage
  {
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << ", ";
  }

  // member: bumper_pressed
  {
    out << "bumper_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper_pressed, out);
    out << ", ";
  }

  // member: stop_active
  {
    out << "stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_active, out);
    out << ", ";
  }

  // member: is_overheated
  {
    out << "is_overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_overheated, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << "\n";
  }

  // member: battery_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << "\n";
  }

  // member: bumper_pressed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bumper_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper_pressed, out);
    out << "\n";
  }

  // member: stop_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_active, out);
    out << "\n";
  }

  // member: is_overheated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_overheated, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MowerStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mowerbot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mowerbot_interfaces::msg::MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mowerbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mowerbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mowerbot_interfaces::msg::MowerStatus & msg)
{
  return mowerbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mowerbot_interfaces::msg::MowerStatus>()
{
  return "mowerbot_interfaces::msg::MowerStatus";
}

template<>
inline const char * name<mowerbot_interfaces::msg::MowerStatus>()
{
  return "mowerbot_interfaces/msg/MowerStatus";
}

template<>
struct has_fixed_size<mowerbot_interfaces::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mowerbot_interfaces::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mowerbot_interfaces::msg::MowerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/motor_status__struct.hpp ---
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mowerbot_interfaces__msg__MotorStatus __attribute__((deprecated))
#else
# define DEPRECATED__mowerbot_interfaces__msg__MotorStatus __declspec(deprecated)
#endif

namespace mowerbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorStatus_
{
  using Type = MotorStatus_<ContainerAllocator>;

  explicit MotorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_rpm = 0.0f;
      this->right_front_rpm = 0.0f;
      this->left_behind_rpm = 0.0f;
      this->right_behind_rpm = 0.0f;
      this->left_front_encoder = 0l;
      this->right_front_encoder = 0l;
      this->left_behind_encoder = 0l;
      this->right_behind_encoder = 0l;
    }
  }

  explicit MotorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_rpm = 0.0f;
      this->right_front_rpm = 0.0f;
      this->left_behind_rpm = 0.0f;
      this->right_behind_rpm = 0.0f;
      this->left_front_encoder = 0l;
      this->right_front_encoder = 0l;
      this->left_behind_encoder = 0l;
      this->right_behind_encoder = 0l;
    }
  }

  // field types and members
  using _left_front_rpm_type =
    float;
  _left_front_rpm_type left_front_rpm;
  using _right_front_rpm_type =
    float;
  _right_front_rpm_type right_front_rpm;
  using _left_behind_rpm_type =
    float;
  _left_behind_rpm_type left_behind_rpm;
  using _right_behind_rpm_type =
    float;
  _right_behind_rpm_type right_behind_rpm;
  using _left_front_encoder_type =
    int32_t;
  _left_front_encoder_type left_front_encoder;
  using _right_front_encoder_type =
    int32_t;
  _right_front_encoder_type right_front_encoder;
  using _left_behind_encoder_type =
    int32_t;
  _left_behind_encoder_type left_behind_encoder;
  using _right_behind_encoder_type =
    int32_t;
  _right_behind_encoder_type right_behind_encoder;

  // setters for named parameter idiom
  Type & set__left_front_rpm(
    const float & _arg)
  {
    this->left_front_rpm = _arg;
    return *this;
  }
  Type & set__right_front_rpm(
    const float & _arg)
  {
    this->right_front_rpm = _arg;
    return *this;
  }
  Type & set__left_behind_rpm(
    const float & _arg)
  {
    this->left_behind_rpm = _arg;
    return *this;
  }
  Type & set__right_behind_rpm(
    const float & _arg)
  {
    this->right_behind_rpm = _arg;
    return *this;
  }
  Type & set__left_front_encoder(
    const int32_t & _arg)
  {
    this->left_front_encoder = _arg;
    return *this;
  }
  Type & set__right_front_encoder(
    const int32_t & _arg)
  {
    this->right_front_encoder = _arg;
    return *this;
  }
  Type & set__left_behind_encoder(
    const int32_t & _arg)
  {
    this->left_behind_encoder = _arg;
    return *this;
  }
  Type & set__right_behind_encoder(
    const int32_t & _arg)
  {
    this->right_behind_encoder = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mowerbot_interfaces__msg__MotorStatus
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mowerbot_interfaces__msg__MotorStatus
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorStatus_ & other) const
  {
    if (this->left_front_rpm != other.left_front_rpm) {
      return false;
    }
    if (this->right_front_rpm != other.right_front_rpm) {
      return false;
    }
    if (this->left_behind_rpm != other.left_behind_rpm) {
      return false;
    }
    if (this->right_behind_rpm != other.right_behind_rpm) {
      return false;
    }
    if (this->left_front_encoder != other.left_front_encoder) {
      return false;
    }
    if (this->right_front_encoder != other.right_front_encoder) {
      return false;
    }
    if (this->left_behind_encoder != other.left_behind_encoder) {
      return false;
    }
    if (this->right_behind_encoder != other.right_behind_encoder) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorStatus_

// alias to use template instance with default allocator
using MotorStatus =
  mowerbot_interfaces::msg::MotorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/motor_status__builder.hpp ---
// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mowerbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorStatus_right_behind_encoder
{
public:
  explicit Init_MotorStatus_right_behind_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  ::mowerbot_interfaces::msg::MotorStatus right_behind_encoder(::mowerbot_interfaces::msg::MotorStatus::_right_behind_encoder_type arg)
  {
    msg_.right_behind_encoder = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_behind_encoder
{
public:
  explicit Init_MotorStatus_left_behind_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_behind_encoder left_behind_encoder(::mowerbot_interfaces::msg::MotorStatus::_left_behind_encoder_type arg)
  {
    msg_.left_behind_encoder = std::move(arg);
    return Init_MotorStatus_right_behind_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_front_encoder
{
public:
  explicit Init_MotorStatus_right_front_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_behind_encoder right_front_encoder(::mowerbot_interfaces::msg::MotorStatus::_right_front_encoder_type arg)
  {
    msg_.right_front_encoder = std::move(arg);
    return Init_MotorStatus_left_behind_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_front_encoder
{
public:
  explicit Init_MotorStatus_left_front_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_front_encoder left_front_encoder(::mowerbot_interfaces::msg::MotorStatus::_left_front_encoder_type arg)
  {
    msg_.left_front_encoder = std::move(arg);
    return Init_MotorStatus_right_front_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_behind_rpm
{
public:
  explicit Init_MotorStatus_right_behind_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_front_encoder right_behind_rpm(::mowerbot_interfaces::msg::MotorStatus::_right_behind_rpm_type arg)
  {
    msg_.right_behind_rpm = std::move(arg);
    return Init_MotorStatus_left_front_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_behind_rpm
{
public:
  explicit Init_MotorStatus_left_behind_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_behind_rpm left_behind_rpm(::mowerbot_interfaces::msg::MotorStatus::_left_behind_rpm_type arg)
  {
    msg_.left_behind_rpm = std::move(arg);
    return Init_MotorStatus_right_behind_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_front_rpm
{
public:
  explicit Init_MotorStatus_right_front_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_behind_rpm right_front_rpm(::mowerbot_interfaces::msg::MotorStatus::_right_front_rpm_type arg)
  {
    msg_.right_front_rpm = std::move(arg);
    return Init_MotorStatus_left_behind_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_front_rpm
{
public:
  Init_MotorStatus_left_front_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorStatus_right_front_rpm left_front_rpm(::mowerbot_interfaces::msg::MotorStatus::_left_front_rpm_type arg)
  {
    msg_.left_front_rpm = std::move(arg);
    return Init_MotorStatus_right_front_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mowerbot_interfaces::msg::MotorStatus>()
{
  return mowerbot_interfaces::msg::builder::Init_MotorStatus_left_front_rpm();
}

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/mower_status__builder.hpp ---
// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mowerbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MowerStatus_mode
{
public:
  explicit Init_MowerStatus_mode(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  ::mowerbot_interfaces::msg::MowerStatus mode(::mowerbot_interfaces::msg::MowerStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_is_overheated
{
public:
  explicit Init_MowerStatus_is_overheated(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_mode is_overheated(::mowerbot_interfaces::msg::MowerStatus::_is_overheated_type arg)
  {
    msg_.is_overheated = std::move(arg);
    return Init_MowerStatus_mode(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_stop_active
{
public:
  explicit Init_MowerStatus_stop_active(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_is_overheated stop_active(::mowerbot_interfaces::msg::MowerStatus::_stop_active_type arg)
  {
    msg_.stop_active = std::move(arg);
    return Init_MowerStatus_is_overheated(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_bumper_pressed
{
public:
  explicit Init_MowerStatus_bumper_pressed(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_stop_active bumper_pressed(::mowerbot_interfaces::msg::MowerStatus::_bumper_pressed_type arg)
  {
    msg_.bumper_pressed = std::move(arg);
    return Init_MowerStatus_stop_active(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_percentage
{
public:
  explicit Init_MowerStatus_battery_percentage(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_bumper_pressed battery_percentage(::mowerbot_interfaces::msg::MowerStatus::_battery_percentage_type arg)
  {
    msg_.battery_percentage = std::move(arg);
    return Init_MowerStatus_bumper_pressed(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_current
{
public:
  explicit Init_MowerStatus_battery_current(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_battery_percentage battery_current(::mowerbot_interfaces::msg::MowerStatus::_battery_current_type arg)
  {
    msg_.battery_current = std::move(arg);
    return Init_MowerStatus_battery_percentage(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_voltage
{
public:
  Init_MowerStatus_battery_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerStatus_battery_current battery_voltage(::mowerbot_interfaces::msg::MowerStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_MowerStatus_battery_current(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mowerbot_interfaces::msg::MowerStatus>()
{
  return mowerbot_interfaces::msg::builder::Init_MowerStatus_battery_voltage();
}

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/motor_status__type_support.hpp ---
// generated from rosidl_generator_cpp/resource/idl__type_support.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_

#include "rosidl_typesupport_interface/macros.h"

#include "mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_cpp,
  mowerbot_interfaces,
  msg,
  MotorStatus
)();
#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/detail/mower_status__struct.hpp ---
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mowerbot_interfaces__msg__MowerStatus __attribute__((deprecated))
#else
# define DEPRECATED__mowerbot_interfaces__msg__MowerStatus __declspec(deprecated)
#endif

namespace mowerbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerStatus_
{
  using Type = MowerStatus_<ContainerAllocator>;

  explicit MowerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_percentage = 0.0f;
      this->bumper_pressed = false;
      this->stop_active = false;
      this->is_overheated = false;
      this->mode = 0;
    }
  }

  explicit MowerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_percentage = 0.0f;
      this->bumper_pressed = false;
      this->stop_active = false;
      this->is_overheated = false;
      this->mode = 0;
    }
  }

  // field types and members
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_current_type =
    float;
  _battery_current_type battery_current;
  using _battery_percentage_type =
    float;
  _battery_percentage_type battery_percentage;
  using _bumper_pressed_type =
    bool;
  _bumper_pressed_type bumper_pressed;
  using _stop_active_type =
    bool;
  _stop_active_type stop_active;
  using _is_overheated_type =
    bool;
  _is_overheated_type is_overheated;
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_current(
    const float & _arg)
  {
    this->battery_current = _arg;
    return *this;
  }
  Type & set__battery_percentage(
    const float & _arg)
  {
    this->battery_percentage = _arg;
    return *this;
  }
  Type & set__bumper_pressed(
    const bool & _arg)
  {
    this->bumper_pressed = _arg;
    return *this;
  }
  Type & set__stop_active(
    const bool & _arg)
  {
    this->stop_active = _arg;
    return *this;
  }
  Type & set__is_overheated(
    const bool & _arg)
  {
    this->is_overheated = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mowerbot_interfaces__msg__MowerStatus
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mowerbot_interfaces__msg__MowerStatus
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerStatus_ & other) const
  {
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_current != other.battery_current) {
      return false;
    }
    if (this->battery_percentage != other.battery_percentage) {
      return false;
    }
    if (this->bumper_pressed != other.bumper_pressed) {
      return false;
    }
    if (this->stop_active != other.stop_active) {
      return false;
    }
    if (this->is_overheated != other.is_overheated) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerStatus_

// alias to use template instance with default allocator
using MowerStatus =
  mowerbot_interfaces::msg::MowerStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/mower_status.hpp ---
// generated from rosidl_generator_cpp/resource/idl.hpp.em
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_
#define MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__builder.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__traits.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__type_support.hpp"

#endif  // MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/motor_status.hpp ---
// generated from rosidl_generator_cpp/resource/idl.hpp.em
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_
#define MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__builder.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__traits.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__type_support.hpp"

#endif  // MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_generator_cpp/mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp ---
// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_mowerbot_interfaces
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_c/mowerbot_interfaces/msg/detail/motor_status__type_support_c.cpp ---
// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice
#include "mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.h"
#include "mowerbot_interfaces/msg/detail/motor_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MotorStatus__ros_msg_type = mowerbot_interfaces__msg__MotorStatus;

static bool _MotorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MotorStatus__ros_msg_type * ros_message = static_cast<const _MotorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: left_front_rpm
  {
    cdr << ros_message->left_front_rpm;
  }

  // Field name: right_front_rpm
  {
    cdr << ros_message->right_front_rpm;
  }

  // Field name: left_behind_rpm
  {
    cdr << ros_message->left_behind_rpm;
  }

  // Field name: right_behind_rpm
  {
    cdr << ros_message->right_behind_rpm;
  }

  // Field name: left_front_encoder
  {
    cdr << ros_message->left_front_encoder;
  }

  // Field name: right_front_encoder
  {
    cdr << ros_message->right_front_encoder;
  }

  // Field name: left_behind_encoder
  {
    cdr << ros_message->left_behind_encoder;
  }

  // Field name: right_behind_encoder
  {
    cdr << ros_message->right_behind_encoder;
  }

  return true;
}

static bool _MotorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MotorStatus__ros_msg_type * ros_message = static_cast<_MotorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: left_front_rpm
  {
    cdr >> ros_message->left_front_rpm;
  }

  // Field name: right_front_rpm
  {
    cdr >> ros_message->right_front_rpm;
  }

  // Field name: left_behind_rpm
  {
    cdr >> ros_message->left_behind_rpm;
  }

  // Field name: right_behind_rpm
  {
    cdr >> ros_message->right_behind_rpm;
  }

  // Field name: left_front_encoder
  {
    cdr >> ros_message->left_front_encoder;
  }

  // Field name: right_front_encoder
  {
    cdr >> ros_message->right_front_encoder;
  }

  // Field name: left_behind_encoder
  {
    cdr >> ros_message->left_behind_encoder;
  }

  // Field name: right_behind_encoder
  {
    cdr >> ros_message->right_behind_encoder;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mowerbot_interfaces
size_t get_serialized_size_mowerbot_interfaces__msg__MotorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MotorStatus__ros_msg_type * ros_message = static_cast<const _MotorStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name left_front_rpm
  {
    size_t item_size = sizeof(ros_message->left_front_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_front_rpm
  {
    size_t item_size = sizeof(ros_message->right_front_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_behind_rpm
  {
    size_t item_size = sizeof(ros_message->left_behind_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_behind_rpm
  {
    size_t item_size = sizeof(ros_message->right_behind_rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_front_encoder
  {
    size_t item_size = sizeof(ros_message->left_front_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_front_encoder
  {
    size_t item_size = sizeof(ros_message->right_front_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_behind_encoder
  {
    size_t item_size = sizeof(ros_message->left_behind_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_behind_encoder
  {
    size_t item_size = sizeof(ros_message->right_behind_encoder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MotorStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mowerbot_interfaces__msg__MotorStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mowerbot_interfaces
size_t max_serialized_size_mowerbot_interfaces__msg__MotorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: left_front_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: right_front_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: left_behind_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: right_behind_rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: left_front_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: right_front_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: left_behind_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: right_behind_encoder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mowerbot_interfaces__msg__MotorStatus;
    is_plain =
      (
      offsetof(DataType, right_behind_encoder) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MotorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mowerbot_interfaces__msg__MotorStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MotorStatus = {
  "mowerbot_interfaces::msg",
  "MotorStatus",
  _MotorStatus__cdr_serialize,
  _MotorStatus__cdr_deserialize,
  _MotorStatus__get_serialized_size,
  _MotorStatus__max_serialized_size
};

static rosidl_message_type_support_t _MotorStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MotorStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mowerbot_interfaces, msg, MotorStatus)() {
  return &_MotorStatus__type_support;
}

#if defined(__cplusplus)
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/rosidl_typesupport_fastrtps_c/mowerbot_interfaces/msg/detail/mower_status__type_support_c.cpp ---
// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice
#include "mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.h"
#include "mowerbot_interfaces/msg/detail/mower_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _MowerStatus__ros_msg_type = mowerbot_interfaces__msg__MowerStatus;

static bool _MowerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _MowerStatus__ros_msg_type * ros_message = static_cast<const _MowerStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: battery_current
  {
    cdr << ros_message->battery_current;
  }

  // Field name: battery_percentage
  {
    cdr << ros_message->battery_percentage;
  }

  // Field name: bumper_pressed
  {
    cdr << (ros_message->bumper_pressed ? true : false);
  }

  // Field name: stop_active
  {
    cdr << (ros_message->stop_active ? true : false);
  }

  // Field name: is_overheated
  {
    cdr << (ros_message->is_overheated ? true : false);
  }

  // Field name: mode
  {
    cdr << ros_message->mode;
  }

  return true;
}

static bool _MowerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _MowerStatus__ros_msg_type * ros_message = static_cast<_MowerStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: battery_voltage
  {
    cdr >> ros_message->battery_voltage;
  }

  // Field name: battery_current
  {
    cdr >> ros_message->battery_current;
  }

  // Field name: battery_percentage
  {
    cdr >> ros_message->battery_percentage;
  }

  // Field name: bumper_pressed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->bumper_pressed = tmp ? true : false;
  }

  // Field name: stop_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stop_active = tmp ? true : false;
  }

  // Field name: is_overheated
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_overheated = tmp ? true : false;
  }

  // Field name: mode
  {
    cdr >> ros_message->mode;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mowerbot_interfaces
size_t get_serialized_size_mowerbot_interfaces__msg__MowerStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _MowerStatus__ros_msg_type * ros_message = static_cast<const _MowerStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery_current
  {
    size_t item_size = sizeof(ros_message->battery_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery_percentage
  {
    size_t item_size = sizeof(ros_message->battery_percentage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bumper_pressed
  {
    size_t item_size = sizeof(ros_message->bumper_pressed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stop_active
  {
    size_t item_size = sizeof(ros_message->stop_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name is_overheated
  {
    size_t item_size = sizeof(ros_message->is_overheated);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mode
  {
    size_t item_size = sizeof(ros_message->mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _MowerStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mowerbot_interfaces__msg__MowerStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mowerbot_interfaces
size_t max_serialized_size_mowerbot_interfaces__msg__MowerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: battery_current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: battery_percentage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bumper_pressed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stop_active
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: is_overheated
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mowerbot_interfaces__msg__MowerStatus;
    is_plain =
      (
      offsetof(DataType, mode) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _MowerStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mowerbot_interfaces__msg__MowerStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_MowerStatus = {
  "mowerbot_interfaces::msg",
  "MowerStatus",
  _MowerStatus__cdr_serialize,
  _MowerStatus__cdr_deserialize,
  _MowerStatus__get_serialized_size,
  _MowerStatus__max_serialized_size
};

static rosidl_message_type_support_t _MowerStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_MowerStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mowerbot_interfaces, msg, MowerStatus)() {
  return &_MowerStatus__type_support;
}

#if defined(__cplusplus)
}
#endif

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/mowerbot_interfaces__py/CMakeLists.txt ---
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Unlike other generators, this custom command depends on the target
# ${rosidl_generate_interfaces_TARGET} and not the IDL files.
# The IDL files could be generated files,as they are for .action files.
# CMake does not allow `add_custom_command()` to depend on files generated in
# a different CMake subdirectory, and this command is invoked after an
# add_subdirectory() call.
add_custom_command(
  OUTPUT ${_generated_extension_files} ${_generated_py_files} ${_generated_c_files}
  COMMAND ${PYTHON_EXECUTABLE} ${rosidl_generator_py_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  --typesupport-impls "${_typesupport_impls}"
  DEPENDS ${target_dependencies} ${rosidl_generate_interfaces_TARGET}
  COMMENT "Generating Python code for ROS interfaces"
  VERBATIM
)

if(TARGET ${rosidl_generate_interfaces_TARGET}${_target_suffix})
  message(WARNING "Custom target ${rosidl_generate_interfaces_TARGET}${_target_suffix} already exists")
else()
  add_custom_target(
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    DEPENDS
    ${_generated_extension_files}
    ${_generated_py_files}
    ${_generated_c_files}
  )
endif()

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/ament_cmake_python/mowerbot_interfaces/setup.py ---
from setuptools import find_packages
from setuptools import setup

setup(
    name='mowerbot_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('mowerbot_interfaces', 'mowerbot_interfaces.*')),
)

--- FILE: ./mowerbot_interfaces/build/mowerbot_interfaces/ament_cmake_python/mowerbot_interfaces/mowerbot_interfaces/__init__.py ---

--- FILE: ./mowerbot_interfaces/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
  </package>
--- FILE: ./mowerbot_interfaces/install/_local_setup_util_sh.py ---
# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import argparse
from collections import OrderedDict
import os
from pathlib import Path
import sys


FORMAT_STR_COMMENT_LINE = '# {comment}'
FORMAT_STR_SET_ENV_VAR = 'export {name}="{value}"'
FORMAT_STR_USE_ENV_VAR = '${name}'
FORMAT_STR_INVOKE_SCRIPT = 'COLCON_CURRENT_PREFIX="{prefix}" _colcon_prefix_sh_source_script "{script_path}"'  # noqa: E501
FORMAT_STR_REMOVE_LEADING_SEPARATOR = 'if [ "$(echo -n ${name} | head -c 1)" = ":" ]; then export {name}=${{{name}#?}} ; fi'  # noqa: E501
FORMAT_STR_REMOVE_TRAILING_SEPARATOR = 'if [ "$(echo -n ${name} | tail -c 1)" = ":" ]; then export {name}=${{{name}%?}} ; fi'  # noqa: E501

DSV_TYPE_APPEND_NON_DUPLICATE = 'append-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE = 'prepend-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS = 'prepend-non-duplicate-if-exists'
DSV_TYPE_SET = 'set'
DSV_TYPE_SET_IF_UNSET = 'set-if-unset'
DSV_TYPE_SOURCE = 'source'


def main(argv=sys.argv[1:]):  # noqa: D103
    parser = argparse.ArgumentParser(
        description='Output shell commands for the packages in topological '
                    'order')
    parser.add_argument(
        'primary_extension',
        help='The file extension of the primary shell')
    parser.add_argument(
        'additional_extension', nargs='?',
        help='The additional file extension to be considered')
    parser.add_argument(
        '--merged-install', action='store_true',
        help='All install prefixes are merged into a single location')
    args = parser.parse_args(argv)

    packages = get_packages(Path(__file__).parent, args.merged_install)

    ordered_packages = order_packages(packages)
    for pkg_name in ordered_packages:
        if _include_comments():
            print(
                FORMAT_STR_COMMENT_LINE.format_map(
                    {'comment': 'Package: ' + pkg_name}))
        prefix = os.path.abspath(os.path.dirname(__file__))
        if not args.merged_install:
            prefix = os.path.join(prefix, pkg_name)
        for line in get_commands(
            pkg_name, prefix, args.primary_extension,
            args.additional_extension
        ):
            print(line)

    for line in _remove_ending_separators():
        print(line)


def get_packages(prefix_path, merged_install):
    """
    Find packages based on colcon-specific files created during installation.

    :param Path prefix_path: The install prefix path of all packages
    :param bool merged_install: The flag if the packages are all installed
      directly in the prefix or if each package is installed in a subdirectory
      named after the package
    :returns: A mapping from the package name to the set of runtime
      dependencies
    :rtype: dict
    """
    packages = {}
    # since importing colcon_core isn't feasible here the following constant
    # must match colcon_core.location.get_relative_package_index_path()
    subdirectory = 'share/colcon-core/packages'
    if merged_install:
        # return if workspace is empty
        if not (prefix_path / subdirectory).is_dir():
            return packages
        # find all files in the subdirectory
        for p in (prefix_path / subdirectory).iterdir():
            if not p.is_file():
                continue
            if p.name.startswith('.'):
                continue
            add_package_runtime_dependencies(p, packages)
    else:
        # for each subdirectory look for the package specific file
        for p in prefix_path.iterdir():
            if not p.is_dir():
                continue
            if p.name.startswith('.'):
                continue
            p = p / subdirectory / p.name
            if p.is_file():
                add_package_runtime_dependencies(p, packages)

    # remove unknown dependencies
    pkg_names = set(packages.keys())
    for k in packages.keys():
        packages[k] = {d for d in packages[k] if d in pkg_names}

    return packages


def add_package_runtime_dependencies(path, packages):
    """
    Check the path and if it exists extract the packages runtime dependencies.

    :param Path path: The resource file containing the runtime dependencies
    :param dict packages: A mapping from package names to the sets of runtime
      dependencies to add to
    """
    content = path.read_text()
    dependencies = set(content.split(os.pathsep) if content else [])
    packages[path.name] = dependencies


def order_packages(packages):
    """
    Order packages topologically.

    :param dict packages: A mapping from package name to the set of runtime
      dependencies
    :returns: The package names
    :rtype: list
    """
    # select packages with no dependencies in alphabetical order
    to_be_ordered = list(packages.keys())
    ordered = []
    while to_be_ordered:
        pkg_names_without_deps = [
            name for name in to_be_ordered if not packages[name]]
        if not pkg_names_without_deps:
            reduce_cycle_set(packages)
            raise RuntimeError(
                'Circular dependency between: ' + ', '.join(sorted(packages)))
        pkg_names_without_deps.sort()
        pkg_name = pkg_names_without_deps[0]
        to_be_ordered.remove(pkg_name)
        ordered.append(pkg_name)
        # remove item from dependency lists
        for k in list(packages.keys()):
            if pkg_name in packages[k]:
                packages[k].remove(pkg_name)
    return ordered


def reduce_cycle_set(packages):
    """
    Reduce the set of packages to the ones part of the circular dependency.

    :param dict packages: A mapping from package name to the set of runtime
      dependencies which is modified in place
    """
    last_depended = None
    while len(packages) > 0:
        # get all remaining dependencies
        depended = set()
        for pkg_name, dependencies in packages.items():
            depended = depended.union(dependencies)
        # remove all packages which are not dependent on
        for name in list(packages.keys()):
            if name not in depended:
                del packages[name]
        if last_depended:
            # if remaining packages haven't changed return them
            if last_depended == depended:
                return packages.keys()
        # otherwise reduce again
        last_depended = depended


def _include_comments():
    # skipping comment lines when COLCON_TRACE is not set speeds up the
    # processing especially on Windows
    return bool(os.environ.get('COLCON_TRACE'))


def get_commands(pkg_name, prefix, primary_extension, additional_extension):
    commands = []
    package_dsv_path = os.path.join(prefix, 'share', pkg_name, 'package.dsv')
    if os.path.exists(package_dsv_path):
        commands += process_dsv_file(
            package_dsv_path, prefix, primary_extension, additional_extension)
    return commands


def process_dsv_file(
    dsv_path, prefix, primary_extension=None, additional_extension=None
):
    commands = []
    if _include_comments():
        commands.append(FORMAT_STR_COMMENT_LINE.format_map({'comment': dsv_path}))
    with open(dsv_path, 'r') as h:
        content = h.read()
    lines = content.splitlines()

    basenames = OrderedDict()
    for i, line in enumerate(lines):
        # skip over empty or whitespace-only lines
        if not line.strip():
            continue
        # skip over comments
        if line.startswith('#'):
            continue
        try:
            type_, remainder = line.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "Line %d in '%s' doesn't contain a semicolon separating the "
                'type from the arguments' % (i + 1, dsv_path))
        if type_ != DSV_TYPE_SOURCE:
            # handle non-source lines
            try:
                commands += handle_dsv_types_except_source(
                    type_, remainder, prefix)
            except RuntimeError as e:
                raise RuntimeError(
                    "Line %d in '%s' %s" % (i + 1, dsv_path, e)) from e
        else:
            # group remaining source lines by basename
            path_without_ext, ext = os.path.splitext(remainder)
            if path_without_ext not in basenames:
                basenames[path_without_ext] = set()
            assert ext.startswith('.')
            ext = ext[1:]
            if ext in (primary_extension, additional_extension):
                basenames[path_without_ext].add(ext)

    # add the dsv extension to each basename if the file exists
    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if os.path.exists(basename + '.dsv'):
            extensions.add('dsv')

    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if 'dsv' in extensions:
            # process dsv files recursively
            commands += process_dsv_file(
                basename + '.dsv', prefix, primary_extension=primary_extension,
                additional_extension=additional_extension)
        elif primary_extension in extensions and len(extensions) == 1:
            # source primary-only files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + primary_extension})]
        elif additional_extension in extensions:
            # source non-primary files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + additional_extension})]

    return commands


def handle_dsv_types_except_source(type_, remainder, prefix):
    commands = []
    if type_ in (DSV_TYPE_SET, DSV_TYPE_SET_IF_UNSET):
        try:
            env_name, value = remainder.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the value')
        try_prefixed_value = os.path.join(prefix, value) if value else prefix
        if os.path.exists(try_prefixed_value):
            value = try_prefixed_value
        if type_ == DSV_TYPE_SET:
            commands += _set(env_name, value)
        elif type_ == DSV_TYPE_SET_IF_UNSET:
            commands += _set_if_unset(env_name, value)
        else:
            assert False
    elif type_ in (
        DSV_TYPE_APPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS
    ):
        try:
            env_name_and_values = remainder.split(';')
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the values')
        env_name = env_name_and_values[0]
        values = env_name_and_values[1:]
        for value in values:
            if not value:
                value = prefix
            elif not os.path.isabs(value):
                value = os.path.join(prefix, value)
            if (
                type_ == DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS and
                not os.path.exists(value)
            ):
                comment = f'skip extending {env_name} with not existing ' \
                    f'path: {value}'
                if _include_comments():
                    commands.append(
                        FORMAT_STR_COMMENT_LINE.format_map({'comment': comment}))
            elif type_ == DSV_TYPE_APPEND_NON_DUPLICATE:
                commands += _append_unique_value(env_name, value)
            else:
                commands += _prepend_unique_value(env_name, value)
    else:
        raise RuntimeError(
            'contains an unknown environment hook type: ' + type_)
    return commands


env_state = {}


def _append_unique_value(name, value):
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # append even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional leading separator
    extend = FORMAT_STR_USE_ENV_VAR.format_map({'name': name}) + os.pathsep
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': extend + value})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


def _prepend_unique_value(name, value):
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # prepend even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional trailing separator
    extend = os.pathsep + FORMAT_STR_USE_ENV_VAR.format_map({'name': name})
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value + extend})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


# generate commands for removing prepended underscores
def _remove_ending_separators():
    # do nothing if the shell extension does not implement the logic
    if FORMAT_STR_REMOVE_TRAILING_SEPARATOR is None:
        return []

    global env_state
    commands = []
    for name in env_state:
        # skip variables that already had values before this script started prepending
        if name in os.environ:
            continue
        commands += [
            FORMAT_STR_REMOVE_LEADING_SEPARATOR.format_map({'name': name}),
            FORMAT_STR_REMOVE_TRAILING_SEPARATOR.format_map({'name': name})]
    return commands


def _set(name, value):
    global env_state
    env_state[name] = value
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    return [line]


def _set_if_unset(name, value):
    global env_state
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    if env_state.get(name, os.environ.get(name)):
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


if __name__ == '__main__':  # pragma: no cover
    try:
        rc = main()
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        rc = 1
    sys.exit(rc)

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/local/lib/python3.10/dist-packages/mowerbot_interfaces/__init__.py ---

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/local/lib/python3.10/dist-packages/mowerbot_interfaces/msg/__init__.py ---
from mowerbot_interfaces.msg._motor_status import MotorStatus  # noqa: F401
from mowerbot_interfaces.msg._mower_status import MowerStatus  # noqa: F401

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/local/lib/python3.10/dist-packages/mowerbot_interfaces/msg/_mower_status.py ---
# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mowerbot_interfaces:msg/MowerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MowerStatus(type):
    """Metaclass of message 'MowerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mowerbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mowerbot_interfaces.msg.MowerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__mower_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__mower_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__mower_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__mower_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__mower_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MowerStatus(metaclass=Metaclass_MowerStatus):
    """Message class 'MowerStatus'."""

    __slots__ = [
        '_battery_voltage',
        '_battery_current',
        '_battery_percentage',
        '_bumper_pressed',
        '_stop_active',
        '_is_overheated',
        '_mode',
    ]

    _fields_and_field_types = {
        'battery_voltage': 'float',
        'battery_current': 'float',
        'battery_percentage': 'float',
        'bumper_pressed': 'boolean',
        'stop_active': 'boolean',
        'is_overheated': 'boolean',
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.battery_current = kwargs.get('battery_current', float())
        self.battery_percentage = kwargs.get('battery_percentage', float())
        self.bumper_pressed = kwargs.get('bumper_pressed', bool())
        self.stop_active = kwargs.get('stop_active', bool())
        self.is_overheated = kwargs.get('is_overheated', bool())
        self.mode = kwargs.get('mode', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.battery_current != other.battery_current:
            return False
        if self.battery_percentage != other.battery_percentage:
            return False
        if self.bumper_pressed != other.bumper_pressed:
            return False
        if self.stop_active != other.stop_active:
            return False
        if self.is_overheated != other.is_overheated:
            return False
        if self.mode != other.mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def battery_current(self):
        """Message field 'battery_current'."""
        return self._battery_current

    @battery_current.setter
    def battery_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_current = value

    @builtins.property
    def battery_percentage(self):
        """Message field 'battery_percentage'."""
        return self._battery_percentage

    @battery_percentage.setter
    def battery_percentage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_percentage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_percentage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_percentage = value

    @builtins.property
    def bumper_pressed(self):
        """Message field 'bumper_pressed'."""
        return self._bumper_pressed

    @bumper_pressed.setter
    def bumper_pressed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'bumper_pressed' field must be of type 'bool'"
        self._bumper_pressed = value

    @builtins.property
    def stop_active(self):
        """Message field 'stop_active'."""
        return self._stop_active

    @stop_active.setter
    def stop_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop_active' field must be of type 'bool'"
        self._stop_active = value

    @builtins.property
    def is_overheated(self):
        """Message field 'is_overheated'."""
        return self._is_overheated

    @is_overheated.setter
    def is_overheated(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_overheated' field must be of type 'bool'"
        self._is_overheated = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/local/lib/python3.10/dist-packages/mowerbot_interfaces/msg/_motor_status.py ---
# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mowerbot_interfaces:msg/MotorStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotorStatus(type):
    """Metaclass of message 'MotorStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mowerbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mowerbot_interfaces.msg.MotorStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motor_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motor_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motor_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motor_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motor_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotorStatus(metaclass=Metaclass_MotorStatus):
    """Message class 'MotorStatus'."""

    __slots__ = [
        '_left_front_rpm',
        '_right_front_rpm',
        '_left_behind_rpm',
        '_right_behind_rpm',
        '_left_front_encoder',
        '_right_front_encoder',
        '_left_behind_encoder',
        '_right_behind_encoder',
    ]

    _fields_and_field_types = {
        'left_front_rpm': 'float',
        'right_front_rpm': 'float',
        'left_behind_rpm': 'float',
        'right_behind_rpm': 'float',
        'left_front_encoder': 'int32',
        'right_front_encoder': 'int32',
        'left_behind_encoder': 'int32',
        'right_behind_encoder': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.left_front_rpm = kwargs.get('left_front_rpm', float())
        self.right_front_rpm = kwargs.get('right_front_rpm', float())
        self.left_behind_rpm = kwargs.get('left_behind_rpm', float())
        self.right_behind_rpm = kwargs.get('right_behind_rpm', float())
        self.left_front_encoder = kwargs.get('left_front_encoder', int())
        self.right_front_encoder = kwargs.get('right_front_encoder', int())
        self.left_behind_encoder = kwargs.get('left_behind_encoder', int())
        self.right_behind_encoder = kwargs.get('right_behind_encoder', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.left_front_rpm != other.left_front_rpm:
            return False
        if self.right_front_rpm != other.right_front_rpm:
            return False
        if self.left_behind_rpm != other.left_behind_rpm:
            return False
        if self.right_behind_rpm != other.right_behind_rpm:
            return False
        if self.left_front_encoder != other.left_front_encoder:
            return False
        if self.right_front_encoder != other.right_front_encoder:
            return False
        if self.left_behind_encoder != other.left_behind_encoder:
            return False
        if self.right_behind_encoder != other.right_behind_encoder:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def left_front_rpm(self):
        """Message field 'left_front_rpm'."""
        return self._left_front_rpm

    @left_front_rpm.setter
    def left_front_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_front_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'left_front_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._left_front_rpm = value

    @builtins.property
    def right_front_rpm(self):
        """Message field 'right_front_rpm'."""
        return self._right_front_rpm

    @right_front_rpm.setter
    def right_front_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_front_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'right_front_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._right_front_rpm = value

    @builtins.property
    def left_behind_rpm(self):
        """Message field 'left_behind_rpm'."""
        return self._left_behind_rpm

    @left_behind_rpm.setter
    def left_behind_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'left_behind_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'left_behind_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._left_behind_rpm = value

    @builtins.property
    def right_behind_rpm(self):
        """Message field 'right_behind_rpm'."""
        return self._right_behind_rpm

    @right_behind_rpm.setter
    def right_behind_rpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'right_behind_rpm' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'right_behind_rpm' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._right_behind_rpm = value

    @builtins.property
    def left_front_encoder(self):
        """Message field 'left_front_encoder'."""
        return self._left_front_encoder

    @left_front_encoder.setter
    def left_front_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_front_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_front_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_front_encoder = value

    @builtins.property
    def right_front_encoder(self):
        """Message field 'right_front_encoder'."""
        return self._right_front_encoder

    @right_front_encoder.setter
    def right_front_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_front_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_front_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_front_encoder = value

    @builtins.property
    def left_behind_encoder(self):
        """Message field 'left_behind_encoder'."""
        return self._left_behind_encoder

    @left_behind_encoder.setter
    def left_behind_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'left_behind_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'left_behind_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._left_behind_encoder = value

    @builtins.property
    def right_behind_encoder(self):
        """Message field 'right_behind_encoder'."""
        return self._right_behind_encoder

    @right_behind_encoder.setter
    def right_behind_encoder(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'right_behind_encoder' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'right_behind_encoder' field must be an integer in [-2147483648, 2147483647]"
        self._right_behind_encoder = value

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_introspection_cpp.hpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__rosidl_typesupport_introspection_cpp.h.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_


#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO(dirk-thomas) these visibility macros should be message package specific
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MotorStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__type_support.hpp ---
// generated from rosidl_generator_cpp/resource/idl__type_support.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_

#include "rosidl_typesupport_interface/macros.h"

#include "mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_cpp,
  mowerbot_interfaces,
  msg,
  MowerStatus
)();
#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TYPE_SUPPORT_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__traits.hpp ---
// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mowerbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_front_rpm
  {
    out << "left_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_rpm, out);
    out << ", ";
  }

  // member: right_front_rpm
  {
    out << "right_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_rpm, out);
    out << ", ";
  }

  // member: left_behind_rpm
  {
    out << "left_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_rpm, out);
    out << ", ";
  }

  // member: right_behind_rpm
  {
    out << "right_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_rpm, out);
    out << ", ";
  }

  // member: left_front_encoder
  {
    out << "left_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_encoder, out);
    out << ", ";
  }

  // member: right_front_encoder
  {
    out << "right_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_encoder, out);
    out << ", ";
  }

  // member: left_behind_encoder
  {
    out << "left_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_encoder, out);
    out << ", ";
  }

  // member: right_behind_encoder
  {
    out << "right_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_encoder, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_front_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_rpm, out);
    out << "\n";
  }

  // member: right_front_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_rpm, out);
    out << "\n";
  }

  // member: left_behind_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_rpm, out);
    out << "\n";
  }

  // member: right_behind_rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_behind_rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_rpm, out);
    out << "\n";
  }

  // member: left_front_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_front_encoder, out);
    out << "\n";
  }

  // member: right_front_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_front_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_front_encoder, out);
    out << "\n";
  }

  // member: left_behind_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.left_behind_encoder, out);
    out << "\n";
  }

  // member: right_behind_encoder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_behind_encoder: ";
    rosidl_generator_traits::value_to_yaml(msg.right_behind_encoder, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mowerbot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mowerbot_interfaces::msg::MotorStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mowerbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mowerbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mowerbot_interfaces::msg::MotorStatus & msg)
{
  return mowerbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mowerbot_interfaces::msg::MotorStatus>()
{
  return "mowerbot_interfaces::msg::MotorStatus";
}

template<>
inline const char * name<mowerbot_interfaces::msg::MotorStatus>()
{
  return "mowerbot_interfaces/msg/MotorStatus";
}

template<>
struct has_fixed_size<mowerbot_interfaces::msg::MotorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mowerbot_interfaces::msg::MotorStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mowerbot_interfaces::msg::MotorStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TRAITS_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_fastrtps_cpp.hpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MowerStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MowerStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MowerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MowerStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__type_support.cpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mowerbot_interfaces::msg::MotorStatus(_init);
}

void MotorStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mowerbot_interfaces::msg::MotorStatus *>(message_memory);
  typed_message->~MotorStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorStatus_message_member_array[8] = {
  {
    "left_front_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_front_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_front_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_front_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_behind_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_behind_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_behind_rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_behind_rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_front_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_front_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_front_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_front_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_behind_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, left_behind_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_behind_encoder",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MotorStatus, right_behind_encoder),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorStatus_message_members = {
  "mowerbot_interfaces::msg",  // message namespace
  "MotorStatus",  // message name
  8,  // number of fields
  sizeof(mowerbot_interfaces::msg::MotorStatus),
  MotorStatus_message_member_array,  // message members
  MotorStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MotorStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MotorStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__rosidl_typesupport_introspection_cpp.hpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__rosidl_typesupport_introspection_cpp.h.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_


#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

// TODO(dirk-thomas) these visibility macros should be message package specific
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MowerStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__traits.hpp ---
// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mowerbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MowerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: battery_current
  {
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << ", ";
  }

  // member: battery_percentage
  {
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << ", ";
  }

  // member: bumper_pressed
  {
    out << "bumper_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper_pressed, out);
    out << ", ";
  }

  // member: stop_active
  {
    out << "stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_active, out);
    out << ", ";
  }

  // member: is_overheated
  {
    out << "is_overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_overheated, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: battery_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_current: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_current, out);
    out << "\n";
  }

  // member: battery_percentage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_percentage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_percentage, out);
    out << "\n";
  }

  // member: bumper_pressed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bumper_pressed: ";
    rosidl_generator_traits::value_to_yaml(msg.bumper_pressed, out);
    out << "\n";
  }

  // member: stop_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop_active: ";
    rosidl_generator_traits::value_to_yaml(msg.stop_active, out);
    out << "\n";
  }

  // member: is_overheated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_overheated: ";
    rosidl_generator_traits::value_to_yaml(msg.is_overheated, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MowerStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mowerbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use mowerbot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mowerbot_interfaces::msg::MowerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mowerbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mowerbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const mowerbot_interfaces::msg::MowerStatus & msg)
{
  return mowerbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mowerbot_interfaces::msg::MowerStatus>()
{
  return "mowerbot_interfaces::msg::MowerStatus";
}

template<>
inline const char * name<mowerbot_interfaces::msg::MowerStatus>()
{
  return "mowerbot_interfaces/msg/MowerStatus";
}

template<>
struct has_fixed_size<mowerbot_interfaces::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mowerbot_interfaces::msg::MowerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mowerbot_interfaces::msg::MowerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__TRAITS_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__struct.hpp ---
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mowerbot_interfaces__msg__MotorStatus __attribute__((deprecated))
#else
# define DEPRECATED__mowerbot_interfaces__msg__MotorStatus __declspec(deprecated)
#endif

namespace mowerbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorStatus_
{
  using Type = MotorStatus_<ContainerAllocator>;

  explicit MotorStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_rpm = 0.0f;
      this->right_front_rpm = 0.0f;
      this->left_behind_rpm = 0.0f;
      this->right_behind_rpm = 0.0f;
      this->left_front_encoder = 0l;
      this->right_front_encoder = 0l;
      this->left_behind_encoder = 0l;
      this->right_behind_encoder = 0l;
    }
  }

  explicit MotorStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left_front_rpm = 0.0f;
      this->right_front_rpm = 0.0f;
      this->left_behind_rpm = 0.0f;
      this->right_behind_rpm = 0.0f;
      this->left_front_encoder = 0l;
      this->right_front_encoder = 0l;
      this->left_behind_encoder = 0l;
      this->right_behind_encoder = 0l;
    }
  }

  // field types and members
  using _left_front_rpm_type =
    float;
  _left_front_rpm_type left_front_rpm;
  using _right_front_rpm_type =
    float;
  _right_front_rpm_type right_front_rpm;
  using _left_behind_rpm_type =
    float;
  _left_behind_rpm_type left_behind_rpm;
  using _right_behind_rpm_type =
    float;
  _right_behind_rpm_type right_behind_rpm;
  using _left_front_encoder_type =
    int32_t;
  _left_front_encoder_type left_front_encoder;
  using _right_front_encoder_type =
    int32_t;
  _right_front_encoder_type right_front_encoder;
  using _left_behind_encoder_type =
    int32_t;
  _left_behind_encoder_type left_behind_encoder;
  using _right_behind_encoder_type =
    int32_t;
  _right_behind_encoder_type right_behind_encoder;

  // setters for named parameter idiom
  Type & set__left_front_rpm(
    const float & _arg)
  {
    this->left_front_rpm = _arg;
    return *this;
  }
  Type & set__right_front_rpm(
    const float & _arg)
  {
    this->right_front_rpm = _arg;
    return *this;
  }
  Type & set__left_behind_rpm(
    const float & _arg)
  {
    this->left_behind_rpm = _arg;
    return *this;
  }
  Type & set__right_behind_rpm(
    const float & _arg)
  {
    this->right_behind_rpm = _arg;
    return *this;
  }
  Type & set__left_front_encoder(
    const int32_t & _arg)
  {
    this->left_front_encoder = _arg;
    return *this;
  }
  Type & set__right_front_encoder(
    const int32_t & _arg)
  {
    this->right_front_encoder = _arg;
    return *this;
  }
  Type & set__left_behind_encoder(
    const int32_t & _arg)
  {
    this->left_behind_encoder = _arg;
    return *this;
  }
  Type & set__right_behind_encoder(
    const int32_t & _arg)
  {
    this->right_behind_encoder = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mowerbot_interfaces__msg__MotorStatus
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mowerbot_interfaces__msg__MotorStatus
    std::shared_ptr<mowerbot_interfaces::msg::MotorStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorStatus_ & other) const
  {
    if (this->left_front_rpm != other.left_front_rpm) {
      return false;
    }
    if (this->right_front_rpm != other.right_front_rpm) {
      return false;
    }
    if (this->left_behind_rpm != other.left_behind_rpm) {
      return false;
    }
    if (this->right_behind_rpm != other.right_behind_rpm) {
      return false;
    }
    if (this->left_front_encoder != other.left_front_encoder) {
      return false;
    }
    if (this->right_front_encoder != other.right_front_encoder) {
      return false;
    }
    if (this->left_behind_encoder != other.left_behind_encoder) {
      return false;
    }
    if (this->right_behind_encoder != other.right_behind_encoder) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorStatus_

// alias to use template instance with default allocator
using MotorStatus =
  mowerbot_interfaces::msg::MotorStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__STRUCT_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__builder.hpp ---
// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mowerbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorStatus_right_behind_encoder
{
public:
  explicit Init_MotorStatus_right_behind_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  ::mowerbot_interfaces::msg::MotorStatus right_behind_encoder(::mowerbot_interfaces::msg::MotorStatus::_right_behind_encoder_type arg)
  {
    msg_.right_behind_encoder = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_behind_encoder
{
public:
  explicit Init_MotorStatus_left_behind_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_behind_encoder left_behind_encoder(::mowerbot_interfaces::msg::MotorStatus::_left_behind_encoder_type arg)
  {
    msg_.left_behind_encoder = std::move(arg);
    return Init_MotorStatus_right_behind_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_front_encoder
{
public:
  explicit Init_MotorStatus_right_front_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_behind_encoder right_front_encoder(::mowerbot_interfaces::msg::MotorStatus::_right_front_encoder_type arg)
  {
    msg_.right_front_encoder = std::move(arg);
    return Init_MotorStatus_left_behind_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_front_encoder
{
public:
  explicit Init_MotorStatus_left_front_encoder(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_front_encoder left_front_encoder(::mowerbot_interfaces::msg::MotorStatus::_left_front_encoder_type arg)
  {
    msg_.left_front_encoder = std::move(arg);
    return Init_MotorStatus_right_front_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_behind_rpm
{
public:
  explicit Init_MotorStatus_right_behind_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_front_encoder right_behind_rpm(::mowerbot_interfaces::msg::MotorStatus::_right_behind_rpm_type arg)
  {
    msg_.right_behind_rpm = std::move(arg);
    return Init_MotorStatus_left_front_encoder(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_behind_rpm
{
public:
  explicit Init_MotorStatus_left_behind_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_right_behind_rpm left_behind_rpm(::mowerbot_interfaces::msg::MotorStatus::_left_behind_rpm_type arg)
  {
    msg_.left_behind_rpm = std::move(arg);
    return Init_MotorStatus_right_behind_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_right_front_rpm
{
public:
  explicit Init_MotorStatus_right_front_rpm(::mowerbot_interfaces::msg::MotorStatus & msg)
  : msg_(msg)
  {}
  Init_MotorStatus_left_behind_rpm right_front_rpm(::mowerbot_interfaces::msg::MotorStatus::_right_front_rpm_type arg)
  {
    msg_.right_front_rpm = std::move(arg);
    return Init_MotorStatus_left_behind_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

class Init_MotorStatus_left_front_rpm
{
public:
  Init_MotorStatus_left_front_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorStatus_right_front_rpm left_front_rpm(::mowerbot_interfaces::msg::MotorStatus::_left_front_rpm_type arg)
  {
    msg_.left_front_rpm = std::move(arg);
    return Init_MotorStatus_right_front_rpm(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MotorStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mowerbot_interfaces::msg::MotorStatus>()
{
  return mowerbot_interfaces::msg::builder::Init_MotorStatus_left_front_rpm();
}

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__BUILDER_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__builder.hpp ---
// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mowerbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MowerStatus_mode
{
public:
  explicit Init_MowerStatus_mode(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  ::mowerbot_interfaces::msg::MowerStatus mode(::mowerbot_interfaces::msg::MowerStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_is_overheated
{
public:
  explicit Init_MowerStatus_is_overheated(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_mode is_overheated(::mowerbot_interfaces::msg::MowerStatus::_is_overheated_type arg)
  {
    msg_.is_overheated = std::move(arg);
    return Init_MowerStatus_mode(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_stop_active
{
public:
  explicit Init_MowerStatus_stop_active(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_is_overheated stop_active(::mowerbot_interfaces::msg::MowerStatus::_stop_active_type arg)
  {
    msg_.stop_active = std::move(arg);
    return Init_MowerStatus_is_overheated(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_bumper_pressed
{
public:
  explicit Init_MowerStatus_bumper_pressed(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_stop_active bumper_pressed(::mowerbot_interfaces::msg::MowerStatus::_bumper_pressed_type arg)
  {
    msg_.bumper_pressed = std::move(arg);
    return Init_MowerStatus_stop_active(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_percentage
{
public:
  explicit Init_MowerStatus_battery_percentage(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_bumper_pressed battery_percentage(::mowerbot_interfaces::msg::MowerStatus::_battery_percentage_type arg)
  {
    msg_.battery_percentage = std::move(arg);
    return Init_MowerStatus_bumper_pressed(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_current
{
public:
  explicit Init_MowerStatus_battery_current(::mowerbot_interfaces::msg::MowerStatus & msg)
  : msg_(msg)
  {}
  Init_MowerStatus_battery_percentage battery_current(::mowerbot_interfaces::msg::MowerStatus::_battery_current_type arg)
  {
    msg_.battery_current = std::move(arg);
    return Init_MowerStatus_battery_percentage(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

class Init_MowerStatus_battery_voltage
{
public:
  Init_MowerStatus_battery_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MowerStatus_battery_current battery_voltage(::mowerbot_interfaces::msg::MowerStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_MowerStatus_battery_current(msg_);
  }

private:
  ::mowerbot_interfaces::msg::MowerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mowerbot_interfaces::msg::MowerStatus>()
{
  return mowerbot_interfaces::msg::builder::Init_MowerStatus_battery_voltage();
}

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__BUILDER_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__type_support.hpp ---
// generated from rosidl_generator_cpp/resource/idl__type_support.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_

#include "rosidl_typesupport_interface/macros.h"

#include "mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_cpp,
  mowerbot_interfaces,
  msg,
  MotorStatus
)();
#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__TYPE_SUPPORT_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/motor_status__rosidl_typesupport_fastrtps_cpp.hpp ---
// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mowerbot_interfaces:msg/MotorStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_serialize(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mowerbot_interfaces::msg::MotorStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
get_serialized_size(
  const mowerbot_interfaces::msg::MotorStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
max_serialized_size_MotorStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mowerbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mowerbot_interfaces, msg, MotorStatus)();

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOTOR_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__struct.hpp ---
// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_
#define MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mowerbot_interfaces__msg__MowerStatus __attribute__((deprecated))
#else
# define DEPRECATED__mowerbot_interfaces__msg__MowerStatus __declspec(deprecated)
#endif

namespace mowerbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MowerStatus_
{
  using Type = MowerStatus_<ContainerAllocator>;

  explicit MowerStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_percentage = 0.0f;
      this->bumper_pressed = false;
      this->stop_active = false;
      this->is_overheated = false;
      this->mode = 0;
    }
  }

  explicit MowerStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_voltage = 0.0f;
      this->battery_current = 0.0f;
      this->battery_percentage = 0.0f;
      this->bumper_pressed = false;
      this->stop_active = false;
      this->is_overheated = false;
      this->mode = 0;
    }
  }

  // field types and members
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _battery_current_type =
    float;
  _battery_current_type battery_current;
  using _battery_percentage_type =
    float;
  _battery_percentage_type battery_percentage;
  using _bumper_pressed_type =
    bool;
  _bumper_pressed_type bumper_pressed;
  using _stop_active_type =
    bool;
  _stop_active_type stop_active;
  using _is_overheated_type =
    bool;
  _is_overheated_type is_overheated;
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__battery_current(
    const float & _arg)
  {
    this->battery_current = _arg;
    return *this;
  }
  Type & set__battery_percentage(
    const float & _arg)
  {
    this->battery_percentage = _arg;
    return *this;
  }
  Type & set__bumper_pressed(
    const bool & _arg)
  {
    this->bumper_pressed = _arg;
    return *this;
  }
  Type & set__stop_active(
    const bool & _arg)
  {
    this->stop_active = _arg;
    return *this;
  }
  Type & set__is_overheated(
    const bool & _arg)
  {
    this->is_overheated = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mowerbot_interfaces__msg__MowerStatus
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mowerbot_interfaces__msg__MowerStatus
    std::shared_ptr<mowerbot_interfaces::msg::MowerStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MowerStatus_ & other) const
  {
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->battery_current != other.battery_current) {
      return false;
    }
    if (this->battery_percentage != other.battery_percentage) {
      return false;
    }
    if (this->bumper_pressed != other.bumper_pressed) {
      return false;
    }
    if (this->stop_active != other.stop_active) {
      return false;
    }
    if (this->is_overheated != other.is_overheated) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const MowerStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MowerStatus_

// alias to use template instance with default allocator
using MowerStatus =
  mowerbot_interfaces::msg::MowerStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mowerbot_interfaces

#endif  // MOWERBOT_INTERFACES__MSG__DETAIL__MOWER_STATUS__STRUCT_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/detail/mower_status__type_support.cpp ---
// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mowerbot_interfaces:msg/MowerStatus.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mowerbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MowerStatus_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mowerbot_interfaces::msg::MowerStatus(_init);
}

void MowerStatus_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mowerbot_interfaces::msg::MowerStatus *>(message_memory);
  typed_message->~MowerStatus();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MowerStatus_message_member_array[7] = {
  {
    "battery_voltage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_voltage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_current",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_current),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "battery_percentage",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, battery_percentage),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bumper_pressed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, bumper_pressed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stop_active",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, stop_active),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_overheated",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, is_overheated),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mowerbot_interfaces::msg::MowerStatus, mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MowerStatus_message_members = {
  "mowerbot_interfaces::msg",  // message namespace
  "MowerStatus",  // message name
  7,  // number of fields
  sizeof(mowerbot_interfaces::msg::MowerStatus),
  MowerStatus_message_member_array,  // message members
  MowerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  MowerStatus_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MowerStatus_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MowerStatus_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mowerbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mowerbot_interfaces::msg::MowerStatus>()
{
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MowerStatus_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mowerbot_interfaces, msg, MowerStatus)() {
  return &::mowerbot_interfaces::msg::rosidl_typesupport_introspection_cpp::MowerStatus_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/mower_status.hpp ---
// generated from rosidl_generator_cpp/resource/idl.hpp.em
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_
#define MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_

#include "mowerbot_interfaces/msg/detail/mower_status__struct.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__builder.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__traits.hpp"
#include "mowerbot_interfaces/msg/detail/mower_status__type_support.hpp"

#endif  // MOWERBOT_INTERFACES__MSG__MOWER_STATUS_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/motor_status.hpp ---
// generated from rosidl_generator_cpp/resource/idl.hpp.em
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_
#define MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_

#include "mowerbot_interfaces/msg/detail/motor_status__struct.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__builder.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__traits.hpp"
#include "mowerbot_interfaces/msg/detail/motor_status__type_support.hpp"

#endif  // MOWERBOT_INTERFACES__MSG__MOTOR_STATUS_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/include/mowerbot_interfaces/mowerbot_interfaces/msg/rosidl_generator_cpp__visibility_control.hpp ---
// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_mowerbot_interfaces
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_mowerbot_interfaces __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_mowerbot_interfaces
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_mowerbot_interfaces
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // MOWERBOT_INTERFACES__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/share/mowerbot_interfaces/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/share/mowerbot_interfaces/msg/MowerStatus.msg ---
#電源
float32 battery_voltage #電池電壓
float32 battery_current #即時電流
float32 battery_percentage #電池電量

#安全系統
bool bumper_pressed #實體保險桿是否觸發
bool stop_active #緊急停止按鈕是否觸發
bool is_overheated #馬達或驅動是否過熱

#作業狀態
uint8 mode #0:移動建圖 1:導航割草 2:搖桿控制割草





--- FILE: ./mowerbot_interfaces/install/mowerbot_interfaces/share/mowerbot_interfaces/msg/MotorStatus.msg ---
float32 left_front_rpm #左前轉速
float32 right_front_rpm #右前轉速
float32 left_behind_rpm #左後轉速
float32 right_behind_rpm #右後轉速

int32 left_front_encoder #左前編碼器累計數值
int32 right_front_encoder #右前編碼器累計數值
int32 left_behind_encoder #左後編碼器累計數值
int32 right_behind_encoder #右後編碼器累計數值

--- FILE: ./mowerbot_interfaces/install/_local_setup_util_ps1.py ---
# Copyright 2016-2019 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import argparse
from collections import OrderedDict
import os
from pathlib import Path
import sys


FORMAT_STR_COMMENT_LINE = '# {comment}'
FORMAT_STR_SET_ENV_VAR = 'Set-Item -Path "Env:{name}" -Value "{value}"'
FORMAT_STR_USE_ENV_VAR = '$env:{name}'
FORMAT_STR_INVOKE_SCRIPT = '_colcon_prefix_powershell_source_script "{script_path}"'  # noqa: E501
FORMAT_STR_REMOVE_LEADING_SEPARATOR = ''  # noqa: E501
FORMAT_STR_REMOVE_TRAILING_SEPARATOR = ''  # noqa: E501

DSV_TYPE_APPEND_NON_DUPLICATE = 'append-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE = 'prepend-non-duplicate'
DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS = 'prepend-non-duplicate-if-exists'
DSV_TYPE_SET = 'set'
DSV_TYPE_SET_IF_UNSET = 'set-if-unset'
DSV_TYPE_SOURCE = 'source'


def main(argv=sys.argv[1:]):  # noqa: D103
    parser = argparse.ArgumentParser(
        description='Output shell commands for the packages in topological '
                    'order')
    parser.add_argument(
        'primary_extension',
        help='The file extension of the primary shell')
    parser.add_argument(
        'additional_extension', nargs='?',
        help='The additional file extension to be considered')
    parser.add_argument(
        '--merged-install', action='store_true',
        help='All install prefixes are merged into a single location')
    args = parser.parse_args(argv)

    packages = get_packages(Path(__file__).parent, args.merged_install)

    ordered_packages = order_packages(packages)
    for pkg_name in ordered_packages:
        if _include_comments():
            print(
                FORMAT_STR_COMMENT_LINE.format_map(
                    {'comment': 'Package: ' + pkg_name}))
        prefix = os.path.abspath(os.path.dirname(__file__))
        if not args.merged_install:
            prefix = os.path.join(prefix, pkg_name)
        for line in get_commands(
            pkg_name, prefix, args.primary_extension,
            args.additional_extension
        ):
            print(line)

    for line in _remove_ending_separators():
        print(line)


def get_packages(prefix_path, merged_install):
    """
    Find packages based on colcon-specific files created during installation.

    :param Path prefix_path: The install prefix path of all packages
    :param bool merged_install: The flag if the packages are all installed
      directly in the prefix or if each package is installed in a subdirectory
      named after the package
    :returns: A mapping from the package name to the set of runtime
      dependencies
    :rtype: dict
    """
    packages = {}
    # since importing colcon_core isn't feasible here the following constant
    # must match colcon_core.location.get_relative_package_index_path()
    subdirectory = 'share/colcon-core/packages'
    if merged_install:
        # return if workspace is empty
        if not (prefix_path / subdirectory).is_dir():
            return packages
        # find all files in the subdirectory
        for p in (prefix_path / subdirectory).iterdir():
            if not p.is_file():
                continue
            if p.name.startswith('.'):
                continue
            add_package_runtime_dependencies(p, packages)
    else:
        # for each subdirectory look for the package specific file
        for p in prefix_path.iterdir():
            if not p.is_dir():
                continue
            if p.name.startswith('.'):
                continue
            p = p / subdirectory / p.name
            if p.is_file():
                add_package_runtime_dependencies(p, packages)

    # remove unknown dependencies
    pkg_names = set(packages.keys())
    for k in packages.keys():
        packages[k] = {d for d in packages[k] if d in pkg_names}

    return packages


def add_package_runtime_dependencies(path, packages):
    """
    Check the path and if it exists extract the packages runtime dependencies.

    :param Path path: The resource file containing the runtime dependencies
    :param dict packages: A mapping from package names to the sets of runtime
      dependencies to add to
    """
    content = path.read_text()
    dependencies = set(content.split(os.pathsep) if content else [])
    packages[path.name] = dependencies


def order_packages(packages):
    """
    Order packages topologically.

    :param dict packages: A mapping from package name to the set of runtime
      dependencies
    :returns: The package names
    :rtype: list
    """
    # select packages with no dependencies in alphabetical order
    to_be_ordered = list(packages.keys())
    ordered = []
    while to_be_ordered:
        pkg_names_without_deps = [
            name for name in to_be_ordered if not packages[name]]
        if not pkg_names_without_deps:
            reduce_cycle_set(packages)
            raise RuntimeError(
                'Circular dependency between: ' + ', '.join(sorted(packages)))
        pkg_names_without_deps.sort()
        pkg_name = pkg_names_without_deps[0]
        to_be_ordered.remove(pkg_name)
        ordered.append(pkg_name)
        # remove item from dependency lists
        for k in list(packages.keys()):
            if pkg_name in packages[k]:
                packages[k].remove(pkg_name)
    return ordered


def reduce_cycle_set(packages):
    """
    Reduce the set of packages to the ones part of the circular dependency.

    :param dict packages: A mapping from package name to the set of runtime
      dependencies which is modified in place
    """
    last_depended = None
    while len(packages) > 0:
        # get all remaining dependencies
        depended = set()
        for pkg_name, dependencies in packages.items():
            depended = depended.union(dependencies)
        # remove all packages which are not dependent on
        for name in list(packages.keys()):
            if name not in depended:
                del packages[name]
        if last_depended:
            # if remaining packages haven't changed return them
            if last_depended == depended:
                return packages.keys()
        # otherwise reduce again
        last_depended = depended


def _include_comments():
    # skipping comment lines when COLCON_TRACE is not set speeds up the
    # processing especially on Windows
    return bool(os.environ.get('COLCON_TRACE'))


def get_commands(pkg_name, prefix, primary_extension, additional_extension):
    commands = []
    package_dsv_path = os.path.join(prefix, 'share', pkg_name, 'package.dsv')
    if os.path.exists(package_dsv_path):
        commands += process_dsv_file(
            package_dsv_path, prefix, primary_extension, additional_extension)
    return commands


def process_dsv_file(
    dsv_path, prefix, primary_extension=None, additional_extension=None
):
    commands = []
    if _include_comments():
        commands.append(FORMAT_STR_COMMENT_LINE.format_map({'comment': dsv_path}))
    with open(dsv_path, 'r') as h:
        content = h.read()
    lines = content.splitlines()

    basenames = OrderedDict()
    for i, line in enumerate(lines):
        # skip over empty or whitespace-only lines
        if not line.strip():
            continue
        # skip over comments
        if line.startswith('#'):
            continue
        try:
            type_, remainder = line.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "Line %d in '%s' doesn't contain a semicolon separating the "
                'type from the arguments' % (i + 1, dsv_path))
        if type_ != DSV_TYPE_SOURCE:
            # handle non-source lines
            try:
                commands += handle_dsv_types_except_source(
                    type_, remainder, prefix)
            except RuntimeError as e:
                raise RuntimeError(
                    "Line %d in '%s' %s" % (i + 1, dsv_path, e)) from e
        else:
            # group remaining source lines by basename
            path_without_ext, ext = os.path.splitext(remainder)
            if path_without_ext not in basenames:
                basenames[path_without_ext] = set()
            assert ext.startswith('.')
            ext = ext[1:]
            if ext in (primary_extension, additional_extension):
                basenames[path_without_ext].add(ext)

    # add the dsv extension to each basename if the file exists
    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if os.path.exists(basename + '.dsv'):
            extensions.add('dsv')

    for basename, extensions in basenames.items():
        if not os.path.isabs(basename):
            basename = os.path.join(prefix, basename)
        if 'dsv' in extensions:
            # process dsv files recursively
            commands += process_dsv_file(
                basename + '.dsv', prefix, primary_extension=primary_extension,
                additional_extension=additional_extension)
        elif primary_extension in extensions and len(extensions) == 1:
            # source primary-only files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + primary_extension})]
        elif additional_extension in extensions:
            # source non-primary files
            commands += [
                FORMAT_STR_INVOKE_SCRIPT.format_map({
                    'prefix': prefix,
                    'script_path': basename + '.' + additional_extension})]

    return commands


def handle_dsv_types_except_source(type_, remainder, prefix):
    commands = []
    if type_ in (DSV_TYPE_SET, DSV_TYPE_SET_IF_UNSET):
        try:
            env_name, value = remainder.split(';', 1)
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the value')
        try_prefixed_value = os.path.join(prefix, value) if value else prefix
        if os.path.exists(try_prefixed_value):
            value = try_prefixed_value
        if type_ == DSV_TYPE_SET:
            commands += _set(env_name, value)
        elif type_ == DSV_TYPE_SET_IF_UNSET:
            commands += _set_if_unset(env_name, value)
        else:
            assert False
    elif type_ in (
        DSV_TYPE_APPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE,
        DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS
    ):
        try:
            env_name_and_values = remainder.split(';')
        except ValueError:
            raise RuntimeError(
                "doesn't contain a semicolon separating the environment name "
                'from the values')
        env_name = env_name_and_values[0]
        values = env_name_and_values[1:]
        for value in values:
            if not value:
                value = prefix
            elif not os.path.isabs(value):
                value = os.path.join(prefix, value)
            if (
                type_ == DSV_TYPE_PREPEND_NON_DUPLICATE_IF_EXISTS and
                not os.path.exists(value)
            ):
                comment = f'skip extending {env_name} with not existing ' \
                    f'path: {value}'
                if _include_comments():
                    commands.append(
                        FORMAT_STR_COMMENT_LINE.format_map({'comment': comment}))
            elif type_ == DSV_TYPE_APPEND_NON_DUPLICATE:
                commands += _append_unique_value(env_name, value)
            else:
                commands += _prepend_unique_value(env_name, value)
    else:
        raise RuntimeError(
            'contains an unknown environment hook type: ' + type_)
    return commands


env_state = {}


def _append_unique_value(name, value):
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # append even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional leading separator
    extend = FORMAT_STR_USE_ENV_VAR.format_map({'name': name}) + os.pathsep
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': extend + value})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


def _prepend_unique_value(name, value):
    global env_state
    if name not in env_state:
        if os.environ.get(name):
            env_state[name] = set(os.environ[name].split(os.pathsep))
        else:
            env_state[name] = set()
    # prepend even if the variable has not been set yet, in case a shell script sets the
    # same variable without the knowledge of this Python script.
    # later _remove_ending_separators() will cleanup any unintentional trailing separator
    extend = os.pathsep + FORMAT_STR_USE_ENV_VAR.format_map({'name': name})
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value + extend})
    if value not in env_state[name]:
        env_state[name].add(value)
    else:
        if not _include_comments():
            return []
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


# generate commands for removing prepended underscores
def _remove_ending_separators():
    # do nothing if the shell extension does not implement the logic
    if FORMAT_STR_REMOVE_TRAILING_SEPARATOR is None:
        return []

    global env_state
    commands = []
    for name in env_state:
        # skip variables that already had values before this script started prepending
        if name in os.environ:
            continue
        commands += [
            FORMAT_STR_REMOVE_LEADING_SEPARATOR.format_map({'name': name}),
            FORMAT_STR_REMOVE_TRAILING_SEPARATOR.format_map({'name': name})]
    return commands


def _set(name, value):
    global env_state
    env_state[name] = value
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    return [line]


def _set_if_unset(name, value):
    global env_state
    line = FORMAT_STR_SET_ENV_VAR.format_map(
        {'name': name, 'value': value})
    if env_state.get(name, os.environ.get(name)):
        line = FORMAT_STR_COMMENT_LINE.format_map({'comment': line})
    return [line]


if __name__ == '__main__':  # pragma: no cover
    try:
        rc = main()
    except RuntimeError as e:
        print(str(e), file=sys.stderr)
        rc = 1
    sys.exit(rc)

--- FILE: ./mowerbot_interfaces/srv/SetDriveMode.srv ---
uint8 mode 
---
bool success
--- FILE: ./mowerbot_interfaces/srv/GenerateCoveragePath.srv ---
# GenerateCoveragePath.srv

# ==========================================
# 請求 (Request)：由 Manager 發送給 F2C 伺服器
# ==========================================
# 輸入剛由 SLAM 地圖提取出的農地邊界多邊形
geometry_msgs/Polygon boundary

# 割草機刀盤的有效切割寬度 (公尺)，這將決定 F2C 算出的弓字型路徑間距
float32 tool_width

# 車體的最小迴轉半徑 (公尺)。針對 74kg 的載具需要給予合理的轉彎半徑，避免打滑或轉向過度
float32 turning_radius

---
# ==========================================
# 回應 (Response)：由 F2C 伺服器回傳給 Manager
# ==========================================
# 規劃完成的完整覆蓋路徑，包含數百個精確的 Pose 座標點
nav_msgs/Path coverage_path

# 標記路徑規劃是否成功 (例如邊界異常時回傳 False)
bool success
--- FILE: ./mowerbot_interfaces/msg/MowerStatus.msg ---
#電源
float32 battery_voltage #電池電壓
float32 battery_current #即時電流
float32 battery_percentage #電池電量

#安全系統
bool bumper_pressed #實體保險桿是否觸發
bool stop_active #緊急停止按鈕是否觸發
bool is_overheated #馬達或驅動是否過熱

#作業狀態
uint8 mode #0:移動建圖 1:導航割草 2:搖桿控制割草





--- FILE: ./mowerbot_interfaces/msg/MotorStatus.msg ---
float32 left_front_rpm #左前轉速
float32 right_front_rpm #右前轉速
float32 left_behind_rpm #左後轉速
float32 right_behind_rpm #右後轉速

int32 left_front_encoder #左前編碼器累計數值
int32 right_front_encoder #右前編碼器累計數值
int32 left_behind_encoder #左後編碼器累計數值
int32 right_behind_encoder #右後編碼器累計數值

--- FILE: ./mowerbot_interfaces/CMakeLists.txt ---
cmake_minimum_required(VERSION 3.8)
project(mowerbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找必要的依賴
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # <== 新增：為了支援 Path 格式

# 宣告你要編譯哪些 Message 檔案
set(msg_files
  "msg/MowerStatus.msg"
  "msg/MotorStatus.msg"
)

# 宣告你要編譯哪些 Service 檔案
set(srv_files
  "srv/SetDriveMode.srv"
  "srv/GenerateCoveragePath.srv" # <== 新增：我們剛剛寫的新服務
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 產生介面程式碼
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs nav_msgs # <== 新增：加入 nav_msgs 作為依賴
)

ament_package()
--- FILE: ./joy_tester/joy_tester/setup.py ---
from setuptools import setup

package_name = 'joy_tester'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Newans',
    maintainer_email='josh.newans@gmail.com',
    description='Simple GUI tool for testing joysticks/gamepads',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_joy = joy_tester.test_joy:main',
        ],
    },
)

--- FILE: ./joy_tester/joy_tester/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>joy_tester</name>
  <version>0.0.2</version>
  <description>Simple GUI tool for testing joysticks/gamepads</description>
  <maintainer email="josh.newans@gmail.com">Josh Newans</maintainer>
  <license>Apache License 2.0</license>


  <depend>sensor_msgs</depend>
  <depend>python3-tk</depend>
  <depend>rclpy</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

--- FILE: ./joy_tester/joy_tester/joy_tester/__init__.py ---

--- FILE: ./joy_tester/joy_tester/joy_tester/test_joy.py ---
# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from tkinter import Canvas, Tk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy


class JoyButton:
    def __init__(self, parent_canvas, left_space, v_space, height, i):
        self.parent_canvas = parent_canvas
        lf = left_space + 15
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space, t+height/2, text=str(i))
        self.circle_obj = self.parent_canvas.create_oval(lf,
                                                         t,
                                                         lf + height,
                                                         t + height,
                                                         width=2, fill='white')

    def update_value(self, value):
        if (value > 0):
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FF0000')
        else:
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FFFFFF')


class JoyAxis:
    def __init__(self, parent_canvas, left_space, v_space, height, width, i):
        self.parent_canvas = parent_canvas
        self.left_space = left_space
        self.v_space = v_space
        self.height = height
        self.width = width
        self.i = i

        lf = left_space + 60
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space+50, t+height/2, text=str(i))

        self.fill_obj = self.parent_canvas.create_rectangle(lf,
                                                            t,
                                                            lf + width,
                                                            t + height,
                                                            width=0, fill='green')
        self.outline_obj = self.parent_canvas.create_rectangle(lf,
                                                               t,
                                                               lf + width,
                                                               t + height,
                                                               width=2, outline='black')

        self.val_txt = self.parent_canvas.create_text(left_space+60 + width + 30,
                                                      t+height/2,
                                                      text=str(i))

    def update_value(self, value):
        lf = self.left_space + 60
        t = self.v_space + (self.height + self.v_space)*self.i

        ww = self.width * (value + 1)/2

        self.parent_canvas.coords(self.fill_obj, lf, t, lf+ww, t+self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=str(f'{value:.3f}'))


class JoyTester(Node):

    def __init__(self):
        super().__init__('test_joy')
        self.get_logger().info('Testing Joystick...')

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 5)
        self.subscription  # prevent unused variable warning

        self.buttons = []
        self.axes = []
        self.initialised = False

        self.tk = Tk()

        self.canvas = Canvas(self.tk, width=800, height=480)
        self.tk.title('Joystick Test')
        self.tk.geometry('800x480+0+0')
        self.canvas.pack(anchor='nw')

        self.tk.update()

    def joy_callback(self, joy_msg):

        left_space = 10
        height = 25
        width = 80
        v_space = 5

        # Handle first receive

        if not self.initialised:
            for i, val in enumerate(joy_msg.buttons):
                self.buttons.append(JoyButton(self.canvas, left_space, v_space, height, i))

            for i, val in enumerate(joy_msg.axes):
                self.axes.append(JoyAxis(self.canvas, left_space, v_space, height, width, i))

            self.initialised = True

        # Update Values

        for i, val in enumerate(joy_msg.buttons):
            self.buttons[i].update_value(val)

        for i, val in enumerate(joy_msg.axes):
            self.axes[i].update_value(val)

        # Redraw
        self.tk.update()

        return


def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()

    try:
        rclpy.spin(joy_tester)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')

    print('Exiting...')

    joy_tester.destroy_node()
    rclpy.try_shutdown()

--- FILE: ./joy_tester/joy_tester/test/test_flake8.py ---
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

--- FILE: ./joy_tester/joy_tester/test/test_copyright.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'

--- FILE: ./joy_tester/joy_tester/test/test_pep257.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'

--- FILE: ./mowerbot_planner/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_planner</name>
  <version>0.0.0</version>
  <description>Fields2Cover Path Planning Node</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>mowerbot_interfaces</depend>
  <depend>fields2cover</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
--- FILE: ./mowerbot_planner/src/f2c_server.cpp ---
#include <rclcpp/rclcpp.hpp>
#include <mowerbot_interfaces/srv/generate_coverage_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// 引入 Fields2Cover 核心函式庫
#include <fields2cover.h>

using GenerateCoveragePath = mowerbot_interfaces::srv::GenerateCoveragePath;
using std::placeholders::_1;
using std::placeholders::_2;

class F2CServer : public rclcpp::Node {
public:
    F2CServer() : Node("f2c_server") {
        srv_ = this->create_service<GenerateCoveragePath>(
            "generate_coverage_path",
            std::bind(&F2CServer::handle_request, this, _1, _2)
        );
        RCLCPP_INFO(this->get_logger(), "🚀 F2C 割草線伺服器已啟動，等待邊界輸入...");
    }

private:
    rclcpp::Service<GenerateCoveragePath>::SharedPtr srv_;

    void handle_request(const std::shared_ptr<GenerateCoveragePath::Request> request,
                        std::shared_ptr<GenerateCoveragePath::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "📥 收到路徑規劃請求！刀盤寬度: %.2f m", request->tool_width);

        if (request->boundary.points.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "邊界頂點數量不足 (需大於等於3)！");
            response->success = false;
            return;
        }

        // ==========================================
        // 1. 轉換 ROS 2 多邊形至 F2C 格式
        // ==========================================
        f2c::types::LinearRing ring;
        for (const auto& p : request->boundary.points) {
            ring.addPoint(p.x, p.y);
        }
        
        auto start_p = request->boundary.points.front();
        auto end_p = request->boundary.points.back();
        if (start_p.x != end_p.x || start_p.y != end_p.y) {
            ring.addPoint(start_p.x, start_p.y);
        }

        f2c::types::Cell cell;
        cell.addRing(ring);
        f2c::types::Cells field(cell);

        try {
            // ==========================================
            // 2. 呼叫 Swath Generator
            // ==========================================
            f2c::sg::BruteForce sg;
            // 回傳的型態是 SwathsByCells (多區塊的割草線集合)
            auto swaths_by_cells = sg.generateSwaths(0.0, request->tool_width, field);

            // ==========================================
            // 3. 手動組裝 Boustrophedon (弓字型) 導航航點
            // ==========================================
            nav_msgs::msg::Path ros_path;
            ros_path.header.stamp = this->now();
            ros_path.header.frame_id = "map"; 

            // 確保有生成成功的割草線
            if (swaths_by_cells.size() > 0) {
                // 【關鍵修正】：我們只有一塊草地，所以剝開第一層，取出真正的 Swaths 群組
                auto swaths = swaths_by_cells[0]; 
                
                bool reverse = false;
                for (size_t i = 0; i < swaths.size(); ++i) {
                    // 這裡拿出的才是真正單一條的 f2c::types::Swath
                    auto swath = swaths[i]; 
                    
                    f2c::types::Point p1 = reverse ? swath.endPoint() : swath.startPoint();
                    f2c::types::Point p2 = reverse ? swath.startPoint() : swath.endPoint();

                    double yaw = std::atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);

                    geometry_msgs::msg::PoseStamped pose1;
                    pose1.header = ros_path.header;
                    pose1.pose.position.x = p1.getX();
                    pose1.pose.position.y = p1.getY();
                    pose1.pose.orientation.x = q.x();
                    pose1.pose.orientation.y = q.y();
                    pose1.pose.orientation.z = q.z();
                    pose1.pose.orientation.w = q.w();
                    ros_path.poses.push_back(pose1);

                    geometry_msgs::msg::PoseStamped pose2 = pose1; 
                    pose2.pose.position.x = p2.getX();
                    pose2.pose.position.y = p2.getY();
                    ros_path.poses.push_back(pose2);

                    reverse = !reverse; 
                }
            }

            response->coverage_path = ros_path;
            response->success = true;
            
            RCLCPP_INFO(this->get_logger(), "🎉 割草線計算成功！共生成 %zu 個 Nav2 關鍵航點。", ros_path.poses.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Fields2Cover 運算失敗: %s", e.what());
            response->success = false;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<F2CServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}】
--- FILE: ./mowerbot_planner/CMakeLists.txt ---
cmake_minimum_required(VERSION 3.8)
project(mowerbot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 尋找必要的 ROS 2 依賴套件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mowerbot_interfaces REQUIRED)

# 2. 尋找 Fields2Cover 函式庫 (【關鍵修正】：首字母大寫)
find_package(Fields2Cover REQUIRED)

# 3. 定義要編譯的 C++ 節點
add_executable(f2c_server src/f2c_server.cpp)
target_include_directories(f2c_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 4. 連結所有需要的函式庫
ament_target_dependencies(f2c_server
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  mowerbot_interfaces
)
# 【關鍵修正】：連結目標名稱也要大寫
target_link_libraries(f2c_server Fields2Cover::Fields2Cover)

# 5. 設定安裝路徑
install(TARGETS
  f2c_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
--- FILE: ./mowerbot_bridge/setup.py ---
from setuptools import find_packages, setup
import os
from glob import glob  # 修正 1: 改成從 glob 引入 glob 函數

package_name = 'mowerbot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a0987747836@gmail.com',
    description='MowerBot 橋接與控制核心套件',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 修正 4: 對應你內層資料夾中的檔案名稱
            'teleop_node = mowerbot_bridge.teleop:main',
        ],
    },
)
--- FILE: ./mowerbot_bridge/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_bridge</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>mowerbot_interfaces</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <exec_depend>joy</exec_depend>
  
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

--- FILE: ./mowerbot_bridge/test/test_flake8.py ---
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

--- FILE: ./mowerbot_bridge/test/test_copyright.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'

--- FILE: ./mowerbot_bridge/test/test_pep257.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'

--- FILE: ./mowerbot_bridge/mowerbot_bridge/__init__.py ---

--- FILE: ./mowerbot_bridge/mowerbot_bridge/bridge_node.py ---
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from mowerbot_interfaces.msg import MowerStatus

 #class MowerBridge(Node):
#     def __init__(self):
#         super().__init__('mower_ackermann_bridge')
#         self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)


#         #self.wheels_base = 
#         #self.whheel_radius = 
--- FILE: ./mowerbot_bridge/mowerbot_bridge/teleop.py ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy                         # 接收手把原始資料
from geometry_msgs.msg import Twist                     # 發布速度指令 (linear.x, angular.z)
from mowerbot_interfaces.srv import SetDriveMode        # 自定義的模式切換服務
                                                        
class MowerTeleop(Node):
    def __init__(self):
        super().__init__('mower_teleop')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('scale_linear', 0.7),('scale_angular', 1.2),
                ('axis_linear', 1),('axis_angular',0),
                ('button_a', 0),('button_b', 1),('button_x',2),('button_y',3),('button_deadman',4),('button_stop',5)
            ]
        )


        self.p = {k: self.get_parameter(k).value for k in ['scale_linear' ,'scale_angular' ,
         'axis_linear' , 'axis_angular' , 
         'button_a' , 'button_b' , 'button_x' , 'button_y' , 'button_deadman','button_stop'
        ]
        }

        # 訂閱 joy 話題，讀取手把硬體狀態
        self.joy_sub = self.create_subscription(Joy,'joy',self.joy_callback,10)
        # 發布 cmd_vel_joy，給 Manager 
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel_joy',10)
        self.mode_client = self.create_client(SetDriveMode ,'change_mower_mode')

        self.last_button = [0]*12

        #手把鎖定
        #False表示目前是鎖定
        self.get_logger().info('手把目前為鎖定狀態，按下LB即可解鎖，解鎖後選可擇模式：A（建圖）, B（F2C）, X(手動), Y（導航）')
        #按鈕B是：1 按鈕A是：0 按鈕X是：2 按鈕Y是：3
        #滾輪左右是0 上下是1 左LB是4 右LB是5

    def joy_callback(self, data):
        if self.last_button is None:
            self.last_button = list(data.buttons)
            return
        
        is_estop_pressed = data.buttons[self.p['button_stop']] == 1
        if is_estop_pressed:
            # 立即發布全 0 的 Twist 強制煞車
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.vel_pub.publish(stop_twist)
        # 確保按鍵按下的瞬間 (Rising edge) 才呼叫服務，避免持續按住造成服務塞車
            if self.last_button[self.p['button_stop']] == 0:
                # 第二道防線：通知 Manager 進入急停模式
                # Manager 收到此模式後，必須拒絕轉發任何後續的移動指令
                self.call_service(4) 
                self.get_logger().error('觸發緊急停止,所有功能已鎖定')
            self.last_button = list(data.buttons)
            return

        LB_ishold = data.buttons[self.p['button_deadman']] ==1
        if LB_ishold:            
                if data.buttons[self.p['button_a']] == 1 and self.last_button[self.p['button_a']] == 0:
                    self.call_service(0) #建圖
                    self.get_logger().info('切換至建圖模式')
                elif data.buttons[self.p['button_b']] == 1 and self.last_button[self.p['button_b']] == 0:
                    self.call_service(1) #F2C
                    self.get_logger().info('切換至 F2C 模式')
                elif data.buttons[self.p['button_x']] == 1 and self.last_button[self.p['button_x']] == 0:
                    self.call_service(2) #手動
                    self.get_logger().info('切換至手動模式')
                elif data.buttons[self.p['button_y']] == 1 and self.last_button[self.p['button_y']] == 0:
                    self.call_service(3) #導航
                    self.get_logger().info('切換導航模式')
        #處理速度輸出
        twist = Twist()
        linear_val = data.axes[self.p['axis_linear']]
        angular_val = data.axes[self.p['axis_angular']]
        if abs(linear_val) < 0.05: linear_val = 0.0
        if abs(angular_val) < 0.05: angular_val = 0.0
        twist.linear.x = linear_val * self.p['scale_linear']            #指令速度(Twist)=搖桿推動量(Joy Data)x最大速限(Scale)
        twist.angular.z = angular_val * self.p['scale_angular']

        self.vel_pub.publish(twist)
        self.last_button = list(data.buttons)

    def call_service(self,mode):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('服務連線失敗')
            return 
        req = SetDriveMode.Request()
        req.mode = mode
        self.mode_client.call_async(req)
        self.get_logger().info(f'發送切換指令：模式 {mode}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MowerTeleop())
    rclpy.shutdown()
--- FILE: ./mowerbot_description/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

--- FILE: ./mowerbot_description/CMakeLists.txt ---
cmake_minimum_required(VERSION 3.8)
project(mowerbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY
    urdf
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

--- FILE: ./mowerbot_description/launch/display.launch.py ---
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

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
--- FILE: ./mowerbot_description/launch/robot_state_publisher.launch.py ---
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 取得套件路徑
    pkg_path = get_package_share_directory('mowerbot_description')
    
    # 讀取並處理 Xacro (指向你的主檔案 car.xacro)
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    # 定義發布器節點
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
--- FILE: ./mowerbot_description/urdf/car_base.xacro ---
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mowerbot">

    <xacro:property name="car_length" value="0.95"/>
    <xacro:property name="car_width" value="0.46"/>
    <xacro:property name="car_height" value="0.46"/>
    <xacro:property name="car_mass" value="73.4"/> 
    <xacro:property name="ground_height" value="0.2"/>

    <xacro:include filename="car_inertia.xacro"/>
    <xacro:include filename="car_wheels.xacro"/>

    <material name="yellow">
        <color rgba="0.9 0.6 0.0 0.7"/>
    </material>

    <link name="base_footprint"/>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${car_length} ${car_width} ${car_height}"/>
            </geometry>
            <material name="yellow"/>
        </visual>
        <collision>
            <geometry>
                <box size="${car_length} ${car_width} ${car_height}"/>
            </geometry>
        </collision>
        <xacro:box_inertia m="${car_mass}" w="${car_length}" h="${car_width}" d="${car_height}" o_xyz="0 0 -0.18"/>
    </link>

    <joint name="base_to_footprint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <origin xyz="0.0 0.0 ${car_height / 2.0 + ground_height}" rpy="0 0 0"/>
    </joint>

    <gazebo>
    <plugin name="mowerbot_diff_drive" filename="libgazebo_ros_diff_drive.so">
        <left_joint>left_rear_wheel_joint</left_joint>
        <right_joint>right_rear_wheel_joint</right_joint>
        <wheel_separation>0.58</wheel_separation>
        <wheel_diameter>0.34</wheel_diameter>
        <max_wheel_torque>200</max_wheel_torque> 
        <max_wheel_acceleration>1.0</max_wheel_acceleration>
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        
        <robot_base_frame>base_footprint</robot_base_frame>

        <update_rate>30</update_rate>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        
        <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
    </gazebo>

</robot>
--- FILE: ./mowerbot_description/urdf/car_inertia.xacro ---
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="box_inertia" params="m w h d o_xyz:='0 0 0' o_rpy:='0 0 0'">
        <inertial>
            <origin xyz="${o_xyz}" rpy="${o_rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0" 
                     iyy="${(m/12) * (w*w + d*d)}" iyz="0.0" 
                     izz="${(m/12) * (w*w + h*h)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertia" params="m r h o_xyz:='0 0 0' o_rpy:='0 0 0'">
        <inertial>
            <origin xyz="${o_xyz}" rpy="${o_rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy="0" ixz="0" 
                     iyy="${(m/12) * (3*r*r + h*h)}" iyz="0" 
                     izz="${(m/2) * (r*r)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="sphere_inertia" params="m r o_xyz:='0 0 0' o_rpy:='0 0 0'">
        <inertial>
            <origin xyz="${o_xyz}" rpy="${o_rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(2/5) * m * (r*r)}" ixy="0.0" ixz="0.0" 
                     iyy="${(2/5) * m * (r*r)}" iyz="0.0" 
                     izz="${(2/5) * m * (r*r)}" />
        </inertial>
    </xacro:macro>
</robot>
--- FILE: ./mowerbot_description/urdf/car_radar.xacro ---
<!-- 雷達 -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 雷達尺寸 -->
    <xacro:property name="radar_radius" value="0.1"/>
    <xacro:property name="radar_height" value="0.1"/>

    <!-- 雷達偏移量 -->
    <xacro:property name="radar_joint_x" value="0.0"/>
    <xacro:property name="radar_joint_y" value="0.0"/>
    <xacro:property name="radar_joint_z" value="${car_height / 2.0 + radar_height / 2.0}"/>

    <!-- 顏色 -->
    <material name="blue">
        <color rgba="0.2 0.0 0.7 0.8"/>
    </material>

    <!-- 雷達幾何-->
    <link name="radar">
        <visual>
            <geometry>
                <cylinder radius="${radar_radius}" length="${radar_height}"/>
            </geometry>
            <material name="blue"/>
        </visual>
    </link>

    <!-- 雷達關聯到base_link -->
    <joint name="radar2base_link" type="fixed">
        <parent link="base_link"/>
        <child link="radar"/>
        <origin xyz="${radar_joint_x} ${radar_joint_y} ${radar_joint_z}"/>
    </joint>

    <gazebo reference="radar">
        <material>Gazebo/Blue</material>

        <sensor name="lidar" type="ray">
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>10.0</update_rate> <ray>
                <scan>
                    <horizontal>
                        <samples>360</samples> <resolution>1.0</resolution>
                        <min_angle>-3.1415926</min_angle> <max_angle>3.1415926</max_angle>  </horizontal>
                </scan>
                <range>
                    <min>0.15</min> <max>12.0</max> <resolution>0.01</resolution> </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev> </noise>
            </ray>

            <plugin name="scan_node" filename="libgazebo_ros_ray_sensor.so">
                <ros>
                    <remapping>~/out:=scan</remapping>
                </ros>
                <output_type>sensor_msgs/LaserScan</output_type>
                <frame_name>radar</frame_name>
            </plugin>
        </sensor>
    </gazebo>
</robot>
--- FILE: ./mowerbot_description/urdf/car_wheels.xacro ---
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="car_inertia.xacro"/>
    <xacro:property name="PI" value="3.1415926535897931"/>

    <xacro:property name="body_length" value="0.95"/>
    <xacro:property name="body_width" value="0.46"/>
    <xacro:property name="body_height" value="0.46"/>

    <xacro:property name="wheel_radius" value="0.17"/> 
    <xacro:property name="wheel_width" value="0.1"/> 
    <xacro:property name="wheel_mass" value="1.0"/> 
    
    <xacro:macro name="visual_wheel" params="prefix">
        <link name="${prefix}_wheel">
            <visual>
                <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <material name="black">
                    <color rgba="0.1 0.1 0.1 1.0"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${PI/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
            </collision>
            <xacro:cylinder_inertia m="${wheel_mass}" r="${wheel_radius}" h="${wheel_width}" o_rpy="${PI/2} 0 0"/>
        </link>
    </xacro:macro>

    <xacro:macro name="add_rear_wheel" params="prefix y_reflect">
        <xacro:visual_wheel prefix="${prefix}_rear" />
        <joint name="${prefix}_rear_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="${prefix}_rear_wheel"/>
            <origin xyz="-0.35 ${y_reflect * (body_width/2 + wheel_width/2 + 0.01)} -0.26" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>

        <gazebo reference="${prefix}_rear_wheel">
            <mu1>2.0</mu1>      <mu2>2.0</mu2>      <kp>1000000.0</kp>  <kd>10.0</kd>  <minDepth>0.001</minDepth>     </gazebo>
    </xacro:macro>

    <xacro:add_rear_wheel prefix="left"  y_reflect="1" />
    <xacro:add_rear_wheel prefix="right" y_reflect="-1" />

    <xacro:macro name="add_front_wheel" params="prefix y_reflect">
        <xacro:visual_wheel prefix="${prefix}_front" />
        <joint name="${prefix}_front_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="${prefix}_front_wheel"/>
            <origin xyz="0.35 ${y_reflect * (body_width/2 + wheel_width/2 + 0.01)} -0.26" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>

        <gazebo reference="${prefix}_front_wheel">
            <mu1>0.1</mu1>      
            <mu2>0.0</mu2>      
            <kp>1000000.0</kp>  
            <kd>100.0</kd>    
            <minDepth>0.001</minDepth>  
        </gazebo>
    </xacro:macro>

    <xacro:add_front_wheel prefix="left"  y_reflect="1" />
    <xacro:add_front_wheel prefix="right" y_reflect="-1" />
</robot>
--- FILE: ./mowerbot_description/urdf/car_imu.xacro ---
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <link name="imu_link">
        <visual>
            <geometry>
                <box size="0.02 0.02 0.02"/>
            </geometry>
            <material name="black">
                <color rgba="0.0 0.0 0.0 1.0"/>
            </material>
        </visual>
    </link>

    <joint name="imu_joint" type="fixed">
        <parent link="base_link"/>
        <child link="imu_link"/>
        <origin xyz="0 0 ${car_height / 2.0}" rpy="0 0 0"/>
    </joint>

    <gazebo reference="imu_link">
        <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
            <visualize>true</visualize>
            
            <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
                <ros>
                    <remapping>~/out:=imu</remapping>
                </ros>
                <frame_name>imu_link</frame_name>
            </plugin>
        </sensor>
    </gazebo>

</robot>
--- FILE: ./mowerbot_description/urdf/car_camera.xacro ---
<!-- camera -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 攝像機尺寸-->
    <xacro:property name="camera_length" value="0.11"/>
    <xacro:property name="camera_width" value="0.25"/>
    <xacro:property name="camera_height" value="0.08"/>

    <!-- 攝像機偏移量 -->
    <xacro:property name="camera_joint_x" value="${car_length / 2.0 - 0.02}"/>
    <xacro:property name="camera_joint_y" value="0.0"/>
    <xacro:property name="camera_joint_z" value="${car_height / 2.0 + camera_height / 2.0}"/>

    <!-- 顏色 -->
    <material name="red">
        <color rgba="1.0 0.0 0.0 0.8"/>
    </material>

    <!-- 攝像機幾何 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="${camera_length} ${camera_width} ${camera_height}"/>
            </geometry>
            <material name="red"/>
        </visual>
    </link>

    <!-- 攝像機關聯到base_link -->
    <joint name="camera2base_link" type="fixed">
        <parent link="base_link"/>
        <child link="camera"/>
        <origin xyz="${camera_joint_x} ${camera_joint_y} ${camera_joint_z}"/>
    </joint>


    <gazebo reference="camera">
        <material>Gazebo/Red</material>
        
        <sensor name="main_camera" type="camera">
            <always_on>true</always_on>
            <visualize>true</visualize> 
            <update_rate>30.0</update_rate> <camera name="head">
                <horizontal_fov>1.3962634</horizontal_fov> <image>
                    <width>640</width>  <height>480</height> <format>R8G8B8</format> </image>
                <clip>
                    <near>0.02</near> <far>300</far>    </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev> 
                </noise>
            </camera>
            
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <alwaysOn>true</alwaysOn>
                <updateRate>0.0</updateRate> <cameraName>camera</cameraName>
                <imageTopicName>image_raw</imageTopicName> 
                <cameraInfoTopicName>camera_info</cameraInfoTopicName>
                <frameName>camera</frameName> 
            </plugin>
        </sensor>
    </gazebo>

</robot>
--- FILE: ./mowerbot_description/urdf/car.xacro ---
<!-- 整車 -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="main_car">
    <!-- PI -->
    <xacro:property name="PI" value="3.141592653"/>

    <!-- 包含子文件, 要在參數定義之後, 子文件才能使用父文件的参數 -->
    <xacro:include filename="car_base.xacro"/>
    <xacro:include filename="car_radar.xacro"/>
    <xacro:include filename="car_camera.xacro"/>
    <xacro:include filename="car_imu.xacro"/>
</robot>



<!--輪子半徑 0.17m 車子總長約 0.95x0.46x0.46 輪胎 1kg 車體57.2kg 電池16.2kg-->
--- FILE: ./mowerbot_action/setup.py ---
from setuptools import find_packages, setup

package_name = 'mowerbot_action'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a0987747836@gmail.com',
    description='MowerBot 決策與管理套件',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 關鍵點：'執行檔名稱 = 套件名.檔名:函數名'
            'mower_manager = mowerbot_action.manager:main',
            'map_to_boundary = mowerbot_action.map_to_boundary:main',
        ],
    },
)
--- FILE: ./mowerbot_action/package.xml ---
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mowerbot_action</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="a0987747836@gmail.com">a</maintainer>
  <license>TODO: License declaration</license>

  <depend>nav2_msgs</depend>
  <depend>rclpy</depend>
  <depend>mowerbot_interfaces</depend>
  <depend>geometry_msgs</depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

--- FILE: ./mowerbot_action/test/test_flake8.py ---
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

--- FILE: ./mowerbot_action/test/test_copyright.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'

--- FILE: ./mowerbot_action/test/test_pep257.py ---
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'

--- FILE: ./mowerbot_action/mowerbot_action/__init__.py ---

--- FILE: ./mowerbot_action/mowerbot_action/map_to_boundary.py ---
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Point32
# 導入 ROS 2 關鍵的 QoS 套件
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import cv2

class MapToBoundaryNode(Node):
    def __init__(self):
        super().__init__('map_to_boundary')
        
        # 【核心修正】：配置與 slam_toolbox 完美的 Transient Local QoS Profile
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,   # 必須是 TRANSIENT_LOCAL
            reliability=ReliabilityPolicy.RELIABLE         # 必須是 RELIABLE
        )
        
        # 使用修正後的 QoS 訂閱地圖
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        
        self.polygon_pub = self.create_publisher(PolygonStamped, '/f2c_boundary', 10)
        
        self.get_logger().info('✅ 邊界提取節點已成功掛載 Transient Local QoS，開始監聽 /map...')

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # 終端機 Debug 訊號一：證明回呼函式成功進來了
        self.get_logger().info(f'📥 成功接收到地圖數據！尺寸: {width}x{height}, 解析度: {resolution}')
        
        # 1. 轉換為 NumPy 二維陣列
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 2. 影像二值化
        # 排除 -1(未知) 與大於 20(有障礙物) 的區域，把 0~20 都當作安全草地
        img = np.zeros((height, width), dtype=np.uint8)
        free_space_mask = (grid >= 0) & (grid <= 20)
        img[free_space_mask] = 255 
        
        # 【影像防錯微調】：給影像四周加上 1 畫素的黑邊，防止草地頂到地圖邊緣導致 OpenCV 找不到輪廓
        padded_img = cv2.copyMakeBorder(img, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
        
        # 3. 尋找輪廓
        contours, _ = cv2.findContours(padded_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('⚠️ 地圖中尚未發現明確的連續草地空白輪廓。')
            return
            
        # 找出面積最大的輪廓
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 4. 多邊形近似 (簡化頂點數量)
        epsilon = 0.01 * cv2.arcLength(largest_contour, True)
        approx_polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
        
        # 5. 封裝成 ROS 訊息並轉換座標系
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = self.get_clock().now().to_msg()
        poly_msg.header.frame_id = 'map' 
        
        for point in approx_polygon:
            # 因為前面加了 1 畫素的 padded 邊框，這裡座標要減 1 扣回來
            u = point[0][0] - 1
            v = point[0][1] - 1
            
            # 像素轉世界座標
            world_x = origin_x + (u * resolution)
            world_y = origin_y + (v * resolution)
            
            # 把 Z 軸抬高 0.15 公尺 (15公分)，確保絕對浮在灰色地圖上方，免除視覺遮擋
            p32 = Point32(x=float(world_x), y=float(world_y), z=0.15)
            poly_msg.polygon.points.append(p32)
            
        self.polygon_pub.publish(poly_msg)
        
        # 終端機 Debug 訊號二：證明邊界順利算完並送出
        self.get_logger().info(f'🎉 邊界提取成功！共化簡出 {len(approx_polygon)} 個多邊形頂點，已發布至 /f2c_boundary')

def main(args=None):
    rclpy.init(args=args)
    node = MapToBoundaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
--- FILE: ./mowerbot_action/mowerbot_action/manager.py ---
#!/usr/bin/env python3
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path
from mowerbot_interfaces.srv import GenerateCoveragePath
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import FollowPath
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mowerbot_interfaces.srv import SetDriveMode

class MowerManager(Node):
    def __init__(self):
        super().__init__('mower_manager')
        # 預設為手動模式
        self.nav_client = ActionClient(self, FollowPath, 'follow_path')
        # 發布者：將 F2C 算出的全局路徑發布給 RViz2 顯示
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/f2c_path', qos_profile)
        self.current_mode = 2
        self.mode_map = {
            0: '建圖模式(SLAM)',
            1: 'F2C', 
            2: '手動模式',
            3: '自動導航(Nav)',
            4: '緊急停止(E-STOP)'
        }

        # 安全計時器 如果0.5秒內沒收到任何速度指令則停止馬達
        self.last_cmd_time = self.get_clock().now()
        self.watchdog_timeout = 0.5
        # 發布者：發給真正的底盤驅動
        self.real_vel_pub = self.create_publisher(Twist, '/cmd_vel' ,10)

        # 訂閱者：接收來自不同來源的速度指令
        self.joy_sub = self.create_subscription(Twist ,'/cmd_vel_joy', self.joy_vel_cb,10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_vel_cb,10)
        
        self.srv = self.create_service(SetDriveMode,'change_mower_mode',self.change_mode_callback)

        self.timer = self.create_timer(0.05,self.safety_check)
        # 訂閱最新算出的綠色邊界
        self.latest_boundary = None
        self.boundary_sub = self.create_subscription(
            PolygonStamped, '/f2c_boundary', self.boundary_cb, 10)

        # 建立呼叫 C++ F2C 伺服器的 Client
        self.f2c_client = self.create_client(GenerateCoveragePath, 'generate_coverage_path')
        self.get_logger().info('Mower Manager 啟動成功,目前模式：【手動模式】')

    # 速度指令處理
    """
    接收手把速度：僅在手動(2)或建圖(0)模式下轉發
    """
    def joy_vel_cb(self,msg):
        if self.current_mode == 0 or self.current_mode == 2:
            self.publish_and_update(msg)
        # 修正拼字錯誤：cureent_mode -> current_mode
        elif self.current_mode == 4:
            self.handle_estop_violation('手把 (Teleop)')

    """ 
    接收導航速度：僅在 F2C(1) 或自動導航(3) 模式下轉發 
    """
    def nav_vel_cb(self,msg):
        if self.current_mode == 1 or self.current_mode == 3:
            self.publish_and_update(msg)
        elif self.current_mode == 4:
            self.handle_estop_violation('導航系統 (Nav2)')

    def publish_and_update(self,msg):
        self.real_vel_pub.publish(msg)
        self.last_cmd_time = self.get_clock().now()
        
    def handle_estop_violation(self, source_name):
        """
        處理急停狀態下的違規指令。
        確保即使收到指令，底盤依然保持靜止，並使用 throttle_duration 限制日誌刷屏。
        """
        self.stop_robot()
        self.get_logger().warn(
            f'急停鎖定中！攔截到來自 {source_name} 的異常移動指令。', 
            throttle_duration_sec=2.0  # 每 2 秒最多印出一次，避免日誌崩潰
        )
    def boundary_cb(self, msg):
        """隨時更新最新圈出的綠色邊界"""
        self.latest_boundary = msg.polygon

    def call_f2c_planner(self):
        """打包邊界並發送給 C++ 伺服器"""
        if self.latest_boundary is None:
            self.get_logger().error('⚠️ 尚未接收到草地邊界！請先在建圖模式下遙控車輛探索。')
            return

        if not self.f2c_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('⚠️ F2C 伺服器未上線，請確認 f2c_server 已經啟動！')
            return

        # 建立請求
        req = GenerateCoveragePath.Request()
        req.boundary = self.latest_boundary
        req.tool_width = 0.5    # 74kg 割草機的刀盤寬度
        req.turning_radius = 1.0 # 迴轉半徑

        self.get_logger().info('🚀 正在將邊界發送給 F2C 伺服器進行運算...')
        future = self.f2c_client.call_async(req)
        future.add_done_callback(self.f2c_response_callback)

    def f2c_response_callback(self, future):
        """接收 C++ 算好的路徑"""
        try:
            response = future.result()
            if response.success:
                self.path_pub.publish(response.coverage_path)
                self.get_logger().info(f'🎉 成功拿到 F2C 路徑！總航點數: {len(response.coverage_path.poses)}')
                # 下一步：在這裡把路徑交給 Nav2
                self.send_path_to_nav2(response.coverage_path)
            else:
                self.get_logger().error('⚠️ F2C 伺服器回報路徑規劃失敗！')
        except Exception as e:
            self.get_logger().error(f'呼叫 F2C 服務時發生錯誤: {str(e)}')
    # 模式切換服務
    # 模式切換服務
    def change_mode_callback(self,request,response):
        if request.mode in self.mode_map:
            self.current_mode = request.mode
            mode_name = self.mode_map[self.current_mode]
            
            # 加入針對急停的專屬提示
            if self.current_mode == 4:
                self.get_logger().error(f'🚨 系統強制鎖定：切換至 {mode_name}')
            else:
                self.get_logger().info(f'成功切換至 {mode_name}')
                
                # 【新增】：如果切換到 F2C 模式(1)，就自動呼叫算圖
                if self.current_mode == 1:
                    self.call_f2c_planner()
            
            self.stop_robot()
            # 修正：重置 watchdog 計時器，避免切換瞬間報錯
            self.last_cmd_time = self.get_clock().now()
            response.success = True

        else:
            self.get_logger().error(f'無效模式編號:{request.mode}')
            response.success = False
            
        return response
    
    def safety_check(self):
        # 修正：處於急停模式(4)、F2C模式(1)、或自動導航模式(3)時，
        # 直接跳過超時檢查，讓 Nav2 直接掌控車子的油門與煞車！
        if self.current_mode in [1, 3, 4]:
            return

        now = self.get_clock().now()
        elapsed_time = (now-self.last_cmd_time).nanoseconds/1e9

        if elapsed_time > self.watchdog_timeout:
            self.stop_robot()

    def stop_robot(self):
        stop_msg = Twist()
        self.real_vel_pub.publish(stop_msg)
        
    def send_path_to_nav2(self, path_msg):
        """
        將 F2C 算出的路徑打包成 Action Goal 交給 Nav2 底層控制器
        """
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('⚠️ 找不到 Nav2 的 follow_path 伺服器，請確認 Nav2 是否正常啟動！')
            return

        self.get_logger().info('🚀 啟接割草任務！正在將路徑交給 Nav2 控制器...')
        
        # 建立 FollowPath 的目標請求
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = 'FollowPath' # 呼叫 Nav2 預設的循跡控制器

        # 發送非同步 Action 請求
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        確認 Nav2 是否成功接受了我們的循跡任務
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2 拒絕了割草路徑！(可能是路徑起點距離車體太遠)')
            return
        self.get_logger().info('✅ Nav2 已接受路徑，車輛開始移動！')

    def f2c_response_callback(self, future):
        """
        接收 C++ 算好的路徑並廣播與導航
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'🎉 成功拿到 F2C 路徑！總航點數: {len(response.coverage_path.poses)}')
                # 【關鍵】：把路徑廣播出去給 RViz2 畫圖
                self.path_pub.publish(response.coverage_path)
                # 將路徑送給 Nav2 驅動馬達
                self.send_path_to_nav2(response.coverage_path)
            else:
                self.get_logger().error('⚠️ F2C 伺服器回報路徑規劃失敗！')
        except Exception as e:
            self.get_logger().error(f'呼叫 F2C 服務時發生錯誤: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = MowerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()