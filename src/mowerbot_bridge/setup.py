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