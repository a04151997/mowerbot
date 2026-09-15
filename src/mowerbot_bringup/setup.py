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
        # 地圖：實際的地圖檔不進版控，但建圖存檔後重新 build 就會被安裝進 share/，
        # localization.launch.py 預設就是去這裡找 mowerbot_map.posegraph
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.pgm') + glob('maps/*.yaml')
            + glob('maps/*.posegraph') + glob('maps/*.data')
            + glob('maps/*.md')),
        # 一鍵存圖腳本，裝到 lib/ 才能用 ros2 run 執行
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
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
