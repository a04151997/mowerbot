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
            'boundary_recorder = mowerbot_action.boundary_recorder:main',
            'keepout_server = mowerbot_action.keepout_server:main',
            'keepout_tool = mowerbot_action.keepout_tool:main',
        ],
    },
)