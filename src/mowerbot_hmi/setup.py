from setuptools import find_packages, setup

package_name = 'mowerbot_hmi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 網頁人機介面的畫面 (web.py 讀這一份)
        ('share/' + package_name + '/web', ['web/index.html', 'web/lichtblick_layout.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a0987747836@gmail.com',
    description='mowerbot 的人機介面 (HMI)：模擬用的 PyQt 視窗 (hmi_node) 與實車用的網頁 (web)',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'hmi_node = mowerbot_hmi.hmi_node:main',
            'web = mowerbot_hmi.web:main',
        ],
    },
)
