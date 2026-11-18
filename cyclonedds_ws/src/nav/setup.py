import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soyo',
    maintainer_email='soyo@todo.todo',
    description='Go2 导航：定位 + costmap 导航栈 + 应用层（单点导航）',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_tf_bridge.py = nav.odom_tf_bridge:main',
            'livox_scan.py = nav.livox_scan:main',
            'cmd_vel_sport_bridge = nav.cmd_vel_sport_bridge:main',
            'goto_node = nav.goto_node:main',
        ],
    },
)
