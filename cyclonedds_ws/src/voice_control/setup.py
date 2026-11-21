import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'voice_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soyo',
    maintainer_email='soyo@todo.todo',
    description='Voice control Go2 via DeepSeek',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_node = voice_control.voice_node:main',
        ],
    },
)
