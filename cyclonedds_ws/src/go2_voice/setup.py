from setuptools import find_packages, setup

package_name = 'go2_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soyo',
    maintainer_email='soyo@todo.todo',
    description='Go2 语音控制：Whisper 识别 + DeepSeek 解析，支持前进与转弯',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_voice_node = go2_voice.voice_node:main',
        ],
    },
)
