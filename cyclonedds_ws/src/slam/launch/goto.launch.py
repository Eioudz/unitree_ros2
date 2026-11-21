"""
语义导航一条龙：定位（AMCL + RViz） + goto_node（去前门/去后门）。

启动：
    source ~/unitree_ros2/setup.sh
    ros2 launch slam goto.launch.py

流程：
    1. 前置：另一终端启动机器人（go2_real.launch.py）+ 狗上 Mid360 驱动
    2. RViz 打开后点工具栏 "2D Pose Estimate"，在地图上拖出狗当前位姿，
       粒子云 /particlecloud 收敛成一簇
    3. 再开一个终端发指令（launch 终端被日志占用，stdin 不通）：
       ros2 topic pub --once /goto_cmd std_msgs/msg/String "{data: 去前门}"
       指令：去前门 / 去后门 / 停止
       RViz 中可实时看到狗（/particlecloud、TF）移向目标点
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('slam')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'localization.launch.py')))

    goto = Node(
        package='slam',
        executable='goto_node',
        name='goto_node',
        output='screen',
    )

    return LaunchDescription([
        localization,
        goto,
    ])
