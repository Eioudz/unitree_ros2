"""
Go2 实验室定位启动文件（Nav2 map_server + AMCL，基于已保存地图）。

在 slam_toolbox 建好图并保存后，用本文件替代建图管线做纯定位：
AMCL 订阅 /scan + /map，发布 map->odom TF，狗即在地图坐标系下知晓自身位姿。

数据管线（与 slam.launch.py 的 mid360 建图源一致）：
    /livox/lidar (PointCloud2, frame: livox_frame) -> livox_scan -> /scan
    /utlidar/robot_odom -> odom_tf_bridge -> TF: odom->base_link(_level)
    map_server -> /map（lab_map.yaml 或其他已保存地图）
    amcl -> TF: map->odom，并发布粒子云 /particlecloud

启动（不要与 slam.launch.py 同时运行，两者都会发 map->odom TF）：
    # 前置：先启动机器人（另一个终端）
    source ~/unitree_ros2/setup.sh
    ros2 launch go2_description go2_real.launch.py use_rviz:=false

    source ~/unitree_ros2/setup.sh
    ros2 launch nav localization.launch.py                       # 自带定位 RViz
    ros2 launch nav localization.launch.py use_rviz:=false       # 不开 RViz
    ros2 launch nav localization.launch.py map:=/home/soyo/maps/新地图名.yaml

初始化位姿（必须）：
    RViz 里点工具栏 "2D Pose Estimate"，在地图上按住拖出狗的朝向；
    粒子云 /particlecloud 收敛成一小簇即定位成功。
    无 RViz 时可命令行给定（把 x/y/yaw 换成实际值）：
    ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0},
       orientation: {z: 0.0, w: 1.0}}}}"

验证：
    ros2 run tf2_ros tf2_echo map base_link_level
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav_pkg = get_package_share_directory('nav')

    nav_params = os.path.join(nav_pkg, 'config', 'localization.yaml')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'localization.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')

    # 1. Mid360 点云切片（与建图管线一致：stamp 对齐狗主控时钟域 + 去俯仰水平切片）
    livox_scan = Node(
        package='nav',
        executable='livox_scan.py',
        name='livox_scan',
        output='screen',
    )

    # Mid360 安装位静态 TF（与 slam.launch.py 一致：背部官方支架）
    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_tf',
        arguments=['0', '0', '0.10', '0', '0', '0', '1', 'base_link', 'livox_frame'],
    )

    # 2. LIO 里程计 -> TF: odom->base_link / odom->base_link_level
    odom_tf_bridge = Node(
        package='nav',
        executable='odom_tf_bridge.py',
        name='odom_tf_bridge',
    )

    # 3. 地图服务器（lifecycle 节点，由下方 lifecycle_manager 激活）
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav_params, {'yaml_filename': map_yaml}],
    )

    # 4. AMCL 定位（发布 map->odom TF）
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav_params],
    )

    # 5. lifecycle 管理器：自动激活 map_server + amcl
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[nav_params],
    )

    # 6. 定位 RViz（用 2D Pose Estimate 给初始位姿）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动定位 RViz'),
        DeclareLaunchArgument(
            'map', default_value='/home/soyo/maps/lab_map.yaml',
            description='定位用栅格地图 yaml 路径'),
        livox_scan,
        livox_static_tf,
        odom_tf_bridge,
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
    ])
