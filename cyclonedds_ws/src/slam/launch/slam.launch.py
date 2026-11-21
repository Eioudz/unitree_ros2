"""
Go2 实验室建图启动文件（slam_toolbox 2D SLAM）。

数据管线（lidar_source:=mid360，默认；外接 Livox Mid360，背部官方支架）：
    /livox/lidar (PointCloud2, 10Hz, frame: livox_frame, 主机时钟域)
        -> livox_scan（stamp 对齐狗主控时钟域 + 去俯仰水平切片）
        -> /scan (frame: base_link_level)
    /utlidar/robot_odom (nav_msgs/Odometry, 雷达内置 LIO, 狗主控时钟域)
        -> odom_tf_bridge -> TF: odom->base_link / odom->base_link_level
        -> slam_toolbox online_async -> /map + TF: map->odom
    静态 TF: base_link -> livox_frame (背部官方支架安装位，x0 y0 z0.10)

数据管线（lidar_source:=utlidar，原内置雷达）：
    /utlidar/cloud_base (PointCloud2, frame: base_link)
        -> pointcloud_to_laserscan -> /scan (frame: base_link_level)

注意：Mid360 stamp 与 robot_odom 分属两个时钟域（实测差约 155.8 天），
必须经 livox_time_bridge 对齐，否则 slam_toolbox 查不到 TF。

前置条件（另一个终端，先启动机器人）：
    source ~/unitree_ros2/setup.sh
    ros2 launch go2_description go2_real.launch.py use_rviz:=false
    # mid360 源还需在狗上跑驱动（有线直收 10Hz）：
    # ssh unitree@192.168.123.18
    # source /opt/ros/foxy/setup.bash; source ~/cyclonedds_ws/install/setup.bash; \
    # source ~/unitree_mid360/install/setup.bash
    # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    # export CYCLONEDDS_URI=file://$HOME/cyclonedds_ws/cyclonedds_bridge.xml
    # ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    #   -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
    #   -p output_data_type:=0 \
    #   -p user_config_path:=/home/unitree/go2_nav/src/livox_ros_driver2/config/MID360_config.json \
    #   -p frame_id:=livox_frame

启动建图（本终端）：
    source ~/unitree_ros2/setup.sh
    ros2 launch slam slam.launch.py                        # Mid360 源 + 建图 RViz
    ros2 launch slam slam.launch.py lidar_source:=utlidar  # 回退内置雷达
    ros2 launch slam slam.launch.py use_rviz:=false        # 不开 RViz

建图操作：
    遥控机器狗缓慢匀速走遍实验室，回到起点闭环。
    转弯处放慢，避免剧烈俯仰/打滑。

保存地图：
    # 保存导航用栅格地图（推荐，nav2 可直接使用）
    mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_map
    # 保存位姿图（供 slam_toolbox 后续定位模式复用，生成 .posegraph/.data 文件）
    # 注意：humble 的 save_map 服务只存栅格图，位姿图必须用 serialize_map
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/soyo/maps/lab_map}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    slam_pkg = get_package_share_directory('slam')

    mapper_params = os.path.join(slam_pkg, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(slam_pkg, 'rviz', 'slam.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_source = LaunchConfiguration('lidar_source')
    # humble 无 LaunchConfigurationEquals，用 PythonExpression 比较
    is_mid360 = IfCondition(PythonExpression(
        ["'", lidar_source, "' == 'mid360'"]))
    is_utlidar = IfCondition(PythonExpression(
        ["'", lidar_source, "' == 'utlidar'"]))

    # Mid360 切片：时间戳对齐（主机时钟域 -> 狗主控时钟域，与 TF 同源）
    # + 水平切片（高度带 -0.15~0.35，即离地约 0.15~0.65m 障碍物层），
    # 直接产出 /scan。
    # 注意：不用 pointcloud_to_laserscan 包——该包在 launch 方式下实测
    # 时好时坏（参数生效、订阅存在却静默无输出，Node 与 ExecuteProcess
    # 均复现；手动 ros2 run 正常），自研 livox_scan.py 行为完全可控。
    livox_scan = Node(
        package='slam',
        executable='livox_scan.py',
        name='livox_scan',
        output='screen',
        condition=is_mid360,
    )

    # Mid360 安装位静态 TF：背部官方支架，背部中心约高于身体中心 10cm，
    # 雷达向下倾 13°（RPY(0,+13°,0)）。修正前零旋转导致地面被误投为前方立面。
    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_static_tf',
        arguments=['0', '0', '0.10', '0', '0.1132032', '0', '0.9935719',
                   'base_link', 'livox_frame'],
        condition=is_mid360,
    )

    # 内置上盖雷达源（原管线，回退用）；p2l 同样用 ExecuteProcess + -p 传参
    pointcloud_to_laserscan_utlidar = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
            '--ros-args',
            '-r', 'cloud_in:=/utlidar/cloud_base',
            '-r', 'scan:=/scan',
            '-p', 'target_frame:=base_link_level',
            '-p', 'min_height:=-0.15',
            '-p', 'max_height:=0.35',
            '-p', 'range_min:=0.55',
            '-p', 'range_max:=12.0',
        ],
        condition=is_utlidar,
    )

    # 2. LIO 里程计 -> TF
    odom_tf_bridge = Node(
        package='slam',
        executable='odom_tf_bridge.py',
        name='odom_tf_bridge',
    )

    # 3. slam_toolbox 在线异步建图
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapper_params, {'use_sim_time': False}],
    )

    # 4. 建图 RViz（use_rviz:=false 可关闭）
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
            description='是否启动建图 RViz'),
        DeclareLaunchArgument(
            'lidar_source', default_value='mid360',
            description='点云源: mid360(外接 Livox Mid360) / utlidar(内置上盖雷达)'),
        livox_scan,
        livox_static_tf,
        pointcloud_to_laserscan_utlidar,
        odom_tf_bridge,
        slam_toolbox,
        rviz,
    ])
