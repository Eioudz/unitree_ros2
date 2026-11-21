"""
Go2 真机 RViz 可视化启动文件。

与 go2.launch.py 的区别：
  - 不启动默认的 joint_state_publisher（它只发布零位姿）
  - 启动 go2_joint_state_publisher 节点：把真机 lowstate 中的 motor_state.q
    转换为 sensor_msgs/JointState，让 RViz 中机器人随真机一起动
  - 发布 radar -> utlidar_lidar 恒等 TF，配合 URDF 的 radar_joint
    外参，让真机 L1 雷达点云 /utlidar/cloud 在 RViz 中正确显示
  - 发布 base_link -> base 恒等 TF，把厂商坐标系 base_link
    （/utlidar/cloud_base、/utlidar/robot_odom 的 frame_id）接入 URDF TF 树

使用前请先 source 真机网络环境：
    source ~/unitree_ros2/setup.sh

启动：
    ros2 launch go2_description go2_real.launch.py
    # 建图时关掉本包 RViz，改用 slam 包的 RViz：
    ros2 launch go2_description go2_real.launch.py use_rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_description_pkg = get_package_share_directory("go2_description")

    # 读取 URDF
    with open(os.path.join(go2_description_pkg, "urdf", "go2_description.urdf"), "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        remappings=[("joint_states", "joint_states")],
    )

    # 真机关节状态桥接节点：lowstate -> /joint_states
    joint_state_publisher = Node(
        package="go2_description",
        executable="go2_joint_state_publisher.py",
        parameters=[{"source_topic": "lowstate"},
                    {"publish_topic": "joint_states"}],
    )

    # 点云 frame_id(utlidar_lidar) 与 URDF 中的雷达 link(radar) 同名对齐。
    # base->radar 的外参已由 URDF 的 radar_joint 提供，这里只需恒等变换，
    # 真机 /utlidar/cloud 即可在 RViz 中正确显示。
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "radar",
                   "--child-frame-id", "utlidar_lidar"],
    )

    # 真机厂商坐标系 base_link(utlidar/cloud_base、robot_odom 的 frame_id)
    # 与 URDF 根坐标系 base 同点，恒等变换接入 TF 树：
    # odom -> base_link -> base -> radar -> utlidar_lidar
    base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "base_link",
                   "--child-frame-id", "base"],
    )

    # RViz 配置
    rviz_config = os.path.join(go2_description_pkg, "rviz", "go2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="是否启动本包的 RViz（建图时传 false，用 slam 包的 RViz）"),
        robot_state_publisher,
        joint_state_publisher,
        lidar_tf,
        base_link_tf,
        rviz_node,
    ])
