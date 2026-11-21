import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    go2_description_pkg = get_package_share_directory("go2_description")

    # 直接读取 urdf 文件
    with open(os.path.join(go2_description_pkg, "urdf", "go2_description.urdf"), "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )

    # 启动 RViz，并加载配置文件
    rviz_config = os.path.join(go2_description_pkg, "rviz", "go2.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else []
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])