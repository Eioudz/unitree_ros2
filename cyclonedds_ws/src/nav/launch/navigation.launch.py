"""
Go2 导航启动文件：定位（AMCL）+ costmap 组件（全局/局部代价地图）。

启动（不要与 slam.launch.py 同时运行，两者都会发 map->odom TF）：
    # 前置：另一终端启动机器人 + 狗上 Mid360 驱动
    source ~/unitree_ros2/setup.sh
    ros2 launch go2_description go2_real.launch.py use_rviz:=false

    source ~/unitree_ros2/setup.sh
    ros2 launch nav navigation.launch.py                       # 定位 + costmap + RViz
    ros2 launch nav navigation.launch.py use_rviz:=false       # 不开 RViz
    ros2 launch nav navigation.launch.py map:=/home/soyo/maps/新地图名.yaml

包含的组件：
    localization.launch.py：
        livox_scan（/livox/lidar -> /scan）+ odom_tf_bridge（TF odom->base_link）
        map_server（/map）+ amcl（TF map->odom）+ lifecycle_manager_localization
    costmap 栈：
        controller_server -> 局部代价地图 /local_costmap/costmap（滚动 4x4m）+ DWB
        planner_server    -> 全局代价地图 /global_costmap/costmap + /plan
        behavior_server   -> 恢复行为（仅 spin/wait，禁 backup 倒车）
        bt_navigator      -> 行为树导航器，接收 /goal_pose 自动规划+避障
        lifecycle_manager_navigation 自动激活上述四个节点
    运动桥接：
        cmd_vel_sport_bridge -> /cmd_vel 转 1008 MOVE（vx>=0 硬约束 + 看门狗）
        速度链路：DWB -> /cmd_vel -> 运动桥（无 collision_monitor）

自动导航用法（定位初始化后）：
    RViz 工具栏 "2D Goal Pose" 在地图上点目标 -> 狗自动规划路径并绕障走过去
    /plan 显示全局路径，局部避障由 DWB 实时调整；堵死时触发 spin recovery
    语义指令应用层（另行 ros2 run，与本 launch 的 RViz 2D Goal Pose 不要
    同时触发）：
        ros2 run nav goto_node     # 去前门 / 去后门 / 去电视机前 / 停止

验证：
    ros2 topic hz /local_costmap/costmap      # ~1Hz（publish_frequency）
    ros2 topic hz /global_costmap/costmap     # ~0.5Hz
    ros2 topic echo /cmd_vel                  # 导航时有输出（DWB 指令，vx>=0）
    RViz 中 Global/Local Costmap 显示层随狗移动刷新障碍物
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('nav')

    nav_params = os.path.join(nav_pkg, 'config', 'costmap.yaml')
    rviz_config = os.path.join(nav_pkg, 'rviz', 'nav.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')

    # 1. 定位栈（内部自带 RViz，这里关掉，由本文件统一启动 nav.rviz）
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'localization.launch.py')),
        launch_arguments={'use_rviz': 'false', 'map': map_yaml}.items())

    # 2. 控制服务器（承载局部代价地图 + DWB 控制器）
    #    DWB 直接输出 /cmd_vel 供 cmd_vel_sport_bridge 订阅
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params],
    )

    # 3. 规划服务器（承载全局代价地图 + NavFn 规划器）
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params],
    )

    # 4. lifecycle 管理器：自动激活 controller/planner/behavior/bt_navigator
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav_params],
    )

    # 5. 恢复行为服务器（仅 spin/wait，禁 backup 倒车——vx>=0 硬约束）
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_params],
    )

    # 6. 行为树导航器：接收 /goal_pose（RViz 2D Goal Pose），调度
    #    ComputePathToPose -> FollowPath -> recovery，是自动导航的入口
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params],
    )

    # 7. cmd_vel -> 1008 MOVE 桥接（DWB 直接输出 /cmd_vel，Go2 只认 1008）
    #    含 vx>=0 硬约束 + 限幅 + 0.5s 断流看门狗停车
    cmd_vel_bridge = Node(
        package='nav',
        executable='cmd_vel_sport_bridge',
        name='cmd_vel_sport_bridge',
        output='screen',
    )

    # 9. 导航 RViz（含全局/局部代价地图、/plan 路径显示、2D Goal Pose 工具）
    #    注：rviz 节点不放 LaunchDescription 列表——本机上 IfCondition 会把
    #    它静默跳过（localization.launch.py 的 IfCondition 却正常，原因未明）。
    #    改用 OpaqueFunction 在 Python 层显式判断，见 _launch_setup。
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    def _launch_setup(context):
        # Python 层直接求值 use_rviz，绕开 IfCondition 静默跳过的问题
        use_rviz_val = LaunchConfiguration('use_rviz').perform(context).lower()
        entities = [
            localization,
            controller_server,
            planner_server,
            lifecycle_manager,
            behavior_server,
            bt_navigator,
            cmd_vel_bridge,
        ]
        if use_rviz_val in ('true', '1'):
            entities.append(rviz)
        else:
            print('[navigation.launch] use_rviz=false, 不启动 RViz')
        return entities

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动导航 RViz'),
        DeclareLaunchArgument(
            'map', default_value='/home/soyo/maps/lab_map.yaml',
            description='定位用栅格地图 yaml 路径'),
        OpaqueFunction(function=_launch_setup),
    ])
