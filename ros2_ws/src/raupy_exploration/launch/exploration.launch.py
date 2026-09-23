"""One-command laptop bringup for autonomous exploration on Bisasam.

    slam.launch.py        scan filter (/scan -> /scan_clean) + slam_toolbox (/map, TF map->odom)
    navigation.launch.py  Nav2 (navigate_to_pose, costmaps) + map_saver
    frontier_explorer     library node, started after explorer_start_delay_s
    exploration_supervisor  stops exploration (complete / timeout / stagnation) and saves the map
    stuck_monitor         marks spots where the robot got stuck on unseen obstacles
    rviz2                 optional

The robot only runs its preinstalled stack (driver, EKF, lidar); everything above runs here.
Bisasam runs that stack as Humble containers with CycloneDDS, so the laptop must use
rmw_cyclonedds_cpp as well (scripts/raupy_env.sh sets it). Its ToF sensors are not
published, hence use_range_sensors defaults to false here.

On the real robot, use_bridge:=true robot_domain:=<id> starts domain_bridge (config/domain_bridge.yaml)
and the stack itself must run on a separate, laptop-only ROS_DOMAIN_ID: then only the bridge
talks to the robot over Wi-Fi. That matters twice on Bisasam, whose domain 30 is shared with
another group. scripts/raupy_explore.sh sets this up.
"""

import os


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = 'raupy_exploration'


def _share(*parts):
    # FindPackageShare is resolved at launch time, so importing this file works
    # before the workspace is built.
    return PathJoinSubstitution([FindPackageShare(PKG), *parts])


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _float_arg(context, name: str) -> float:
    raw = LaunchConfiguration(name).perform(context)
    try:
        return float(raw)
    except ValueError as exc:
        raise RuntimeError(f"Launch argument '{name}' must be a number, got '{raw}'") from exc


def _launch_supervisor(context, *args, **kwargs):
    # Built in an OpaqueFunction so the values get real Python types. A plain
    # substitution would hand '600' to the node as an int (or '' as a string),
    # which fails against the supervisor's declared double/string parameters.
    overrides = {
        'use_sim_time': _as_bool(LaunchConfiguration('use_sim_time').perform(context)),
        'max_duration_s': _float_arg(context, 'max_duration_s'),
        'stagnation_window_s': _float_arg(context, 'stagnation_window_s'),
        'min_new_area_m2': _float_arg(context, 'min_new_area_m2'),
        # value_type=str: launch_ros would otherwise YAML-parse e.g. '2026' into an int.
        'map_name': ParameterValue(LaunchConfiguration('map_name').perform(context), value_type=str),
    }
    # Empty map_output_dir means "use the supervisor's default", so don't override it.
    map_output_dir = LaunchConfiguration('map_output_dir').perform(context).strip()
    if map_output_dir:
        overrides['map_output_dir'] = ParameterValue(map_output_dir, value_type=str)

    return [
        Node(
            package=PKG,
            executable='exploration_supervisor.py',
            name='exploration_supervisor',
            output='screen',
            parameters=[LaunchConfiguration('supervisor_params_file'), overrides],
        )
    ]


def _launch_bridge(context, *args, **kwargs):
    if not _as_bool(LaunchConfiguration('use_bridge').perform(context)):
        return []
    robot_domain = LaunchConfiguration('robot_domain').perform(context).strip()
    stack_domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not robot_domain.isdigit():
        raise RuntimeError("use_bridge:=true needs robot_domain:=<robot's ROS_DOMAIN_ID>")
    if robot_domain == stack_domain:
        raise RuntimeError(
            f'The stack runs on ROS_DOMAIN_ID={stack_domain}, the same as the robot. With the '
            'bridge it must use its own domain (see scripts/raupy_explore.sh).')
    return [
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='raupy_bridge',
            output='screen',
            arguments=[
                LaunchConfiguration('bridge_config').perform(context),
                '--from', robot_domain, '--to', stack_domain],
            # The stack may be restricted to localhost; the bridge must reach the robot.
            additional_env={'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'},
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (simulation). False on the real robot.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Raw LaserScan topic from the robot.'),
        DeclareLaunchArgument(
            'laser_yaw_fix', default_value='',
            description="Override the robot URDF's lidar yaw in rad. Raupy needed 3.14159; on "
                        "Bisasam the default is empty (trust its URDF) until measured."),
        DeclareLaunchArgument(
            'use_scan_filter', default_value='true',
            description='Filter chassis returns (scan_topic -> /scan_clean) before SLAM.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with rviz/exploration.rviz.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=_share('rviz', 'exploration.rviz'),
            description='RViz layout (raupy_explore.sh passes a copy with a lower frame rate).'),
        DeclareLaunchArgument(
            'explorer_params_file', default_value=_share('config', 'explorer_raupy.yaml'),
            description='frontier_explorer parameter file.'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=_share('config', 'nav2_raupy.yaml'),
            description='Nav2 parameter file.'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=_share('config', 'slam_raupy.yaml'),
            description='slam_toolbox parameter file.'),
        DeclareLaunchArgument(
            'supervisor_params_file', default_value=_share('config', 'supervisor.yaml'),
            description='exploration_supervisor parameter file (launch args below override it).'),
        DeclareLaunchArgument(
            'max_duration_s', default_value='600.0',
            description='Supervisor: hard time limit for the exploration run.'),
        DeclareLaunchArgument(
            'stagnation_window_s', default_value='90.0',
            description='Supervisor: window in which the map must grow by min_new_area_m2.'),
        DeclareLaunchArgument(
            'min_new_area_m2', default_value='0.5',
            description='Supervisor: minimum new known area per stagnation window.'),
        DeclareLaunchArgument(
            'map_name', default_value='bisasam_map',
            description='Supervisor: base file name of the saved map.'),
        DeclareLaunchArgument(
            'map_output_dir', default_value='',
            description='Supervisor: output directory for the map (empty = supervisor default).'),
        DeclareLaunchArgument(
            'use_range_sensors', default_value='false',
            description='Use the low ToF sensors (/range/*) for obstacles below the lidar. '
                        'False on Bisasam: its stack does not publish them.'),
        DeclareLaunchArgument(
            'range_fov_scale', default_value='0.5',
            description='Scale the ToF cone (driver: 15 deg) marked in the costmap; 0.5 = 7.5 deg.'),
        DeclareLaunchArgument(
            'range_cutoff', default_value='0.0',
            description='Ignore ToF hits farther than this in m (0 = sensor max).'),
        DeclareLaunchArgument(
            'use_bridge', default_value='false',
            description='Start domain_bridge between the robot domain and this (stack) domain.'),
        DeclareLaunchArgument(
            'robot_domain', default_value='',
            description="Robot's ROS_DOMAIN_ID, for use_bridge:=true."),
        DeclareLaunchArgument(
            'bridge_config', default_value=_share('config', 'domain_bridge.yaml'),
            description='domain_bridge topic list.'),
        DeclareLaunchArgument(
            'use_stuck_monitor', default_value='true',
            description='Detect getting stuck (driving but scan unchanged) and mark it for Nav2.'),
        DeclareLaunchArgument(
            'stuck_monitor_params_file', default_value=_share('config', 'stuck_monitor.yaml'),
            description='stuck_monitor parameter file.'),
        DeclareLaunchArgument(
            'explorer_start_delay_s', default_value='15.0',
            description='Delay before starting frontier_explorer so /map, TF and Nav2 are up.'),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share('launch', 'slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': LaunchConfiguration('scan_topic'),
            'laser_yaw_fix': LaunchConfiguration('laser_yaw_fix'),
            'use_scan_filter': LaunchConfiguration('use_scan_filter'),
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share('launch', 'navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': 'true',
            'use_range_sensors': LaunchConfiguration('use_range_sensors'),
            'range_cutoff': LaunchConfiguration('range_cutoff'),
            'range_fov_scale': LaunchConfiguration('range_fov_scale'),
        }.items(),
    )

    # Started late: if the explorer comes up before Nav2's action server and the
    # costmaps exist, its first frontier goals fail and may get suppressed.
    explorer = TimerAction(
        period=LaunchConfiguration('explorer_start_delay_s'),
        actions=[
            Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[
                    LaunchConfiguration('explorer_params_file'),
                    {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                ],
            )
        ],
    )

    stuck_monitor = Node(
        package=PKG,
        executable='stuck_monitor.py',
        name='stuck_monitor',
        output='screen',
        parameters=[
            LaunchConfiguration('stuck_monitor_params_file'),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(LaunchConfiguration('use_stuck_monitor')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declare_args + [
        OpaqueFunction(function=_launch_bridge),
        slam,
        navigation,
        explorer,
        stuck_monitor,
        OpaqueFunction(function=_launch_supervisor),
        rviz,
    ])
