"""One-command laptop bringup for autonomous exploration on Raupy.

    slam.launch.py        scan filter (/scan -> /scan_clean) + slam_toolbox (/map, TF map->odom)
    navigation.launch.py  Nav2 (navigate_to_pose, costmaps) + map_saver
    frontier_explorer     library node, started after explorer_start_delay_s
    exploration_supervisor  stops exploration (complete / timeout / stagnation) and saves the map
    rviz2                 optional

The robot only runs its preinstalled services (driver, EKF, lidar); everything above runs here.
"""

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
            'use_scan_filter', default_value='true',
            description='Filter chassis returns (scan_topic -> /scan_clean) before SLAM.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with rviz/exploration.rviz.'),
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
            'map_name', default_value='raupy_map',
            description='Supervisor: base file name of the saved map.'),
        DeclareLaunchArgument(
            'map_output_dir', default_value='',
            description='Supervisor: output directory for the map (empty = supervisor default).'),
        DeclareLaunchArgument(
            'explorer_start_delay_s', default_value='15.0',
            description='Delay before starting frontier_explorer so /map, TF and Nav2 are up.'),
    ]

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share('launch', 'slam.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'scan_topic': LaunchConfiguration('scan_topic'),
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', _share('rviz', 'exploration.rviz')],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declare_args + [
        slam,
        navigation,
        explorer,
        OpaqueFunction(function=_launch_supervisor),
        rviz,
    ])
