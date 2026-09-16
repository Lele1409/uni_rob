"""Sensor pipeline + SLAM for Raupy (runs on the laptop).

    /scan (robot, raw) --[laser_filters box filter]--> /scan_clean --[slam_toolbox]--> /map, TF map->odom

The robot itself only provides /scan and TF odom->base_link; everything here runs locally.
This file is included by exploration.launch.py, so the argument names are part of its interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Single scan topic consumed by slam_toolbox AND the Nav2 costmaps / collision monitor.
# It always exists: produced by the box filter, or by a plain relay when the filter is off.
CLEAN_SCAN_TOPIC = '/scan_clean'


def generate_launch_description():
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare('raupy_exploration'), 'config', 'slam_raupy.yaml'])
    default_filter_params = PathJoinSubstitution(
        [FindPackageShare('raupy_exploration'), 'config', 'scan_filter.yaml'])

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (simulation). False on the real robot.'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Raw LaserScan input topic from the robot.'),
        DeclareLaunchArgument(
            'use_scan_filter', default_value='true',
            description='Run the chassis box filter (scan_topic -> /scan_clean). '
                        'If false, scan_topic is relayed unchanged to /scan_clean '
                        '(e.g. simulation without chassis returns).'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='slam_toolbox parameter file.'),
        DeclareLaunchArgument(
            'filter_params_file', default_value=default_filter_params,
            description='laser_filters scan_to_scan_filter_chain parameter file.'),
    ]

    # Node name must match the top-level key in scan_filter.yaml, otherwise the
    # filter chain gets no parameters (the bug on the robot's own filter node).
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('filter_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('scan_filtered', CLEAN_SCAN_TOPIC),
        ],
        condition=IfCondition(LaunchConfiguration('use_scan_filter')),
    )

    # Filter disabled: keep the /scan_clean contract so Nav2 still gets a scan.
    scan_relay = Node(
        package='raupy_exploration',
        executable='scan_relay.py',
        name='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('scan_in', LaunchConfiguration('scan_topic')),
            ('scan_out', CLEAN_SCAN_TOPIC),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_scan_filter')),
    )

    # slam_toolbox via its own launch file: autostart performs the lifecycle
    # configure -> activate transitions (a bare node would sit unconfigured).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
        }.items(),
    )

    return LaunchDescription(declare_args + [scan_filter, scan_relay, slam])
