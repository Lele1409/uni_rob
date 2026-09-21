"""Sensor pipeline + SLAM for Raupy (runs on the laptop).

    /scan (robot, raw) --[laser_filters box filter]--> /scan_clean --[slam_toolbox]--> /map, TF map->odom

The robot itself only provides /scan and TF odom->base_link; everything here runs locally.

laser_yaw_fix:=<rad> corrects a wrong lidar mounting in the robot's URDF without touching the
robot: /scan is relayed to /scan_fixed in frame laser_fixed, with our own static TF
base_link -> laser_fixed (laser_x, 0, laser_z, yaw). Raupy needs 3.14159 (URDF says -90 deg).
This file is included by exploration.launch.py, so the argument names are part of its interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Single scan topic consumed by slam_toolbox AND the Nav2 costmaps / collision monitor.
# It always exists: produced by the box filter, or by a plain relay when the filter is off.
CLEAN_SCAN_TOPIC = '/scan_clean'
FIXED_SCAN_TOPIC = '/scan_fixed'
FIXED_LASER_FRAME = 'laser_fixed'


def _scan_pipeline(context, *args, **kwargs):
    """Optional frame fix, then box filter (or plain relay) into /scan_clean."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_in = LaunchConfiguration('scan_topic').perform(context)
    yaw = LaunchConfiguration('laser_yaw_fix').perform(context).strip()
    nodes = []
    if yaw:
        float(yaw)  # fail early on typos
        nodes += [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='laser_fixed_tf',
                arguments=[
                    '--x', LaunchConfiguration('laser_x').perform(context), '--y', '0',
                    '--z', LaunchConfiguration('laser_z').perform(context), '--yaw', yaw,
                    '--frame-id', 'base_link', '--child-frame-id', FIXED_LASER_FRAME],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='raupy_exploration',
                executable='scan_relay.py',
                name='scan_frame_fix',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'frame_id': FIXED_LASER_FRAME}],
                remappings=[('scan_in', scan_in), ('scan_out', FIXED_SCAN_TOPIC)],
            ),
        ]
        scan_in = FIXED_SCAN_TOPIC

    if _is_true(LaunchConfiguration('use_scan_filter').perform(context)):
        # Node name must match the top-level key in scan_filter.yaml, otherwise the
        # filter chain gets no parameters (the bug on the robot's own filter node).
        nodes.append(Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            output='screen',
            parameters=[
                LaunchConfiguration('filter_params_file'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('scan', scan_in), ('scan_filtered', CLEAN_SCAN_TOPIC)],
        ))
    else:
        # Filter disabled: keep the /scan_clean contract so Nav2 still gets a scan.
        nodes.append(Node(
            package='raupy_exploration',
            executable='scan_relay.py',
            name='scan_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('scan_in', scan_in), ('scan_out', CLEAN_SCAN_TOPIC)],
        ))
    return nodes


def _is_true(value):
    return value.strip().lower() in ('true', '1', 'yes', 'on')


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
            'laser_yaw_fix', default_value='',
            description='If set (rad): override the robot URDF lidar yaw (see module docstring).'),
        DeclareLaunchArgument(
            'laser_x', default_value='0.02', description='Lidar x in base_link, for laser_yaw_fix.'),
        DeclareLaunchArgument(
            'laser_z', default_value='0.131', description='Lidar z in base_link, for laser_yaw_fix.'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='slam_toolbox parameter file.'),
        DeclareLaunchArgument(
            'filter_params_file', default_value=default_filter_params,
            description='laser_filters scan_to_scan_filter_chain parameter file.'),
    ]

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

    return LaunchDescription(declare_args + [OpaqueFunction(function=_scan_pipeline), slam])
