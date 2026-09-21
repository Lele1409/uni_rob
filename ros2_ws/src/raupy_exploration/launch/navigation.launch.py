"""Nav2 for Raupy: nav2_bringup navigation_launch.py plus a standalone map_saver.

No AMCL and no map_server. slam_toolbox provides /map and map->odom.
The final velocity output is /cmd_vel as geometry_msgs/TwistStamped
(controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> cmd_vel). See config/nav2_raupy.yaml.

The map_saver is separate because navigation_launch.py doesn't start one. The exploration
supervisor calls /map_saver/save_map.

range_relay feeds the robot's low ToF sensors (/range/*) into the costmaps' range_layer and
the collision monitor, for obstacles below the lidar plane.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the Nav2 + map_saver launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # FindPackageShare resolves when the launch runs, not at import or description time.
    # Importing this file doesn't need either package to be installed.
    default_params = PathJoinSubstitution(
        [FindPackageShare('raupy_exploration'), 'config', 'nav2_raupy.yaml'])
    nav2_navigation_launch = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])

    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (simulation) if true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameter file (also used for map_saver)'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically configure and activate the lifecycle nodes'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS log level for all Nav2 nodes'),
        DeclareLaunchArgument(
            'use_range_sensors', default_value='true',
            description='Relay the low ToF sensors /range/* into the costmaps and collision monitor'),
        DeclareLaunchArgument(
            'range_fov_scale', default_value='0.5',
            description='Scale the ToF cone (driver: 15 deg) marked in the costmap; 0.5 = 7.5 deg.'),
        DeclareLaunchArgument(
            'range_cutoff', default_value='0.0',
            description='Ignore ToF hits farther than this in m (0 = sensor max, 0.9 m). '
                        'Lower it if the sensors see the floor.'),
    ]

    # Non-composed bringup: one process per server, easier to debug and to read logs.
    # navigation_launch.py applies use_sim_time via SetParameter and rewrites autostart.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': log_level,
        }.items(),
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        # The params file has a map_saver section. use_sim_time comes last so it wins.
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_map_saver = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_saver'],
        }],
    )

    range_relay = Node(
        package='raupy_exploration',
        executable='range_relay.py',
        name='range_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_range_cutoff': ParameterValue(LaunchConfiguration('range_cutoff'), value_type=float),
            'fov_scale': ParameterValue(LaunchConfiguration('range_fov_scale'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('use_range_sensors')),
    )

    return LaunchDescription(
        declare_args + [nav2, map_saver, lifecycle_manager_map_saver, range_relay])
