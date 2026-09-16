"""Nav2 for Raupy: nav2_bringup navigation_launch.py plus a standalone map_saver.

No AMCL and no map_server. slam_toolbox provides /map and map->odom.
The final velocity output is /cmd_vel as geometry_msgs/TwistStamped
(controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> cmd_vel). See config/nav2_raupy.yaml.

The map_saver is separate because navigation_launch.py doesn't start one. The exploration
supervisor calls /map_saver/save_map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    return LaunchDescription(declare_args + [nav2, map_saver, lifecycle_manager_map_saver])
