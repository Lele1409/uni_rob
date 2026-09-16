"""Start only the frontier_explorer library node with Raupy's parameters.

For debugging: run SLAM and Nav2 separately (slam.launch.py, navigation.launch.py),
then start/restart the explorer on its own with this file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Resolved at launch time, so importing this file works before the workspace is built.
    default_params = PathJoinSubstitution(
        [FindPackageShare('raupy_exploration'), 'config', 'explorer_raupy.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock (simulation). False on the real robot.'),
        DeclareLaunchArgument(
            'explorer_params_file', default_value=default_params,
            description='frontier_explorer parameter file.'),
        Node(
            package='frontier_exploration_ros2',
            executable='frontier_explorer',
            name='frontier_explorer',  # must match the root key in the params file
            output='screen',
            parameters=[
                LaunchConfiguration('explorer_params_file'),
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])
