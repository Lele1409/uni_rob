from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('epsilon',        default_value='0.98'),
        DeclareLaunchArgument('epsilon_k',      default_value='0.98'),
        DeclareLaunchArgument('min_points',     default_value='3'),
        DeclareLaunchArgument('max_point_dist', default_value='2.0'),
        DeclareLaunchArgument('scan_topic',     default_value='/scanout/scan'),

        Node(
            package='online_line_finder',
            executable='online_line_finder',
            name='online_line_finder',
            parameters=[{
                'epsilon':        LaunchConfiguration('epsilon'),
                'epsilon_k':      LaunchConfiguration('epsilon_k'),
                'min_points':     LaunchConfiguration('min_points'),
                'max_point_dist': LaunchConfiguration('max_point_dist'),
                'scan_topic':     LaunchConfiguration('scan_topic'),
            }],
        ),
    ])
