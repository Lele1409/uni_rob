import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Declare launch configuratuin parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    num_wheels = LaunchConfiguration('num_wheels', default=4)
    wheel_radius = LaunchConfiguration('wheel_radius', default=0.0985)
    publish_tf = LaunchConfiguration('publish_tf', default='false')
    #tf_prefix = LaunchConfiguration("tf_prefix", '')

    # Get Volksbot URDF / xacro file and parse into valid URDF 
    # description
    urdf_file_name = 'urdf/volksbot.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('fake_encoder'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')

    # Generate lauch description consisting of launch time 
    # arguments and node configurations. Here we launch a 
    # robot state publisher and volksbot instance
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='fake_encoder',
            executable='fake_encoder',
            name='fake_encoder',
            parameters=[{'num_wheels': num_wheels, 'wheel_radius': wheel_radius,'robot_description': robot_desc}],
            output='screen'),
    ])

