import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    sim_mode = LaunchConfiguration('sim_mode')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('elkapod_description'))
    xacro_file = os.path.join(pkg_path, 'urdf','elkapod.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', sim_mode])

    params = {'robot_description': robot_description_config, 'sim_mode': sim_mode}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher
    ])
