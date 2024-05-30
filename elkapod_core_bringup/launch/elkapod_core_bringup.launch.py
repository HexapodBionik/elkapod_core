from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument 
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(name="sim", default_value="true", choices=["true", "false"])
    )

    elkapod_sim_pkg_prefix = get_package_share_directory('elkapod_sim')
    elkapod_sim_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [elkapod_sim_pkg_prefix, '/launch/robot_launch.py']),
        launch_arguments={}.items(),
        condition=IfCondition(LaunchConfiguration("sim")),

    )
    ld.add_action(elkapod_sim_handler_launch)

    ld.add_action(
        Node(
            package="elkapod_comm",
            executable="elkapod_comm_server",
            condition=UnlessCondition(LaunchConfiguration("sim"))
        )
    )

    elkapod_motion_pkg_prefix = get_package_share_directory('elkapod_motion')
    elkapod_motion_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [elkapod_motion_pkg_prefix, '/launch/elkapod_motion_launch.py']),
        launch_arguments={}.items()
    )
    ld.add_action(elkapod_motion_handler_launch)

    ld.add_action(
        Node(
            package="elkapod_trajectory_generator",
            executable="elkapod_trajectory_generator",
        )
    )

    return ld
