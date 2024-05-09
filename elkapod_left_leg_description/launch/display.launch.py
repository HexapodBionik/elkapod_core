import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    # Find the package
    urdf_launch_package = FindPackageShare(package="elkapod_left_leg_description").find(
        "elkapod_left_leg_description"
    )

    # Get the paths for URDF model and RViz configuration
    my_model_path = os.path.join(urdf_launch_package, "urdf/left_leg.urdf")

    # default_rviz_config_path = os.path.join(
    #     urdf_launch_package, "urdf/manipulator.rviz"
    # )
    robot_description_content = ParameterValue(Command(['xacro ', my_model_path]), value_type=str)

    default_rviz_config_path = os.path.join(
        urdf_launch_package, "config/left_leg.rviz"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_content,
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'])
    )

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"rate": ParameterValue(30)}],
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path],
        )
    )
    ld.add_action(robot_state_publisher_node)

    return ld
