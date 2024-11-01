import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import yaml

def generate_launch_description():
    # Find the package
    urdf_launch_package = FindPackageShare(package="elkapod_description").find(
        "elkapod_description"
    )

    # Get the paths for URDF model and RViz configuration
    my_model_path = os.path.join(urdf_launch_package, "urdf/elkapod.urdf.xacro")
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
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'])
    )
    
    ld.add_action(robot_state_publisher_node)

    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"rate": ParameterValue(30)}],
            condition=IfCondition(LaunchConfiguration('gui'))
        )
    )

    # ld.add_action(
    #     Node(
    #         package='elkapod_visualization_translation',
    #         executable='elkapod_translation',
    #         condition=UnlessCondition(LaunchConfiguration('gui')),
    #         parameters=[
    #             {"hard_angle1": hardware_angle1},
    #             {"hard_angle2": hardware_angle2},
    #             {"hard_angle3": hardware_angle3}
    #         ]
    #     )
    # )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path],
        )
    )

    return ld

