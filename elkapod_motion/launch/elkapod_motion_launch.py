import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals


def generate_launch_description():
    translation_node_name = 'kinematics_translation'

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(name=translation_node_name, default_value='true', choices=['true', 'false'])
    )

    ld.add_action(
        Node(
            package='elkapod_motion',
            executable='elkapod_kinematics',
            parameters=[
                {"soft_angle1": 2.0},
                {"soft_angle2": 1.0},
                {"soft_angle3": 3.0}
            ]
        )
    )

    ld.add_action(
        Node(
            package='elkapod_motion',
            executable='elkapod_translation',
            condition=IfCondition(LaunchConfiguration(translation_node_name)),
            parameters=[
                {"hard_angle1": 4.0},
                {"hard_angle2": 2.0},
                {"hard_angle3": 0.0}
            ]
        )
    )

    return ld