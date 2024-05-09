from launch import LaunchDescription
from launch.substitutions import  LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition


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
                {"config_path": "elkapod_motion/config/leg_configuration.yaml"}
            ]
        )
    )

    ld.add_action(
        Node(
            package='elkapod_visualization_translation',
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
