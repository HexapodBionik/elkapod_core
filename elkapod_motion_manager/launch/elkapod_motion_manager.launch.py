from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    elkapod_motion_manager_dir = get_package_share_directory('elkapod_motion_manager')
    config_path = os.path.join(elkapod_motion_manager_dir, 'config', 'elkapod_motion_manager.yaml')

    elkapod_description_dir = get_package_share_directory('elkapod_description')
    leg_config = os.path.join(elkapod_description_dir, 'config', 'leg_configuration.yaml')



    control_node = Node(
        package="elkapod_leg_control_tests",
        executable="elkapod_control",
        parameters=[{
            "config_path": leg_config
        }],
    )

    gait_node = Node(
        package="elkapod_gait_gen_cpp",
        executable="elkapod_gait",
        parameters=[config_path]
    )

    motion_manager = Node(
            package="elkapod_motion_manager",
            executable="elkapod_motion_manager",
            parameters=[config_path]
    )

    return LaunchDescription([
        control_node,
        gait_node,
        motion_manager

    ])
