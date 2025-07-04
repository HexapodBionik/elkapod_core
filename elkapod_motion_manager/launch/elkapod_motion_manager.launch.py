from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    elkapod_motion_manager_dir = get_package_share_directory('elkapod_motion_manager')
    config_path = os.path.join(elkapod_motion_manager_dir, 'config', 'elkapod_motion_manager.yaml')

    return LaunchDescription([
        Node(
            package="elkapod_motion_manager",
            executable="elkapod_motion_manager",
            parameters=[config_path]
        )
    ])
