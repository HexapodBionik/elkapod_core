from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_launch_path = os.path.join(
        get_package_share_directory('elkapod_gazebo'),
        'launch',
        'elkapod.gazebo.launch.py'
    )

    motion_manager_launch_path = os.path.join(
        get_package_share_directory('elkapod_motion_manager'),
        'launch',
        'elkapod_motion_manager.launch.py'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path), launch_arguments={'use_sim_time': 'true'}.items()
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        delayed_actions
    ])