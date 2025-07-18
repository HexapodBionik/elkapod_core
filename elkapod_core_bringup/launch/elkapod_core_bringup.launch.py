from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='True',
    )

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
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path)
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ]
    )

    return LaunchDescription([
        sim_arg,
        gazebo_launch,
        delayed_actions
    ])