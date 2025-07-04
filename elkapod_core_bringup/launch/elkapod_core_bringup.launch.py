from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='True',
    )

    elkapod_description_dir = get_package_share_directory('elkapod_description')
    config_path = os.path.join(elkapod_description_dir, 'config', 'leg_configuration.yaml')

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

    control_node = Node(
        package="elkapod_leg_control_tests",
        executable="elkapod_control",
        parameters=[{
            "config_path": config_path
        }],
        output="screen"
    )

    gait_node = Node(
        package="elkapod_gait_gen_py",
        executable="elkapod_gait",
        output="screen"
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
            control_node,
            gait_node
        ]
    )

    return LaunchDescription([
        sim_arg,
        gazebo_launch,
        delayed_actions
    ])