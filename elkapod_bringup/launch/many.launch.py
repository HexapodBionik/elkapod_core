from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    rviz_only = LaunchConfiguration('rviz_only')

    elkapod_core_launch_path = os.path.join(
        get_package_share_directory('elkapod_bringup'),
        'launch',
        'simulation.launch.py'
    )

    elkapod_visualization_launch_path = os.path.join(
        get_package_share_directory('elkapod_visualization'),
        'launch',
        'display.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_only', default_value='true',
            description='If true, starts gazebo in headless mode, also doesn\'t opens rtabmap viewer, overwrites headless for gazebo launch'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(elkapod_core_launch_path),
            launch_arguments={'headless': rviz_only}.items()
        ),

        TimerAction(period=20.0,
                    actions=[IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            elkapod_visualization_launch_path),
                    )]),

        Node(
            package="elkapod_controller_gui",
            executable="elkapod_controller_gui",
            name='elkapod_gui',
            output='screen',
            emulate_tty=True),
    ])