from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("elkapod_bringup"), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': 'false'}.items()
    )

    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("elkapod_bringup"), 'launch', 'controllers.launch.py'
        )]), launch_arguments={'sim_mode': 'false'}.items()
    )

    motion_manager_launch_path = os.path.join(
        get_package_share_directory('elkapod_motion_manager'),
        'launch',
        'elkapod_motion_manager.launch.py'
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path), launch_arguments={'use_sim_time': 'false'}.items()
    )
    delayed_nodes = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ]
    )

    return LaunchDescription([
        rsp,
        ros2_control,
        delayed_nodes
    ])