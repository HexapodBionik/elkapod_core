import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    elkapod_rsp_launch_path = os.path.join(
            get_package_share_directory('elkapod_bringup'),
            'launch',
            'rsp.launch.py'
    )
    elkapod_rl_policy_launch_path = os.path.join(
            get_package_share_directory('elkapod_rl_deploy'),
            'launch',
            'hexapod_policy.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(elkapod_rsp_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(elkapod_rl_policy_launch_path)),
    ])
