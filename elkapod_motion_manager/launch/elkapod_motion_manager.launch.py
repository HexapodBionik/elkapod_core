from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('sim_mode')

    elkapod_motion_manager_dir = get_package_share_directory('elkapod_motion_manager')
    config_path = os.path.join(elkapod_motion_manager_dir, 'config', 'elkapod_motion_manager.yaml')

    imu_republisher = Node(
        package="elkapod_odometry",
        executable="imu_republisher",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_sim_time)
    )

    gait_node = Node(
        package="elkapod_gait_gen_cpp",
        executable="elkapod_gait",
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )

    motion_manager = Node(
        package="elkapod_motion_manager",
        executable="elkapod_motion_manager",
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        imu_republisher,
        gait_node,
        motion_manager

    ])
