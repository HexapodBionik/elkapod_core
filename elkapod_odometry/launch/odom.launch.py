from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('sim_mode')

    elkapod_odometry_dir = get_package_share_directory('elkapod_odometry')
    ekf_config = os.path.join(elkapod_odometry_dir, 'config', 'ekf_config.yaml')

    relay_node = Node(
        package="elkapod_odometry",
        executable="elkapod_relay",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )

    odom_node = Node(
        package="elkapod_odometry",
        executable="elkapod_odom",
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True
    )


    return LaunchDescription([
        relay_node,
        odom_node,
        ekf_node
    ])
