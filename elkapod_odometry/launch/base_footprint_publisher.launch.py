import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
    )

    elkapod_odometry_dir = get_package_share_directory("elkapod_odometry")
    odom_config = os.path.join(elkapod_odometry_dir, "config", "elkapod_odometry_params.yaml")


    base_footprint_publisher = Node(
        package="elkapod_odometry",
        executable="elkapod_base_footprint_publisher",
        parameters=[odom_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [use_sim_time_arg, base_footprint_publisher]
    )
