import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("sim_mode", default_value="false", description="Use simulation time")

    elkapod_motion_manager_dir = get_package_share_directory("elkapod_motion_manager")
    config_path = os.path.join(elkapod_motion_manager_dir, "config", "elkapod_motion_manager.yaml")
    twist_mux_config_path = os.path.join(elkapod_motion_manager_dir, "config", "twist_mux_config.yaml")

    imu_republisher_sim = Node(
        package="elkapod_odometry",
        executable="imu_republisher",
        parameters=[{"use_sim_time": LaunchConfiguration("sim_mode")}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("sim_mode")),
    )

    imu_republisher_hardware = Node(
        package="topic_tools",
        executable="relay",
        name="imu_republisher",
        arguments=["/imu_broadcaster/imu", "/imu"],
        remappings=[("input", "/imu_broadcaster/imu"), ("output", "/imu")],
        condition=UnlessCondition(LaunchConfiguration("sim_mode")),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim_mode")},
            twist_mux_config_path,
        ],
    )

    motion_manager = Node(
        package="elkapod_motion_manager",
        executable="elkapod_motion_manager",
        parameters=[config_path, {"use_sim_time": LaunchConfiguration("sim_mode")}],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            imu_republisher_sim,
            imu_republisher_hardware,
            motion_manager,
            twist_mux_node,
        ]
    )
