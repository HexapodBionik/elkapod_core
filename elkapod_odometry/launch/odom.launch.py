from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import TimerAction
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("sim_mode")

    elkapod_odometry_dir = get_package_share_directory("elkapod_odometry")
    ekf_config = os.path.join(elkapod_odometry_dir, "config", "ekf_config.yaml")
    odom_config = os.path.join(
        elkapod_odometry_dir, "config", "elkapod_odometry_params.yaml"
    )
    fsr_publisher_sim = os.path.join(
        elkapod_odometry_dir,
        "config",
        "elkapod_binary_fsr_publisher_hardware_params.yaml",
    )
    fsr_publisher_hardware = os.path.join(
        elkapod_odometry_dir, "config", "elkapod_binary_fsr_publisher_sim_params.yaml"
    )

    relay_node = Node(
        package="elkapod_odometry",
        executable="elkapod_relay",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_sim_time),
    )

    binary_fsr_publisher_node = Node(
        package="elkapod_odometry",
        executable="elkapod_binary_fsr_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            PythonExpression(
                [
                    "'",
                    fsr_publisher_sim,
                    "' if ",
                    use_sim_time,
                    " == 'true' else '",
                    fsr_publisher_hardware,
                    "'",
                ]
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    odom_node = Node(
        package="elkapod_odometry",
        executable="elkapod_odom",
        parameters=[odom_config, {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    delayed_actions = TimerAction(
        period=5.0,
        actions=[odom_node, ekf_node],
    )

    return LaunchDescription([relay_node, binary_fsr_publisher_node, delayed_actions])
