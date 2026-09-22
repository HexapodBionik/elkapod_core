from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("elkapod_rl_deploy")

    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint_path",
        description="Absolute path to the skrl .pt checkpoint",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "hexapod.yaml"]),
        description="Path to YAML config",
    )
    rate_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="0.0",
        description=(
            "Inference loop frequency (Hz). 0 = use the value from the YAML "
            "config (which must match 1/(sim.dt*decimation) from training)."
        ),
    )

    node = Node(
        package="elkapod_rl_deploy",
        executable="policy_node",
        name="hexapod_policy_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "checkpoint_path": LaunchConfiguration("checkpoint_path"),
            "control_rate_hz": LaunchConfiguration("control_rate_hz"),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([checkpoint_arg, config_arg, rate_arg, node])
