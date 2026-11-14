import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VARIANTS = {
    "flat": ("elkapod_policy_node", "hexapod.yaml", "best_agent.pt"),
    "rough": ("elkapod_rough_policy_node", "hexapod_rough.yaml", "rough_agent.pt"),
}

def launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory("elkapod_rl_deploy"), "config")

    variant = LaunchConfiguration("variant").perform(context)
    if variant not in VARIANTS:
        raise RuntimeError(f"type must be one of {list(VARIANTS)}, got '{variant}'")
    name, default_config, default_checkpoint = VARIANTS[variant]

    config_file = LaunchConfiguration("config_file").perform(context) or os.path.join(
        config_dir, default_config
    )
    checkpoint_path = LaunchConfiguration("checkpoint_path").perform(
        context
    ) or os.path.join(config_dir, default_checkpoint)

    node = Node(
        package="elkapod_rl_deploy",
        executable="policy_node",
        name=name,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": config_file,
            "checkpoint_path": checkpoint_path,
            "use_sim_time": True,
        }],
    )
    return [node]


def generate_launch_description():
    variant_arg = DeclareLaunchArgument(
        "variant",
        default_value="flat",
        choices=list(VARIANTS),
        description=(
            "Policy variant: end2end policy without height map (flat), end2end policy with height map (rough)."
        ),
    )
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="",
        description="Absolute path to the skrl .pt checkpoint (empty = variant default)",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Path to YAML config (empty = variant default)",
    )
    return LaunchDescription([
        variant_arg,
        checkpoint_arg,
        config_arg,
        OpaqueFunction(function=launch_setup),
    ])
