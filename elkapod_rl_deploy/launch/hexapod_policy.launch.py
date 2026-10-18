import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# terrain -> (executable, node name, default config, default checkpoint)
POLICIES = {
    "flat": ("policy_node", "elkapod_policy_node", "hexapod.yaml", "best_agent.pt"),
    "rough": (
        "rough_policy_node",
        "elkapod_rough_policy_node",
        "hexapod_rough.yaml",
        "rough_agent.pt",
    ),
}


def launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory("elkapod_rl_deploy"), "config")

    terrain = LaunchConfiguration("terrain").perform(context)
    if terrain not in POLICIES:
        raise RuntimeError(f"terrain must be one of {list(POLICIES)}, got '{terrain}'")
    executable, name, default_config, default_checkpoint = POLICIES[terrain]

    config_file = LaunchConfiguration("config_file").perform(context) or os.path.join(
        config_dir, default_config
    )
    checkpoint_path = LaunchConfiguration("checkpoint_path").perform(
        context
    ) or os.path.join(config_dir, default_checkpoint)

    node = Node(
        package="elkapod_rl_deploy",
        executable=executable,
        name=name,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "config_file": config_file,
            "checkpoint_path": checkpoint_path,
            "control_rate_hz": float(LaunchConfiguration("control_rate_hz").perform(context)),
            "use_sim_time": True,
        }],
    )
    return [node]


def generate_launch_description():
    terrain_arg = DeclareLaunchArgument(
        "terrain",
        default_value="flat",
        choices=list(POLICIES),
        description=(
            "Policy variant: 'flat' (proprioceptive only) or 'rough' (height scan "
            "from elevation_mapping_cupy)."
        ),
    )
    checkpoint_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="",
        description="Absolute path to the skrl .pt checkpoint (empty = terrain default)",
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Path to YAML config (empty = terrain default)",
    )
    rate_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="50.0",
        description=(
            "Inference loop frequency (Hz). 0 = use the value from the YAML "
            "config (which must match 1/(sim.dt*decimation) from training)."
        ),
    )

    return LaunchDescription([
        terrain_arg,
        checkpoint_arg,
        config_arg,
        rate_arg,
        OpaqueFunction(function=launch_setup),
    ])
