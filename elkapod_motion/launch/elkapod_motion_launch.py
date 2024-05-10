from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ld = LaunchDescription()

    elkapod_motion_package_path = FindPackageShare(package="elkapod_motion").find("elkapod_motion")
    # Get the paths for URDF model and RViz configuration
    kin_params_path = os.path.join(elkapod_motion_package_path, "config/leg_configuration.yaml")

    ld.add_action(
        Node(
            package='elkapod_motion',
            executable='elkapod_kinematics',
            parameters=[
                {"config_path": kin_params_path}
            ]
        )
    )

    return ld
