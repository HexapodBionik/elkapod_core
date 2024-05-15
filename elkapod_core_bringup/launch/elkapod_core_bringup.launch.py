import os 
from launch import LaunchDescription 
from launch.substitutions import Command, LaunchConfiguration 
from launch.actions import DeclareLaunchArgument 
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue 
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import yaml


def generate_launch_description():
    
    RorS_launch_arg = DeclareLaunchArgument(
        "simulation", default_value=TextSubstitution(text="0")
    )
    # RorS_launch_arg = str(RorS_launch_arg)
    
    # Find the packages
    bringup_package = FindPackageShare(package="elkapod_core_bringup").find(
        "elkapod_core_bringup"
    )
    motion_package = FindPackageShare(package="elkapod_motion").find(
        "elkapod_motion"
    )
    trajectory_package = FindPackageShare(package="elkapod_trajectory_generator").find(
        "elkapod_trajectory_generator"
    )
    comm_package = FindPackageShare(package="elkapod_comm").find(
        "elkapod_comm"
    )
    simulation_package = FindPackageShare(package="elkapod_sim").find("elkapod_sim")


    # elkapod_kinematics_node = Node(
    #     package="elkapod_motion",
    #     executable="elkapod_kinematics",
    # )
    
    elkapod_kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_package, "launch/elkapod_motion_launch.py")
        )
    )
    
    elkapod_trajectory_gen_node = Node(
        package="elkapod_trajectory_generator",
        executable="elkapod_trajectory_generator",
    )
    
    if (RorS_launch_arg=="0"):
        elkapod_comm_node = Node(
            package="elkapod_comm",
            executable="elkapod_comm_server"
        )
    else:
        elkapod_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_package, "launch/robot_launch.py")
        )
    )

    # Create the launch description and populate
    
    # ld = LaunchDescription(RorS_launch_arg)
    ld = LaunchDescription()

    ld.add_action(elkapod_kinematics_launch)

    ld.add_action(
        Node(
            package="elkapod_trajectory_generator",
            executable="elkapod_trajectory_generator"
        )
    )
    
    if (RorS_launch_arg=="0"):
        ld.add_action(
            Node(
                package="elkapod_comm",
                executable="elkapod_comm_server"
            )
        )
    else:
        ld.add_action(elkapod_simulation_launch)
    
    return ld
