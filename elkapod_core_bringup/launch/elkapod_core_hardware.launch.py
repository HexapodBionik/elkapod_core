from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def generate_launch_description():
    robot_description_package = 'elkapod_description'
    elkapod_core = "elkapod_core_bringup"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(elkapod_core), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    motion_manager_launch_path = os.path.join(
        get_package_share_directory('elkapod_motion_manager'),
        'launch',
        'elkapod_motion_manager.launch.py'
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path)
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("elkapod_description"), "urdf", "elkapod.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_description"),
            "config",
            "my_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        emulate_tty=True
    )

    joint_position_controller_spawner = TimerAction(period=5.0, actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_position_controller"],
            output='screen',
            emulate_tty=True
    )])

    joint_broad_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen',
        emulate_tty=True
    )])


    return LaunchDescription([
        rsp,
        control_node,
        joint_position_controller_spawner,
        joint_broad_spawner,
        delayed_actions
    ])