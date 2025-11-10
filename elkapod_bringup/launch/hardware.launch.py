from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_description"),
            "config",
            "my_controllers.yaml",
        ]
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("elkapod_bringup"), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': 'false'}.items()
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        emulate_tty=True
    )

    elkapod_ik_controller_spawner = TimerAction(period=8.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["elkapod_ik_controller"],
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

    imu_broad_spawner = TimerAction(period=5.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"],
        output='screen',
        emulate_tty=True
    )])

    motion_manager_launch_path = os.path.join(
        get_package_share_directory('elkapod_motion_manager'),
        'launch',
        'elkapod_motion_manager.launch.py'
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path), launch_arguments={'use_sim_time': 'false'}.items()
    )
    delayed_nodes = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ]
    )


    return LaunchDescription([
        rsp,
        control_node,
        joint_broad_spawner,
        imu_broad_spawner,
        elkapod_ik_controller_spawner,
        delayed_nodes
    ])