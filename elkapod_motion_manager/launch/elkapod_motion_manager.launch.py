from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation time'
    )

    elkapod_motion_manager_dir = get_package_share_directory('elkapod_motion_manager')
    config_path = os.path.join(elkapod_motion_manager_dir, 'config', 'elkapod_motion_manager.yaml')

    imu_republisher_sim = Node(
        package="elkapod_odometry",
        executable="imu_republisher",
        parameters=[{'use_sim_time': LaunchConfiguration("sim_mode")}],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("sim_mode"))
    )

    imu_republisher_hardware = Node(
        package="topic_tools",
        executable="relay",
        name="imu_republisher",
        arguments=["/imu_broadcaster/imu", "/imu"],
        remappings=[
            ('input', '/imu_broadcaster/imu'),
            ('output', '/imu')
        ],
        condition=UnlessCondition(LaunchConfiguration("sim_mode"))
    )

    gait_node = Node(
        package="elkapod_gait_gen_cpp",
        executable="elkapod_gait",
        parameters=[config_path, {'use_sim_time': LaunchConfiguration("sim_mode")}],
        output='screen',
        emulate_tty=True
    )

    motion_manager = Node(
        package="elkapod_motion_manager",
        executable="elkapod_motion_manager",
        parameters=[config_path, {'use_sim_time': LaunchConfiguration("sim_mode")}],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        use_sim_time_arg,
        imu_republisher_sim,
        imu_republisher_hardware,
        gait_node,
        motion_manager
    ])
