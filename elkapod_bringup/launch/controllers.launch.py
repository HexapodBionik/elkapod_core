from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
def generate_launch_description():
    sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Launch hardware or simulation system'
    )
        
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
        parameters=[robot_controllers],
        output="both",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("sim_mode"))
    )

    controllers = TimerAction(period=5.0, actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["elkapod_ik_controller"],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_broadcaster"],
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("sim_mode"))
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output='screen',
            emulate_tty=True
        )
    ])

    controllers_second_wave = TimerAction(period=8.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["elkapod_gait_controller",  "--inactive"],
        output='screen',
        emulate_tty=True
    )])

    return LaunchDescription([
        sim_mode,
        control_node,
        controllers,
        controllers_second_wave
    ])