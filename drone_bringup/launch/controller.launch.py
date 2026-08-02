"""Start the modular manual/autonomous controller with shared APF safety."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_id = LaunchConfiguration("device_id")
    use_local_joy = LaunchConfiguration("use_local_joy")
    visualization_enabled = LaunchConfiguration("visualization_enabled")
    navigation_client_terminal = LaunchConfiguration("navigation_client_terminal")
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution([
        FindPackageShare("drone_control"), "config", "controller.yaml"
    ])
    shared_parameters = [config_file, {"use_sim_time": True}]
    navigation_client_command = [
        TextSubstitution(
            text='ros2 run drone_navigation navigation_client_node '
            '--ros-args --params-file "'
        ),
        config_file,
        TextSubstitution(text='"; exec bash'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument(
            "use_local_joy",
            default_value="true",
            description="false receives /joy from a remote ROS 2 computer.",
        ),
        DeclareLaunchArgument("visualization_enabled", default_value="true"),
        DeclareLaunchArgument("navigation_client_terminal", default_value="true"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="joy",
            executable="game_controller_node",
            name="ps4_game_controller",
            output="screen",
            condition=IfCondition(use_local_joy),
            parameters=[{
                "device_id": device_id,
                "deadzone": 0.08,
                "autorepeat_rate": 30.0,
                "coalesce_interval_ms": 1,
            }],
        ),
        Node(
            package="drone_control",
            executable="px4_gateway_node",
            name="px4_gateway",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_control",
            executable="flight_supervisor_node",
            name="flight_supervisor",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_control",
            executable="manual_control_node",
            name="manual_control",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_control",
            executable="gimbal_control_node",
            name="gimbal_control",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="lidar_processor_node",
            name="lidar_processor",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="command_mux_node",
            name="command_mux",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="apf_safety_node",
            name="apf_safety",
            output="screen",
            parameters=shared_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="navigation_server_node",
            name="navigation_server",
            output="screen",
            parameters=shared_parameters,
        ),
        ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--wait",
                "--title=Drone Navigation Client",
                "--",
                "bash",
                "-lc",
                navigation_client_command,
            ],
            output="screen",
            name="navigation_client_terminal",
            condition=IfCondition(navigation_client_terminal),
        ),
        Node(
            package="drone_navigation",
            executable="apf_visualizer_node",
            name="apf_visualizer",
            output="screen",
            condition=IfCondition(visualization_enabled),
            parameters=shared_parameters,
        ),
    ])
