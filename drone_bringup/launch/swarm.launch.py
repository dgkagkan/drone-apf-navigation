"""Start the swarm target buffer, assignment coordinator, and operator terminal.

Individual drone stacks publish their configured VehicleState and run one local
route executor. Every executor receives the common swarm mission broadcast,
keeps only its own route, and feeds the existing namespaced NavigateTo action.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    operator_terminal = LaunchConfiguration("operator_terminal")
    use_dashboard = LaunchConfiguration("use_dashboard")
    open_dashboard = LaunchConfiguration("open_dashboard")
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")
    default_config = PathJoinSubstitution([
        FindPackageShare("drone_swarm"), "config", "swarm.yaml"
    ])
    operator_command = [
        TextSubstitution(
            text='ros2 run drone_swarm swarm_operator_node --ros-args --params-file "'
        ),
        config_file,
        TextSubstitution(text='"; exec bash'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("operator_terminal", default_value="true"),
        DeclareLaunchArgument("use_dashboard", default_value="true"),
        DeclareLaunchArgument("open_dashboard", default_value="true"),
        DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("dashboard_port", default_value="8765"),
        Node(
            package="drone_swarm",
            executable="swarm_coordinator_node",
            name="swarm_coordinator",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="drone_swarm",
            executable="swarm_visualization_node",
            name="swarm_visualization",
            output="screen",
        ),
        Node(
            package="drone_dashboard",
            executable="dashboard_node",
            name="swarm_dashboard",
            output="screen",
            parameters=[{
                "host": dashboard_host,
                "port": ParameterValue(dashboard_port, value_type=int),
            }],
            condition=IfCondition(use_dashboard),
        ),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=["xdg-open", ["http://", dashboard_host, ":", dashboard_port]],
                output="log",
                name="open_swarm_dashboard",
                condition=IfCondition(PythonExpression([
                    "'", use_dashboard, "' == 'true' and '", open_dashboard, "' == 'true'"
                ])),
            )],
        ),
        ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--wait",
                "--title=Swarm Operator",
                "--",
                "bash",
                "-lc",
                operator_command,
            ],
            output="screen",
            name="swarm_operator_terminal",
            condition=IfCondition(operator_terminal),
        ),
    ])
