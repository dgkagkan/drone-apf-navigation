"""Start the swarm target buffer, assignment coordinator, and operator terminal.

Individual drone stacks publish their configured VehicleState and run one local
route executor. Every executor receives the common swarm mission broadcast,
keeps only its own route, and feeds the existing namespaced NavigateTo action.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    operator_terminal = LaunchConfiguration("operator_terminal")
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
        Node(
            package="drone_swarm",
            executable="swarm_operator_node",
            name="swarm_operator",
            output="screen",
            parameters=[config_file],
            condition=UnlessCondition(operator_terminal),
        ),
    ])
