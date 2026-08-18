"""Run the Optuna course for manual PS4 flight with RViz OctoMap mapping."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    px4_terminal = LaunchConfiguration("px4_terminal")
    follow = LaunchConfiguration("follow")
    device_id = LaunchConfiguration("device_id")
    use_local_joy = LaunchConfiguration("use_local_joy")
    navigation_client_terminal = LaunchConfiguration("navigation_client_terminal")
    controller_delay = LaunchConfiguration("controller_delay")
    mapping_delay = LaunchConfiguration("mapping_delay")
    mapping_rate = LaunchConfiguration("mapping_rate")
    mapping_resolution = LaunchConfiguration("mapping_resolution")
    mapping_max_range = LaunchConfiguration("mapping_max_range")

    bringup_share = FindPackageShare("drone_bringup")
    sim_launch = PathJoinSubstitution([bringup_share, "launch", "sim.launch.py"])
    controller_launch = PathJoinSubstitution(
        [bringup_share, "launch", "controller.launch.py"]
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "headless",
            default_value="0",
            description="1 disables the Gazebo GUI; RViz remains enabled.",
        ),
        DeclareLaunchArgument(
            "px4_terminal",
            default_value="inline",
            description="Use gnome-terminal for an interactive PX4 shell.",
        ),
        DeclareLaunchArgument(
            "follow",
            default_value="true",
            description="Keep the Gazebo camera behind the drone.",
        ),
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument("use_local_joy", default_value="true"),
        DeclareLaunchArgument("navigation_client_terminal", default_value="true"),
        DeclareLaunchArgument("controller_delay", default_value="12.0"),
        DeclareLaunchArgument("mapping_delay", default_value="10.0"),
        DeclareLaunchArgument("mapping_rate", default_value="10.0"),
        DeclareLaunchArgument("mapping_resolution", default_value="0.5"),
        DeclareLaunchArgument("mapping_max_range", default_value="200.0"),
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "world": "optuna_course",
            "headless": headless,
            "px4_terminal": px4_terminal,
            "follow": follow,
            "use_rviz": "true",
        }.items(),
    )

    manual_controller = TimerAction(
        period=controller_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch),
                launch_arguments={
                    "device_id": device_id,
                    "use_local_joy": use_local_joy,
                    "navigation_client_terminal": navigation_client_terminal,
                }.items(),
            )
        ],
    )

    mapping = TimerAction(
        period=mapping_delay,
        actions=[
            Node(
                package="drone_navigation",
                executable="point_cloud_throttle_node",
                name="octomap_cloud_throttle",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "publish_rate": ParameterValue(mapping_rate, value_type=float),
                    "input_topic": "/scan_3d/points",
                    "output_topic": "/scan_3d/filtered_points",
                    "min_valid_range": 0.2,
                    "max_valid_range": ParameterValue(
                        mapping_max_range, value_type=float
                    ),
                }],
            ),
            Node(
                package="octomap_server",
                executable="octomap_server_node",
                name="octomap_server",
                output="screen",
                remappings=[("cloud_in", "/scan_3d/filtered_points")],
                parameters=[{
                    "use_sim_time": True,
                    "frame_id": "map",
                    "base_frame_id": "base_link",
                    "resolution": ParameterValue(mapping_resolution, value_type=float),
                    "sensor_model.max_range": ParameterValue(
                        mapping_max_range, value_type=float
                    ),
                    "filter_ground_plane": False,
                    "occupancy_min_z": -1.0,
                    "occupancy_max_z": 60.0,
                }],
            ),
        ],
    )

    return LaunchDescription(
        launch_arguments + [simulation, manual_controller, mapping]
    )
