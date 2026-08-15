"""Start one independent drone brain without Gazebo, PX4 SITL, RViz, or coordinator."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drone_id = LaunchConfiguration("drone_id")
    target_system = LaunchConfiguration("target_system")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ground_truth = LaunchConfiguration("use_ground_truth")
    publish_vehicle_tf = LaunchConfiguration("publish_vehicle_tf")
    publish_robot_description = LaunchConfiguration("publish_robot_description")
    config_file = LaunchConfiguration("config_file")
    loopback_config = PathJoinSubstitution([
        FindPackageShare("drone_bringup"), "config", "fastdds_loopback.xml"
    ])
    ethernet_config = PathJoinSubstitution([
        FindPackageShare("drone_bringup"), "config", "fastdds_ethernet_pi.xml"
    ])
    network_mode = LaunchConfiguration("network_mode")
    local_network = IfCondition(PythonExpression(["'", network_mode, "' == 'local'"]))
    lan_network = UnlessCondition(PythonExpression(["'", network_mode, "' == 'local'"]))

    controller_launch = PathJoinSubstitution([
        FindPackageShare("drone_bringup"), "launch", "controller.launch.py"
    ])
    default_config = PathJoinSubstitution([
        FindPackageShare("drone_control"), "config", "controller.yaml"
    ])
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("drone_description"), "urdf", "drone.urdf.xacro"
    ])
    robot_description = ParameterValue(
        Command(["xacro ", urdf_xacro]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "network_mode", default_value="lan", choices=["local", "lan"]
        ),
        DeclareLaunchArgument(
            "ros_domain_id", default_value=os.environ.get("ROS_DOMAIN_ID", "0")
        ),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", condition=local_network
        ),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET", condition=lan_network
        ),
        UnsetEnvironmentVariable("ROS_LOCALHOST_ONLY"),
        SetEnvironmentVariable(
            "FASTDDS_DEFAULT_PROFILES_FILE", loopback_config, condition=local_network
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", loopback_config, condition=local_network
        ),
        SetEnvironmentVariable(
            "FASTDDS_DEFAULT_PROFILES_FILE", ethernet_config, condition=lan_network
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", ethernet_config, condition=lan_network
        ),
        DeclareLaunchArgument(
            "drone_id",
            description="Unique swarm identity and ROS namespace, for example drone_4.",
        ),
        DeclareLaunchArgument("target_system", description="PX4 MAV system ID."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_ground_truth", default_value="false"),
        DeclareLaunchArgument(
            "publish_vehicle_tf",
            default_value="true",
            description=(
                "Publish map->base_link from this brain. Set false when the "
                "PC simulation launch owns the ground-truth TF."
            ),
        ),
        DeclareLaunchArgument("publish_robot_description", default_value="true"),
        DeclareLaunchArgument("visualization_enabled", default_value="true"),
        DeclareLaunchArgument(
            "lidar_points_topic",
            default_value="scan_3d/points",
            description="Namespaced LiDAR input; use scan_3d/filtered_points for remote simulation.",
        ),
        DeclareLaunchArgument("map_origin_east_m", default_value="0.0"),
        DeclareLaunchArgument("map_origin_north_m", default_value="0.0"),
        DeclareLaunchArgument("map_origin_up_m", default_value="0.0"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(controller_launch),
            launch_arguments={
                "drone_namespace": drone_id,
                "target_system": target_system,
                "map_origin_east_m": LaunchConfiguration("map_origin_east_m"),
                "map_origin_north_m": LaunchConfiguration("map_origin_north_m"),
                "map_origin_up_m": LaunchConfiguration("map_origin_up_m"),
                "use_local_joy": "false",
                "visualization_enabled": LaunchConfiguration("visualization_enabled"),
                "navigation_client_terminal": "false",
                "swarm_member_enabled": "true",
                "use_sim_time": use_sim_time,
                "lidar_points_topic": LaunchConfiguration("lidar_points_topic"),
                "config_file": config_file,
            }.items(),
        ),
        Node(
            package="drone_control",
            executable="vehicle_tf_broadcaster_node",
            namespace=drone_id,
            name="vehicle_tf_broadcaster",
            output="screen",
            condition=IfCondition(publish_vehicle_tf),
            parameters=[{
                "map_frame": "map",
                "base_frame": ParameterValue([drone_id, "/base_link"], value_type=str),
                "use_ground_truth": ParameterValue(use_ground_truth, value_type=bool),
                "ground_truth_topic": ParameterValue(
                    ["/", drone_id, "/ground_truth/odometry"], value_type=str
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=drone_id,
            name="robot_state_publisher",
            output="screen",
            condition=IfCondition(publish_robot_description),
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": ParameterValue([drone_id, "/"], value_type=str),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[("joint_states", ["/", drone_id, "/gimbal/joint_state"])],
        ),
    ])
