"""Start one independent drone brain without Gazebo, PX4 SITL, RViz, or coordinator."""

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
    mapping_enabled = LaunchConfiguration("mapping_enabled")
    mapping_rate_hz = LaunchConfiguration("mapping_rate_hz")
    mapping_max_range_m = LaunchConfiguration("mapping_max_range_m")
    mapping_queue_size = LaunchConfiguration("mapping_queue_size")
    lidar_points_topic = LaunchConfiguration("lidar_points_topic")
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
            "ros_domain_id",
            default_value="0",
            description="ROS 2 domain shared with the swarm coordinator.",
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
        DeclareLaunchArgument(
            "mapping_enabled",
            default_value="true",
            description=(
                "Run the lightweight onboard LiDAR relay and publish "
                "/swarm/<drone_id>/map_cloud. The PC owns global fusion."
            ),
        ),
        DeclareLaunchArgument("mapping_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("mapping_max_range_m", default_value="300.0"),
        DeclareLaunchArgument(
            "mapping_queue_size",
            default_value="5",
            description="Small onboard mapping DDS queue; normally keep this 5-10.",
        ),
        DeclareLaunchArgument(
            "manual_control_enabled",
            default_value="false",
            description="Enable the remote /joy path (requires a full build).",
        ),
        DeclareLaunchArgument(
            "gimbal_control_enabled",
            default_value="false",
            description="Enable joystick gimbal control (disabled on the onboard brain).",
        ),
        DeclareLaunchArgument(
            "gimbal_mux_enabled",
            default_value="true",
            description=(
                "Enable the swarm/teleop gimbal command mux. Keep enabled so "
                "coordinator commands reach the simulated or real gimbal."
            ),
        ),
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
                "manual_control_enabled": LaunchConfiguration(
                    "manual_control_enabled"
                ),
                "gimbal_control_enabled": LaunchConfiguration(
                    "gimbal_control_enabled"
                ),
                "gimbal_mux_enabled": LaunchConfiguration("gimbal_mux_enabled"),
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
                "map_origin_east_m": LaunchConfiguration("map_origin_east_m"),
                "map_origin_north_m": LaunchConfiguration("map_origin_north_m"),
                "map_origin_up_m": LaunchConfiguration("map_origin_up_m"),
                "ground_truth_topic": ParameterValue(
                    ["/", drone_id, "/ground_truth/odometry"], value_type=str
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="drone_navigation",
            executable="point_cloud_throttle_node",
            namespace=drone_id,
            name="swarm_map_cloud_throttle",
            output="screen",
            condition=IfCondition(mapping_enabled),
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "publish_rate": ParameterValue(mapping_rate_hz, value_type=float),
                "input_topic": lidar_points_topic,
                "output_topic": "mapping/filtered_points",
                "secondary_output_topic": "",
                "secondary_include_max_range_rays": True,
                "drone_id": drone_id,
                "mapping_frame": "map",
                "map_cloud_topic": ParameterValue(
                    ["/swarm/", drone_id, "/map_cloud"], value_type=str
                ),
                "mapping_tf_timeout_sec": 0.05,
                "mapping_queue_size": ParameterValue(
                    mapping_queue_size, value_type=int
                ),
                "min_valid_range": 0.2,
                "max_valid_range": ParameterValue(
                    mapping_max_range_m, value_type=float
                ),
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
