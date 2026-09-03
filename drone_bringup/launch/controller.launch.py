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
    manual_control_enabled = LaunchConfiguration("manual_control_enabled")
    gimbal_control_enabled = LaunchConfiguration("gimbal_control_enabled")
    visualization_enabled = LaunchConfiguration("visualization_enabled")
    navigation_client_terminal = LaunchConfiguration("navigation_client_terminal")
    swarm_member_enabled = LaunchConfiguration("swarm_member_enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")
    lidar_points_topic = LaunchConfiguration("lidar_points_topic")
    drone_namespace = LaunchConfiguration("drone_namespace")
    target_system = LaunchConfiguration("target_system")
    map_origin_east_m = LaunchConfiguration("map_origin_east_m")
    map_origin_north_m = LaunchConfiguration("map_origin_north_m")
    map_origin_up_m = LaunchConfiguration("map_origin_up_m")
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution([
        FindPackageShare("drone_control"), "config", "controller.yaml"
    ])
    shared_parameters = [config_file, {"use_sim_time": use_sim_time}]
    apf_mode_parameters = [
        config_file,
        PathJoinSubstitution([
            FindPackageShare("drone_navigation"),
            "config",
            "apf_modes",
            "stable.yaml",
        ]),
        PathJoinSubstitution([
            FindPackageShare("drone_navigation"),
            "config",
            "apf_modes",
            "normal.yaml",
        ]),
        PathJoinSubstitution([
            FindPackageShare("drone_navigation"),
            "config",
            "apf_modes",
            "sport.yaml",
        ]),
        {"use_sim_time": use_sim_time},
    ]
    # Existing nodes use absolute ROS names. Remap them to relative names so
    # the launch namespace isolates one complete drone control stack.
    namespaced_remappings = [
        ("/joy", "joy"),
        ("/vehicle/state", "vehicle/state"),
        ("/fmu/out/vehicle_status_v4", "fmu/out/vehicle_status_v4"),
        ("/fmu/out/vehicle_local_position_v1", "fmu/out/vehicle_local_position_v1"),
        ("/fmu/out/vehicle_attitude", "fmu/out/vehicle_attitude"),
        ("/fmu/in/offboard_control_mode", "fmu/in/offboard_control_mode"),
        ("/fmu/in/trajectory_setpoint", "fmu/in/trajectory_setpoint"),
        ("/fmu/in/vehicle_command", "fmu/in/vehicle_command"),
        ("/px4/vehicle_command_request", "px4/vehicle_command_request"),
        ("/flight/request", "flight/request"),
        ("/flight/arm", "flight/arm"),
        ("/takeoff", "takeoff"),
        ("/land", "land"),
        ("/transition_vtol", "transition_vtol"),
        ("/motion/manual_intent", "motion/manual_intent"),
        ("/motion/autonomous_intent", "motion/autonomous_intent"),
        ("/motion/supervisor_intent", "motion/supervisor_intent"),
        ("/motion/selected_intent", "motion/selected_intent"),
        ("/motion/safe_command", "motion/safe_command"),
        ("/navigation/manual_override", "navigation/manual_override"),
        ("/navigation/speed_override", "navigation/speed_override"),
        ("/navigation/active_goal", "navigation/active_goal"),
        ("/navigation/flown_path", "navigation/flown_path"),
        ("/navigation/nominal_path", "navigation/nominal_path"),
        ("/navigation/validate_goal", "navigation/validate_goal"),
        ("/navigate_to", "navigate_to"),
        ("/scan_3d/points", lidar_points_topic),
        ("/perception/obstacles", "perception/obstacles"),
        ("/perception/lidar_range_override", "perception/lidar_range_override"),
        ("/apf/set_enabled", "apf/set_enabled"),
        ("/apf/set_mode", "apf/set_mode"),
        ("/apf/telemetry", "apf/telemetry"),
        ("/apf/forces", "apf/forces"),
        ("/apf/obstacles_used", "apf/obstacles_used"),
        ("/apf/obstacles_sector_ignored", "apf/obstacles_sector_ignored"),
        ("/gimbal/cmd_pan", "gimbal/cmd_pan"),
        ("/gimbal/cmd_tilt", "gimbal/cmd_tilt"),
        ("/gimbal/joint_state", "gimbal/joint_state"),
    ]
    navigation_client_command = [
        TextSubstitution(
            text='ros2 run drone_navigation navigation_client_node '
            '--ros-args --params-file "'
        ),
        config_file,
        TextSubstitution(text='" -r __ns:=/'),
        drone_namespace,
        TextSubstitution(
            text=" -r /navigate_to:=navigate_to"
            " -r /takeoff:=takeoff"
            " -r /navigation/validate_goal:=navigation/validate_goal"
            " -r /flight/arm:=flight/arm"
            " -r /apf/set_enabled:=apf/set_enabled"
            " -r /apf/set_mode:=apf/set_mode"
            " -r /vehicle/state:=vehicle/state"
            " -r /apf/telemetry:=apf/telemetry; exec bash"
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument(
            "use_local_joy",
            default_value="true",
            description="false receives /joy from a remote ROS 2 computer.",
        ),
        DeclareLaunchArgument("manual_control_enabled", default_value="true"),
        DeclareLaunchArgument("gimbal_control_enabled", default_value="true"),
        DeclareLaunchArgument("visualization_enabled", default_value="true"),
        DeclareLaunchArgument("navigation_client_terminal", default_value="true"),
        DeclareLaunchArgument("swarm_member_enabled", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("lidar_points_topic", default_value="scan_3d/points"),
        DeclareLaunchArgument(
            "drone_namespace",
            default_value="",
            description="Namespace for one complete drone stack, for example drone_1.",
        ),
        DeclareLaunchArgument("target_system", default_value="1"),
        DeclareLaunchArgument("map_origin_east_m", default_value="0.0"),
        DeclareLaunchArgument("map_origin_north_m", default_value="0.0"),
        DeclareLaunchArgument("map_origin_up_m", default_value="0.0"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="joy",
            executable="game_controller_node",
            name="ps4_game_controller",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(use_local_joy),
            parameters=[{
                "device_id": device_id,
                "deadzone": 0.08,
                "autorepeat_rate": 30.0,
                "coalesce_interval_ms": 1,
            }],
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_control",
            executable="px4_gateway_node",
            name="px4_gateway",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters + [{
                "target_system": target_system,
                "map_origin_east_m": map_origin_east_m,
                "map_origin_north_m": map_origin_north_m,
                "map_origin_up_m": map_origin_up_m,
            }],
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_control",
            executable="flight_supervisor_node",
            name="flight_supervisor",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_control",
            executable="manual_control_node",
            name="manual_control",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(manual_control_enabled),
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_control",
            executable="gimbal_control_node",
            name="gimbal_control",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(gimbal_control_enabled),
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_navigation",
            executable="lidar_processor_node",
            name="lidar_processor",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_navigation",
            executable="command_mux_node",
            name="command_mux",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_navigation",
            executable="apf_safety_node",
            name="apf_safety",
            namespace=drone_namespace,
            output="screen",
            parameters=apf_mode_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_navigation",
            executable="navigation_server_node",
            name="navigation_server",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_navigation",
            executable="route_executor_node",
            name="route_executor",
            namespace=drone_namespace,
            output="screen",
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
        Node(
            package="drone_swarm",
            executable="swarm_member_node",
            name="swarm_member",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(swarm_member_enabled),
            parameters=[config_file, {
                "drone_id": drone_namespace,
                "use_sim_time": use_sim_time,
            }],
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
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(visualization_enabled),
            parameters=shared_parameters,
            remappings=namespaced_remappings,
        ),
    ])
