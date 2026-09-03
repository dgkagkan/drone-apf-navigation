"""Run the modular automated VTOL mission with shared 3D APF safety."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visualization_enabled = LaunchConfiguration("visualization_enabled")
    transform_enabled = LaunchConfiguration("transform_enabled")
    config_file = LaunchConfiguration("config_file")
    argument_defaults = {
        "mission_profile": "single",
        "goal_x": "700.0",
        "goal_y": "0.0",
        "cruise_altitude": "15.0",
        "goal_2_x": "0.0",
        "goal_2_y": "0.0",
        "goal_2_altitude": "15.0",
        "goal_3_x": "750.0",
        "goal_3_y": "15.0",
        "goal_3_altitude": "15.0",
        "goal_approach_distance": "30.0",
        "intermediate_goal_tolerance": "25.0",
        "goal_tolerance": "5.0",
        "landing_handover_altitude": "3.0",
        "landing_altitude_tolerance": "0.5",
        "landing_max_horizontal_speed": "0.8",
        "landing_settle_time": "1.0",
        "takeoff_climb_speed": "3.0",
        "auto_start_delay": "2.0",
        "target_system": "1",
        "mc_speed": "4.0",
        "mc_climb_speed": "2.0",
        "fw_speed_min": "10.0",
        "fw_speed_cruise": "20.0",
        "fw_speed_max": "20.0",
        "fw_lookahead": "40.0",
        "avoidance_enabled": "true",
        "obstacle_influence_radius": "200.0",
        "fw_avoid_trigger_dist": "190.0",
        "mc_trail_half_width": "2.5",
        "fw_trail_half_width": "10.0",
        "lidar_vertical_half_fov_deg": "15.0",
        "ground_filter_height": "0.4",
        "mc_attractive_gain": "1.0",
        "fw_attractive_gain": "1.0",
        "mc_repulsive_gain": "2.5",
        "fw_repulsive_gain": "20.0",
        "repulsive_distance_power": "2.0",
        "fw_max_avoid_angle_deg": "35.0",
        "fw_max_avoid_pitch_deg": "10.0",
        "vertical_escape_pitch_gain": "2.0",
        "apf_clearance_radius": "1.5",
        "avoidance_clear_hold_time": "2.0",
        "sector_margin_min_deg": "15.0",
        "sector_margin_max_deg": "35.0",
        "sector_margin_speed_min": "3.0",
        "sector_margin_speed_max": "20.0",
        "sector_direction_min_speed": "0.5",
        "emergency_radius": "5.0",
        "lidar_timeout": "0.5",
    }
    default_config = PathJoinSubstitution([
        FindPackageShare("drone_control"), "config", "controller.yaml"
    ])
    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("drone_description"),
                "urdf",
                "drone.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )
    launch_arguments = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in argument_defaults.items()
    ]
    launch_arguments.extend([
        DeclareLaunchArgument("visualization_enabled", default_value="false"),
        DeclareLaunchArgument(
            "transform_enabled",
            default_value="true",
            description="Publish map-to-base and vehicle sensor transforms.",
        ),
        DeclareLaunchArgument("config_file", default_value=default_config),
    ])
    value = {name: LaunchConfiguration(name) for name in argument_defaults}
    base_parameters = [config_file, {"use_sim_time": True}]

    mission_parameters = {
        name: value[name]
        for name in (
            "mission_profile",
            "goal_x",
            "goal_y",
            "cruise_altitude",
            "goal_2_x",
            "goal_2_y",
            "goal_2_altitude",
            "goal_3_x",
            "goal_3_y",
            "goal_3_altitude",
            "goal_approach_distance",
            "intermediate_goal_tolerance",
            "goal_tolerance",
            "landing_handover_altitude",
            "landing_altitude_tolerance",
            "landing_max_horizontal_speed",
            "landing_settle_time",
            "takeoff_climb_speed",
            "auto_start_delay",
            "mc_speed",
            "mc_climb_speed",
            "fw_speed_min",
            "fw_speed_cruise",
            "fw_speed_max",
            "avoidance_enabled",
        )
    }
    mission_parameters["obstacle_timeout_s"] = value["lidar_timeout"]

    apf_parameters = {
        name: value[name]
        for name in (
            "avoidance_enabled",
            "obstacle_influence_radius",
            "fw_avoid_trigger_dist",
            "mc_trail_half_width",
            "fw_trail_half_width",
            "lidar_vertical_half_fov_deg",
            "mc_attractive_gain",
            "fw_attractive_gain",
            "mc_repulsive_gain",
            "fw_repulsive_gain",
            "repulsive_distance_power",
            "fw_max_avoid_angle_deg",
            "fw_max_avoid_pitch_deg",
            "vertical_escape_pitch_gain",
            "apf_clearance_radius",
            "mc_speed",
            "mc_climb_speed",
            "avoidance_clear_hold_time",
            "sector_margin_min_deg",
            "sector_margin_max_deg",
            "sector_margin_speed_min",
            "sector_margin_speed_max",
            "sector_direction_min_speed",
            "emergency_radius",
        )
    }
    apf_parameters["obstacle_timeout_s"] = value["lidar_timeout"]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            condition=IfCondition(transform_enabled),
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
            remappings=[("joint_states", "/gimbal/joint_state")],
        ),
        Node(
            package="drone_control",
            executable="vehicle_tf_broadcaster_node",
            name="vehicle_tf_broadcaster",
            output="screen",
            condition=IfCondition(transform_enabled),
            parameters=[{
                "use_sim_time": True,
                "use_ground_truth": True,
            }],
        ),
        Node(
            package="drone_control",
            executable="px4_gateway_node",
            name="px4_gateway",
            output="screen",
            parameters=base_parameters + [{
                "target_system": value["target_system"],
                "fw_lookahead_m": value["fw_lookahead"],
            }],
        ),
        Node(
            package="drone_control",
            executable="flight_supervisor_node",
            name="flight_supervisor",
            output="screen",
            parameters=base_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="lidar_processor_node",
            name="lidar_processor",
            output="screen",
            parameters=base_parameters + [{
                "ground_height_m": value["ground_filter_height"],
            }],
        ),
        Node(
            package="drone_navigation",
            executable="command_mux_node",
            name="command_mux",
            output="screen",
            parameters=base_parameters,
        ),
        Node(
            package="drone_navigation",
            executable="apf_safety_node",
            name="apf_safety",
            output="screen",
            parameters=base_parameters + [apf_parameters],
        ),
        Node(
            package="drone_control",
            executable="automated_mission_node",
            name="automated_mission",
            output="screen",
            parameters=base_parameters + [mission_parameters],
        ),
        Node(
            package="drone_navigation",
            executable="apf_visualizer_node",
            name="apf_visualizer",
            output="screen",
            condition=IfCondition(visualization_enabled),
            parameters=base_parameters + [{
                "goal_x": value["goal_x"],
                "goal_y": value["goal_y"],
                "cruise_altitude": value["cruise_altitude"],
            }],
        ),
        Node(
            package="drone_navigation",
            executable="point_cloud_throttle_node",
            name="octomap_cloud_throttle",
            output="screen",
            condition=IfCondition(visualization_enabled),
            parameters=[{
                "use_sim_time": True,
                "publish_rate": 10.0,
                "input_topic": "/scan_3d/points",
                "output_topic": "/scan_3d/filtered_points",
                "min_valid_range": 0.2,
                "max_valid_range": 200.0,
            }],
        ),
        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            condition=IfCondition(visualization_enabled),
            remappings=[("cloud_in", "/scan_3d/filtered_points")],
            parameters=[{
                "use_sim_time": True,
                "frame_id": "map",
                "base_frame_id": "base_link",
                "resolution": 0.5,
                "sensor_model.max_range": 200.0,
                "filter_ground_plane": False,
                "occupancy_min_z": -1.0,
                "occupancy_max_z": 60.0,
            }],
        ),
    ]
    return LaunchDescription(launch_arguments + nodes)
