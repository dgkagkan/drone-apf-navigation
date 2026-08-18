"""Compatibility entry point for the modular automated APF mission."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "goal_x": "25.0",
        "goal_y": "0.0",
        "cruise_alt": "15.0",
        "mc_speed": "3.0",
        "fw_speed": "15.0",
        "trail_half_width": "2.5",
        "fw_trail_half_width": "10.0",
        "goal_approach_dist": "30.0",
        "obstacle_influence_radius": "200.0",
        "fw_avoid_trigger_dist": "190.0",
        "fw_max_avoid_angle_deg": "12.0",
        "fw_max_avoid_pitch_deg": "10.0",
        "vertical_escape_pitch_gain": "2.0",
        "mc_attractive_gain": "0.8",
        "mc_repulsive_gain": "2.5",
        "fw_attractive_gain": "1.0",
        "fw_repulsive_gain": "6.0",
        "repulsive_distance_power": "2.0",
        "apf_clearance_radius": "1.5",
        "fw_apf_lookahead": "40.0",
        "visualization_enabled": "false",
    }
    declarations = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in defaults.items()
    ]
    automated_launch = PathJoinSubstitution([
        FindPackageShare("drone_bringup"), "launch", "automated_controller.launch.py"
    ])
    mapped_arguments = {
        "goal_x": LaunchConfiguration("goal_x"),
        "goal_y": LaunchConfiguration("goal_y"),
        "cruise_altitude": LaunchConfiguration("cruise_alt"),
        "goal_approach_distance": LaunchConfiguration("goal_approach_dist"),
        "mc_speed": LaunchConfiguration("mc_speed"),
        "fw_speed_cruise": LaunchConfiguration("fw_speed"),
        "mc_trail_half_width": LaunchConfiguration("trail_half_width"),
        "fw_trail_half_width": LaunchConfiguration("fw_trail_half_width"),
        "obstacle_influence_radius": LaunchConfiguration("obstacle_influence_radius"),
        "fw_avoid_trigger_dist": LaunchConfiguration("fw_avoid_trigger_dist"),
        "fw_max_avoid_angle_deg": LaunchConfiguration("fw_max_avoid_angle_deg"),
        "fw_max_avoid_pitch_deg": LaunchConfiguration("fw_max_avoid_pitch_deg"),
        "vertical_escape_pitch_gain": LaunchConfiguration("vertical_escape_pitch_gain"),
        "mc_attractive_gain": LaunchConfiguration("mc_attractive_gain"),
        "mc_repulsive_gain": LaunchConfiguration("mc_repulsive_gain"),
        "fw_attractive_gain": LaunchConfiguration("fw_attractive_gain"),
        "fw_repulsive_gain": LaunchConfiguration("fw_repulsive_gain"),
        "repulsive_distance_power": LaunchConfiguration("repulsive_distance_power"),
        "apf_clearance_radius": LaunchConfiguration("apf_clearance_radius"),
        "fw_lookahead": LaunchConfiguration("fw_apf_lookahead"),
        "visualization_enabled": LaunchConfiguration("visualization_enabled"),
    }
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(automated_launch),
            launch_arguments=mapped_arguments.items(),
        )
    ])
