"""Start the VTOL APF offboard stack.

This launch file intentionally does not start PX4/Gazebo. Use it beside
sim.launch.py after the vehicle is spawned and the MicroXRCE bridge is alive.

Examples:
  ros2 launch drone_bringup apf.launch.py
  ros2 launch drone_bringup apf.launch.py engage:=true auto_arm:=true goal_x:=30 goal_y:=8 goal_z:=5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    engage = LaunchConfiguration("engage")
    auto_arm = LaunchConfiguration("auto_arm")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_z = LaunchConfiguration("goal_z")
    cruise_speed = LaunchConfiguration("cruise_speed")

    return LaunchDescription([
        DeclareLaunchArgument(
            "engage",
            default_value="false",
            description="true = request PX4 offboard and send APF velocity setpoints.",
        ),
        DeclareLaunchArgument(
            "auto_arm",
            default_value="false",
            description="true = also arm through VehicleCommand after offboard warmup.",
        ),
        DeclareLaunchArgument("goal_x", default_value="25.0", description="Goal East in map/ENU meters."),
        DeclareLaunchArgument("goal_y", default_value="0.0", description="Goal North in map/ENU meters."),
        DeclareLaunchArgument("goal_z", default_value="5.0", description="Goal Up in map/ENU meters."),
        DeclareLaunchArgument("cruise_speed", default_value="3.0", description="Horizontal APF speed cap."),

        Node(
            package="commander_cpp",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_z": goal_z,
                "cruise_speed": cruise_speed,
            }],
        ),

        Node(
            package="commander_cpp",
            executable="vtol_offboard_control",
            name="vtol_offboard_control",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "engage": engage,
                "auto_arm": auto_arm,
                "max_speed": cruise_speed,
            }],
        ),
    ])
