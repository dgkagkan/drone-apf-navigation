"""Bring up the full PX4 VTOL simulation from the ROS workspace.

Starts, so you don't have to `cd` into the PX4 tree:
  * MicroXRCEAgent   (udp4:8888)  -> PX4 <-> ROS 2 (px4_msgs) bridge
  * PX4 SITL + Gazebo             -> `make px4_sitl gz_standard_vtol_lidar`
                                     with PX4_GZ_WORLD selecting the world.
  * ros_gz_bridge                 -> exposes gz /scan and /clock as ROS topics
                                     (so /scan is available with or without RViz).
  * RViz (+ robot_state_publisher + vtol_tf_broadcaster) -> via rviz.launch.py,
                                     started a bit later so gz is up first.
                                     Disable with use_rviz:=false.

The gz world is chosen via PX4_GZ_WORLD, so the single base target works for
any world (default, forest, singapure, ...). Set headless:=1 for no GUI.

PX4's pxh> shell needs an interactive terminal (`ros2 launch` does not forward
stdin). So by default PX4 is opened in its OWN gnome-terminal window, where you
can type `commander takeoff`, while the agent + bridge stay in this terminal.

  px4_terminal:=gnome-terminal  (default) -> PX4 in a popup terminal, pxh usable
  px4_terminal:=inline                    -> PX4 in this terminal, NO pxh
  start_px4:=false                        -> don't start PX4 at all (run it yourself)

Example:
  ros2 launch drone_bringup sim.launch.py
  ros2 launch drone_bringup sim.launch.py world:=default headless:=1
  ros2 launch drone_bringup sim.launch.py px4_terminal:=inline
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    home = os.path.expanduser("~")
    px4_dir = LaunchConfiguration("px4_dir")
    agent = LaunchConfiguration("agent")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    start_px4 = LaunchConfiguration("start_px4")
    px4_terminal = LaunchConfiguration("px4_terminal")
    follow = LaunchConfiguration("follow")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_delay = LaunchConfiguration("rviz_delay")

    bridge_config = PathJoinSubstitution(
        [FindPackageShare("drone_bringup"), "config", "gazebo_bridge.yaml"]
    )
    rviz_launch = PathJoinSubstitution(
        [FindPackageShare("drone_bringup"), "launch", "rviz.launch.py"]
    )

    # NOTE: PX4's px4-rc.gzsim starts the gz GUI only when HEADLESS is EMPTY
    # (`if [ -z "$HEADLESS" ]`). So we must NOT set HEADLESS=0 (that is a
    # non-empty string and would suppress the GUI). Only export HEADLESS=1
    # when headless was actually requested; otherwise leave it unset.
    make_cmd = [
        TextSubstitution(text="cd "), px4_dir,
        TextSubstitution(text=" && export PX4_GZ_WORLD="), world,
        TextSubstitution(text='; if [ "'), headless,
        TextSubstitution(text='" = "1" ]; then export HEADLESS=1; fi'),
        TextSubstitution(text='; if [ "'), follow,
        TextSubstitution(text='" = "false" ]; then export PX4_GZ_NO_FOLLOW=1; fi'),
        TextSubstitution(text=" && make px4_sitl gz_standard_vtol_lidar"),
    ]

    # For the popup terminal: keep the window open after PX4/pxh exits.
    make_inner = make_cmd + [TextSubstitution(text="; exec bash")]

    run_px4_term = IfCondition(PythonExpression([
        "'", start_px4, "'.lower() in ('true', '1') and '", px4_terminal, "' != 'inline'",
    ]))
    run_px4_inline = IfCondition(PythonExpression([
        "'", start_px4, "'.lower() in ('true', '1') and '", px4_terminal, "' == 'inline'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value=os.path.join(home, "drone_project", "PX4-Autopilot"),
            description="Path to the PX4-Autopilot tree.",
        ),
        DeclareLaunchArgument(
            "agent",
            default_value=os.path.join(
                home, "drone_project", "Micro-XRCE-DDS-Agent", "build", "MicroXRCEAgent"
            ),
            description="Path to the MicroXRCEAgent binary.",
        ),
        DeclareLaunchArgument("world", default_value="forest"),
        DeclareLaunchArgument("headless", default_value="0",
                              description="1 = no Gazebo GUI."),
        DeclareLaunchArgument(
            "follow", default_value="true",
            description="false = free Gazebo GUI camera (don't lock/follow the drone).",
        ),
        DeclareLaunchArgument(
            "start_px4", default_value="true",
            description="false = don't start PX4 here; still starts agent + bridge.",
        ),
        DeclareLaunchArgument(
            "px4_terminal", default_value="gnome-terminal",
            description="gnome-terminal = PX4 in a popup window with interactive "
                        "pxh; 'inline' = run PX4 in this terminal (no pxh).",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Also start RViz (+ robot_state_publisher + vtol_tf_broadcaster).",
        ),
        DeclareLaunchArgument(
            "rviz_delay", default_value="8.0",
            description="Seconds to wait before starting RViz (let gz boot first).",
        ),

        # Micro-XRCE-DDS agent (PX4 uORB <-> ROS 2)
        ExecuteProcess(
            cmd=[agent, "udp4", "-p", "8888"],
            output="screen",
            name="micro_xrce_agent",
        ),

        # PX4 SITL + Gazebo in its OWN gnome-terminal -> interactive pxh>
        # (type `commander takeoff` there). Default.
        ExecuteProcess(
            cmd=["gnome-terminal", "--wait", "--title=PX4 SITL (pxh)",
                 "--", "bash", "-lc", make_inner],
            output="screen",
            name="px4_sitl_gz_terminal",
            condition=run_px4_term,
        ),

        # Alternative: PX4 inline in this terminal (no pxh) via px4_terminal:=inline
        ExecuteProcess(
            cmd=["bash", "-lc", make_cmd],
            output="screen",
            name="px4_sitl_gz",
            condition=run_px4_inline,
        ),

        # gz -> ROS bridge (/scan, /clock). Delayed a few seconds so the gz
        # server is up first. With RViz off you still get /scan for the APF nodes.
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="gz_bridge",
                    output="screen",
                    parameters=[{
                        "config_file": bridge_config,
                        "use_sim_time": True,
                    }],
                ),
            ],
        ),

        # RViz (+ robot_state_publisher + vtol_tf_broadcaster). Delayed so gz
        # is up first. Disable with use_rviz:=false.
        TimerAction(
            period=rviz_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([rviz_launch]),
                ),
            ],
            condition=IfCondition(use_rviz),
        ),
    ])
