"""Visualize the VTOL + its sensors in RViz.

Brings up:
  * robot_state_publisher  -> /robot_description + TF base_link->lidar_3d_link,
                               lidar_down_link, and the PTZ gimbal chain
                               (driven live by /gimbal/joint_state, bridged
                               from gz -> remapped to the RSP's "joint_states")
  * vtol_tf_broadcaster    -> map->base_link from PX4 (default) so the drone MOVES
  * rviz2                  -> drone_lidar.rviz

The gz->ROS bridge (/scan_3d/points, /lidar_down, /gimbal_camera, /clock, ...)
is NOT here; it lives in sim.launch.py, so sensor topics are available whether
or not RViz is running.

TF for map->base_link comes from ONE of two sources (never both, to avoid a
double publisher):
  * use_broadcaster:=true (default) -> vtol_tf_broadcaster, live PX4 pose.
  * use_static_tf:=true             -> a fixed map->base_link at the origin,
                                       useful when PX4 isn't publishing yet.

Assumes PX4 SITL + gz (and, for the broadcaster, the micro-XRCE agent) are
already running, e.g. `ros2 launch drone_bringup sim.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_static_tf = LaunchConfiguration("use_static_tf")
    use_broadcaster = LaunchConfiguration("use_broadcaster")

    description_share = FindPackageShare("drone_description")

    urdf_xacro = PathJoinSubstitution([description_share, "urdf", "drone.urdf.xacro"])
    rviz_config = PathJoinSubstitution([description_share, "rviz", "drone_lidar.rviz"])

    robot_description = ParameterValue(Command(["xacro ", urdf_xacro]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_broadcaster",
            default_value="true",
            description="Run vtol_tf_broadcaster for a live map->base_link from PX4.",
        ),
        DeclareLaunchArgument(
            "use_static_tf",
            default_value="false",
            description="Instead publish a fixed map->base_link at the origin "
                        "(use when PX4 isn't publishing; don't combine with use_broadcaster).",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
            # gz's JointStatePublisher plugin publishes gimbal angles on
            # /gimbal/joint_state (bridged from gz); feed that into RSP's
            # default "joint_states" input so the gimbal TF chain moves.
            remappings=[("joint_states", "/gimbal/joint_state")],
        ),

        # Live map->base_link from the PX4 estimator (default).
        Node(
            package="commander_cpp",
            executable="vtol_tf_broadcaster",
            name="vtol_tf_broadcaster",
            output="screen",
            condition=IfCondition(use_broadcaster),
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # Fixed map->base_link fallback (only if the broadcaster is off).
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_base",
            output="screen",
            condition=IfCondition(use_static_tf),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--frame-id", "map", "--child-frame-id", "base_link",
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
