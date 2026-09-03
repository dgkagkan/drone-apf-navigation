"""Add one PX4/Gazebo vehicle to an already running swarm simulation on this PC."""

import os
from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"expected exactly one model token: {old}")
    return text.replace(old, new)


def _prepare_model_variant(source_text, model_name, drone_id):
    text = _replace_once(
        source_text,
        '<model name="standard_vtol_lidar">',
        f'<model name="{model_name}">',
    )
    replacements = {
        "<topic>scan_3d</topic>": f"<topic>/{drone_id}/scan_3d</topic>",
        "<topic>lidar_down</topic>": f"<topic>/{drone_id}/lidar_down</topic>",
        "<topic>gimbal_camera</topic>": f"<topic>/{drone_id}/gimbal_camera</topic>",
        "<gz_frame_id>lidar_3d_link</gz_frame_id>":
            f"<gz_frame_id>{drone_id}/lidar_3d_link</gz_frame_id>",
        "<gz_frame_id>lidar_down_link</gz_frame_id>":
            f"<gz_frame_id>{drone_id}/lidar_down_link</gz_frame_id>",
        "<gz_frame_id>gimbal_pitch_link</gz_frame_id>":
            f"<gz_frame_id>{drone_id}/gimbal_pitch_link</gz_frame_id>",
        "<topic>/gimbal/cmd_pan</topic>": f"<topic>/{drone_id}/gimbal/cmd_pan</topic>",
        "<topic>/gimbal/cmd_roll</topic>": f"<topic>/{drone_id}/gimbal/cmd_roll</topic>",
        "<topic>/gimbal/cmd_tilt</topic>": f"<topic>/{drone_id}/gimbal/cmd_tilt</topic>",
        "<topic>/gimbal/joint_state</topic>":
            f"<topic>/{drone_id}/gimbal/joint_state</topic>",
        "<robot_base_frame>base_link</robot_base_frame>":
            f"<robot_base_frame>{drone_id}/base_link</robot_base_frame>",
        "<odom_topic>/drone/ground_truth/odometry</odom_topic>":
            f"<odom_topic>/{drone_id}/ground_truth/odometry</odom_topic>",
    }
    for old, new in replacements.items():
        text = _replace_once(text, old, new)
    return text


def _bridge_config(drone_id):
    entries = (
        ("scan_3d/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", "GZ_TO_ROS"),
        ("lidar_down/points", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", "GZ_TO_ROS"),
        ("ground_truth/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        ("gimbal_camera", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
        ("gimbal/cmd_pan", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
        ("gimbal/cmd_tilt", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
        ("gimbal/cmd_roll", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
        ("gimbal/joint_state", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"),
    )
    blocks = []
    for suffix, ros_type, gz_type, direction in entries:
        topic = f"/{drone_id}/{suffix}"
        blocks.append(
            f'- ros_topic_name: "{topic}"\n'
            f'  gz_topic_name: "{topic}"\n'
            f'  ros_type_name: "{ros_type}"\n'
            f'  gz_type_name: "{gz_type}"\n'
            f'  direction: {direction}\n'
        )
    return "\n".join(blocks)


def _launch_setup(context):
    drone_id = LaunchConfiguration("drone_id").perform(context)
    instance = int(LaunchConfiguration("px4_instance").perform(context))
    system_id = int(LaunchConfiguration("system_id").perform(context))
    agent_port = int(LaunchConfiguration("agent_port").perform(context))
    world = LaunchConfiguration("world").perform(context)
    east = float(LaunchConfiguration("east_m").perform(context))
    north = float(LaunchConfiguration("north_m").perform(context))
    up = float(LaunchConfiguration("up_m").perform(context))
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser().resolve()
    agent = Path(LaunchConfiguration("agent").perform(context)).expanduser().resolve()
    models_dir = Path(LaunchConfiguration("models_dir").perform(context)).expanduser().resolve()
    work_root = Path(LaunchConfiguration("work_root").perform(context)).expanduser().resolve()

    if not drone_id or any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for character in drone_id):
        raise RuntimeError("drone_id must contain only letters, numbers, '_' or '-'")
    if instance < 0 or not 1 <= system_id <= 255 or not 1024 <= agent_port <= 65535:
        raise RuntimeError("invalid PX4 instance, system ID, or XRCE agent port")
    if system_id != instance + 1:
        raise RuntimeError(
            "PX4 SITL derives MAV_SYS_ID as px4_instance + 1; system_id must match"
        )

    source_env = px4_dir / "build/px4_sitl_default/rootfs/gz_env.sh"
    px4_binary = px4_dir / "build/px4_sitl_default/bin/px4"
    px4_etc = px4_dir / "build/px4_sitl_default/etc"
    required = (source_env, px4_binary, agent)
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError("missing simulation runtime file(s): " + ", ".join(missing))

    description_share = Path(get_package_share_directory("drone_description"))
    bringup_share = Path(get_package_share_directory("drone_bringup"))
    source_model = description_share / "models/standard_vtol_lidar/model.sdf"
    urdf_xacro = description_share / "urdf/drone.urdf.xacro"
    worlds_dir = bringup_share / "worlds"
    stock_models = px4_dir / "Tools/simulation/gz/models"
    model_name = f"standard_vtol_lidar_{drone_id}"
    model_dir = models_dir / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    model_dir.joinpath("model.sdf").write_text(
        _prepare_model_variant(source_model.read_text(encoding="utf-8"), model_name, drone_id),
        encoding="utf-8",
    )

    vehicle_work_dir = work_root / drone_id
    vehicle_work_dir.mkdir(parents=True, exist_ok=True)
    bridge_config = vehicle_work_dir / "gazebo_bridge.yaml"
    bridge_config.write_text(_bridge_config(drone_id), encoding="utf-8")
    robot_description = ParameterValue(
        Command(["xacro ", str(urdf_xacro)]), value_type=str
    )

    pose = f"{east},{north},{up},0,0,0"
    exports = {
        "PX4_SYS_AUTOSTART": "4030",
        "PX4_SIM_MODEL": f"gz_{model_name}",
        "PX4_GZ_MODEL_POSE": pose,
        "PX4_GZ_WORLD": world,
        "PX4_GZ_STANDALONE": "1",
        "PX4_UXRCE_DDS_NS": drone_id,
        "PX4_UXRCE_DDS_PORT": str(agent_port),
        "PX4_GZ_MODELS": str(models_dir),
        "PX4_GZ_WORLDS": str(worlds_dir),
        "PX4_GZ_NO_FOLLOW": "1",
        "GZ_IP": "127.0.0.1",
    }
    commands = [
        f"source {shlex.quote(str(source_env))}",
        f"export GZ_SIM_RESOURCE_PATH={shlex.quote(str(models_dir))}:"
        f"{shlex.quote(str(worlds_dir))}:{shlex.quote(str(stock_models))}:"
        "${GZ_SIM_RESOURCE_PATH:-}",
    ]
    commands.extend(
        f"export {name}={shlex.quote(value)}" for name, value in exports.items()
    )
    commands.append(
        f"exec {shlex.quote(str(px4_binary))} -d {shlex.quote(str(px4_etc))} "
        f"-i {instance} -w {shlex.quote(str(vehicle_work_dir))}"
    )

    return [
        ExecuteProcess(
            cmd=[str(agent), "udp4", "-p", str(agent_port)],
            name=f"{drone_id}_xrce_agent",
            output="screen",
        ),
        TimerAction(
            period=1.0,
            actions=[ExecuteProcess(
                cmd=["bash", "-lc", " && ".join(commands)],
                name=f"{drone_id}_px4",
                output="screen",
            )],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{drone_id}_gazebo_bridge",
                    output="screen",
                    parameters=[{
                        "config_file": str(bridge_config),
                        "use_sim_time": True,
                    }],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    namespace=drone_id,
                    name="robot_state_publisher",
                    output="screen",
                    parameters=[{
                        "robot_description": robot_description,
                        "frame_prefix": f"{drone_id}/",
                        "use_sim_time": True,
                    }],
                    remappings=[
                        ("joint_states", f"/{drone_id}/gimbal/joint_state"),
                    ],
                ),
                Node(
                    package="drone_control",
                    executable="vehicle_tf_broadcaster_node",
                    namespace=drone_id,
                    name="vehicle_tf_broadcaster",
                    output="screen",
                    parameters=[{
                        "map_frame": "map",
                        "base_frame": f"{drone_id}/base_link",
                        "use_ground_truth": True,
                        "ground_truth_topic":
                            f"/{drone_id}/ground_truth/odometry",
                        "use_sim_time": True,
                    }],
                ),
            ],
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="drone_navigation",
                executable="point_cloud_throttle_node",
                namespace=drone_id,
                name="swarm_map_cloud_throttle",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "publish_rate": 5.0,
                    "input_topic": f"/{drone_id}/scan_3d/points",
                    "output_topic": f"/{drone_id}/scan_3d/filtered_points",
                    "secondary_output_topic": "/swarm/scan_3d/filtered_points",
                    "min_valid_range": 0.2,
                    "max_valid_range": 300.0,
                }],
            )],
        ),
    ]


def generate_launch_description():
    home = str(Path.home())
    bringup_share = Path(get_package_share_directory("drone_bringup"))
    fastdds_config = bringup_share / "config/fastdds_loopback.xml"
    ethernet_config = bringup_share / "config/fastdds_ethernet_pc.xml"
    network_mode = LaunchConfiguration("network_mode")
    local_network = IfCondition(PythonExpression(["'", network_mode, "' == 'local'"]))
    lan_network = UnlessCondition(PythonExpression(["'", network_mode, "' == 'local'"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            "network_mode", default_value="local", choices=["local", "lan"]
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="0",
            description="ROS 2 domain shared with the running swarm.",
        ),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        UnsetEnvironmentVariable("ROS_LOCALHOST_ONLY"),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", condition=local_network
        ),
        SetEnvironmentVariable(
            "FASTDDS_DEFAULT_PROFILES_FILE", str(fastdds_config), condition=local_network
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", str(fastdds_config), condition=local_network
        ),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET", condition=lan_network
        ),
        SetEnvironmentVariable(
            "FASTDDS_DEFAULT_PROFILES_FILE", str(ethernet_config), condition=lan_network
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", str(ethernet_config), condition=lan_network
        ),
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
        DeclareLaunchArgument("drone_id"),
        DeclareLaunchArgument("px4_instance"),
        DeclareLaunchArgument("system_id"),
        DeclareLaunchArgument("agent_port"),
        DeclareLaunchArgument("world", default_value="test"),
        DeclareLaunchArgument("east_m", default_value="0.0"),
        DeclareLaunchArgument("north_m", default_value="16.0"),
        DeclareLaunchArgument("up_m", default_value="0.0"),
        DeclareLaunchArgument(
            "px4_dir", default_value=os.path.join(home, "drone_project/PX4-Autopilot")
        ),
        DeclareLaunchArgument(
            "agent",
            default_value=os.path.join(
                home, "drone_project/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
            ),
        ),
        DeclareLaunchArgument("models_dir", default_value="/tmp/drone_swarm_gz_models"),
        DeclareLaunchArgument("work_root", default_value="/tmp/drone_swarm_px4"),
        OpaqueFunction(function=_launch_setup),
    ])
