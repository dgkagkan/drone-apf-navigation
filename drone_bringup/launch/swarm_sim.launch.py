"""Run three independent PX4 VTOLs in one Gazebo world and ROS 2 graph."""

import os
from pathlib import Path
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DRONES = (
    {"id": "drone_1", "instance": 0, "system_id": 1, "agent_port": 8888},
    {"id": "drone_2", "instance": 1, "system_id": 2, "agent_port": 8889},
    {"id": "drone_3", "instance": 2, "system_id": 3, "agent_port": 8890},
)


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


def _px4_command(
    px4_dir, models_dir, worlds_dir, work_dir, world, pose, drone, headless
):
    source_env = Path(px4_dir) / "build/px4_sitl_default/rootfs/gz_env.sh"
    px4_binary = Path(px4_dir) / "build/px4_sitl_default/bin/px4"
    px4_etc = Path(px4_dir) / "build/px4_sitl_default/etc"
    stock_models = Path(px4_dir) / "Tools/simulation/gz/models"
    model_name = f"standard_vtol_lidar_{drone['id']}"
    exports = {
        "PX4_SYS_AUTOSTART": "4030",
        "PX4_SIM_MODEL": f"gz_{model_name}",
        "PX4_GZ_MODEL_POSE": pose,
        "PX4_GZ_WORLD": world,
        "PX4_UXRCE_DDS_NS": drone["id"],
        "PX4_UXRCE_DDS_PORT": str(drone["agent_port"]),
        "PX4_GZ_MODELS": str(models_dir),
        "PX4_GZ_WORLDS": str(worlds_dir),
        "PX4_GZ_NO_FOLLOW": "1",
        "GZ_IP": "127.0.0.1",
    }
    if drone["instance"] > 0:
        exports["PX4_GZ_STANDALONE"] = "1"
    if headless:
        exports["HEADLESS"] = "1"

    commands = [
        f"mkdir -p {shlex.quote(str(work_dir))}",
        f"source {shlex.quote(str(source_env))}",
        f"export GZ_SIM_RESOURCE_PATH={shlex.quote(str(models_dir))}:"
        f"{shlex.quote(str(worlds_dir))}:"
        f"{shlex.quote(str(stock_models))}:${{GZ_SIM_RESOURCE_PATH:-}}",
    ]
    commands.extend(
        f"export {name}={shlex.quote(value)}" for name, value in exports.items()
    )
    commands.append(
        f"exec {shlex.quote(str(px4_binary))} -d {shlex.quote(str(px4_etc))} "
        f"-i {drone['instance']} -w {shlex.quote(str(work_dir))}"
    )
    return " && ".join(commands)


def _launch_setup(context):
    px4_dir = Path(LaunchConfiguration("px4_dir").perform(context)).expanduser().resolve()
    agent = Path(LaunchConfiguration("agent").perform(context)).expanduser().resolve()
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() in ("1", "true")
    operator_terminal = LaunchConfiguration("operator_terminal").perform(context)
    use_dashboard = LaunchConfiguration("use_dashboard").perform(context)
    open_dashboard = LaunchConfiguration("open_dashboard").perform(context)
    use_rviz = LaunchConfiguration("use_rviz").perform(context).lower() in ("1", "true")
    use_mapping = LaunchConfiguration("use_mapping").perform(context).lower() in (
        "1", "true"
    )
    mapping_rate_hz = float(LaunchConfiguration("mapping_rate_hz").perform(context))
    mapping_resolution_m = float(
        LaunchConfiguration("mapping_resolution_m").perform(context)
    )
    mapping_max_range_m = float(
        LaunchConfiguration("mapping_max_range_m").perform(context)
    )
    models_dir = Path(LaunchConfiguration("models_dir").perform(context)).expanduser().resolve()
    work_root = Path(LaunchConfiguration("work_root").perform(context)).expanduser().resolve()

    if mapping_rate_hz <= 0.0:
        raise RuntimeError("mapping_rate_hz must be greater than zero")
    if mapping_resolution_m <= 0.0:
        raise RuntimeError("mapping_resolution_m must be greater than zero")
    if mapping_max_range_m <= 0.0:
        raise RuntimeError("mapping_max_range_m must be greater than zero")
    required_paths = (
        px4_dir / "build/px4_sitl_default/bin/px4",
        px4_dir / "build/px4_sitl_default/rootfs/gz_env.sh",
        agent,
    )
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        raise RuntimeError("missing required swarm runtime file(s): " + ", ".join(missing))

    description_share = Path(get_package_share_directory("drone_description"))
    source_model = description_share / "models/standard_vtol_lidar/model.sdf"
    source_text = source_model.read_text(encoding="utf-8")
    models_dir.mkdir(parents=True, exist_ok=True)

    poses = {}
    for drone in DRONES:
        drone_id = drone["id"]
        east = float(LaunchConfiguration(f"{drone_id}_east_m").perform(context))
        north = float(LaunchConfiguration(f"{drone_id}_north_m").perform(context))
        up = float(LaunchConfiguration(f"{drone_id}_up_m").perform(context))
        poses[drone_id] = (east, north, up)
        model_name = f"standard_vtol_lidar_{drone_id}"
        model_dir = models_dir / model_name
        model_dir.mkdir(parents=True, exist_ok=True)
        variant = _prepare_model_variant(source_text, model_name, drone_id)
        (model_dir / "model.sdf").write_text(variant, encoding="utf-8")

    bringup_share = Path(get_package_share_directory("drone_bringup"))
    worlds_dir = bringup_share / "worlds"
    controller_launch = bringup_share / "launch/controller.launch.py"
    swarm_launch = bringup_share / "launch/swarm.launch.py"
    bridge_config = bringup_share / "config/gazebo_bridge_swarm.yaml"
    control_config = Path(get_package_share_directory("drone_control")) / "config/controller.yaml"
    urdf_xacro = description_share / "urdf/drone.urdf.xacro"
    rviz_config = description_share / "rviz/swarm_lidar.rviz"
    robot_description = ParameterValue(Command(["xacro ", str(urdf_xacro)]), value_type=str)

    stock_world = px4_dir / "Tools/simulation/gz/worlds" / f"{world}.sdf"
    custom_world = worlds_dir / f"{world}.sdf"
    if not stock_world.exists() and not custom_world.exists():
        raise RuntimeError(
            f"Gazebo world '{world}' was not found in {worlds_dir} or "
            f"{stock_world.parent}"
        )

    actions = []
    for drone in DRONES:
        actions.append(ExecuteProcess(
            cmd=[str(agent), "udp4", "-p", str(drone["agent_port"])],
            name=f"{drone['id']}_xrce_agent",
            output="screen",
        ))

    for drone in DRONES:
        east, north, up = poses[drone["id"]]
        pose = f"{east},{north},{up},0,0,0"
        px4_command = _px4_command(
            px4_dir,
            models_dir,
            worlds_dir,
            work_root / drone["id"],
            world,
            pose,
            drone,
            headless,
        )
        delay = 1.0 if drone["instance"] == 0 else 4.0 + drone["instance"]
        actions.append(TimerAction(
            period=delay,
            actions=[ExecuteProcess(
                cmd=["bash", "-lc", px4_command],
                name=f"{drone['id']}_px4",
                output="screen",
            )],
        ))

    actions.append(TimerAction(
        period=7.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="swarm_gazebo_bridge",
            output="screen",
            parameters=[{"config_file": str(bridge_config), "use_sim_time": True}],
        )],
    ))

    for drone in DRONES:
        drone_id = drone["id"]
        east, north, up = poses[drone_id]
        actions.append(TimerAction(
            period=9.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(controller_launch)),
                    launch_arguments={
                        "drone_namespace": drone_id,
                        "target_system": str(drone["system_id"]),
                        "map_origin_east_m": str(east),
                        "map_origin_north_m": str(north),
                        "map_origin_up_m": str(up),
                        "use_local_joy": "false",
                        "visualization_enabled": "true",
                        "navigation_client_terminal": "false",
                        "swarm_member_enabled": "true",
                        "use_sim_time": "true",
                        "config_file": str(control_config),
                    }.items(),
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
                    remappings=[("joint_states", f"/{drone_id}/gimbal/joint_state")],
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
                        "ground_truth_topic": f"/{drone_id}/ground_truth/odometry",
                        "use_sim_time": True,
                    }],
                ),
            ],
        ))

    if use_mapping:
        mapping_actions = []
        for drone in DRONES:
            drone_id = drone["id"]
            mapping_actions.append(Node(
                package="drone_navigation",
                executable="point_cloud_throttle_node",
                namespace=drone_id,
                name="swarm_map_cloud_throttle",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "publish_rate": mapping_rate_hz,
                    "input_topic": f"/{drone_id}/scan_3d/points",
                    "output_topic": f"/{drone_id}/scan_3d/filtered_points",
                    "secondary_output_topic": "/swarm/scan_3d/filtered_points",
                    "min_valid_range": 0.2,
                    "max_valid_range": mapping_max_range_m,
                }],
            ))

        # A ROS topic can have multiple publishers. Each cloud keeps its own
        # sensor frame, so octomap_server transforms all three into `map`.
        mapping_actions.append(Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="swarm_octomap_server",
            output="screen",
            remappings=[
                ("cloud_in", "/swarm/scan_3d/filtered_points"),
                ("octomap_point_cloud_centers", "/swarm/octomap_point_cloud_centers"),
                ("octomap_binary", "/swarm/octomap_binary"),
                ("octomap_full", "/swarm/octomap_full"),
            ],
            parameters=[{
                "use_sim_time": True,
                "frame_id": "map",
                "base_frame_id": "map",
                "resolution": mapping_resolution_m,
                "sensor_model.max_range": mapping_max_range_m,
                "filter_ground_plane": False,
                "occupancy_min_z": -1.0,
                "occupancy_max_z": 60.0,
            }],
        ))
        actions.append(TimerAction(period=11.0, actions=mapping_actions))

    if use_rviz:
        actions.append(TimerAction(
            period=12.0,
            actions=[Node(
                package="rviz2",
                executable="rviz2",
                name="swarm_rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": True}],
            )],
        ))

    actions.append(TimerAction(
        period=11.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(swarm_launch)),
            launch_arguments={
                "operator_terminal": operator_terminal,
                "use_dashboard": use_dashboard,
                "open_dashboard": open_dashboard,
            }.items(),
        )],
    ))
    return actions


def generate_launch_description():
    home = str(Path.home())
    bringup_share = get_package_share_directory("drone_bringup")
    fastdds_config = Path(bringup_share) / "config/fastdds_loopback.xml"
    ethernet_config = Path(bringup_share) / "config/fastdds_ethernet_pc.xml"
    network_mode = LaunchConfiguration("network_mode")
    local_network = IfCondition(PythonExpression(["'", network_mode, "' == 'local'"]))
    lan_network = UnlessCondition(PythonExpression(["'", network_mode, "' == 'local'"]))
    return LaunchDescription([
        DeclareLaunchArgument(
            "network_mode",
            default_value="local",
            choices=["local", "lan"],
            description="local keeps ROS on loopback; lan allows remote drone brains.",
        ),
        DeclareLaunchArgument(
            "ros_domain_id", default_value=os.environ.get("ROS_DOMAIN_ID", "0")
        ),
        # Gazebo Transport and PX4 remain local in both modes. ROS uses loopback
        # by default and is exposed to the LAN only for remote drone brains.
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
        DeclareLaunchArgument(
            "px4_dir",
            default_value=os.path.join(home, "drone_project/PX4-Autopilot"),
        ),
        DeclareLaunchArgument(
            "agent",
            default_value=os.path.join(
                home, "drone_project/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
            ),
        ),
        DeclareLaunchArgument("world", default_value="test"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("operator_terminal", default_value="false"),
        DeclareLaunchArgument("use_dashboard", default_value="true"),
        DeclareLaunchArgument("open_dashboard", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_mapping", default_value="true"),
        DeclareLaunchArgument("mapping_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("mapping_resolution_m", default_value="0.5"),
        DeclareLaunchArgument("mapping_max_range_m", default_value="300.0"),
        DeclareLaunchArgument("models_dir", default_value="/tmp/drone_swarm_gz_models"),
        DeclareLaunchArgument("work_root", default_value="/tmp/drone_swarm_px4"),
        DeclareLaunchArgument("drone_1_east_m", default_value="0.0"),
        DeclareLaunchArgument("drone_1_north_m", default_value="0.0"),
        DeclareLaunchArgument("drone_1_up_m", default_value="0.0"),
        DeclareLaunchArgument("drone_2_east_m", default_value="0.0"),
        DeclareLaunchArgument("drone_2_north_m", default_value="8.0"),
        DeclareLaunchArgument("drone_2_up_m", default_value="0.0"),
        DeclareLaunchArgument("drone_3_east_m", default_value="0.0"),
        DeclareLaunchArgument("drone_3_north_m", default_value="-8.0"),
        DeclareLaunchArgument("drone_3_up_m", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
