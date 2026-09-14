"""Serve the swarm dashboard and translate typed HTTP requests to ROS 2 calls."""

import json
import math
import queue
import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable, Optional
from urllib.parse import unquote, urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from drone_interfaces.msg import ApfTelemetry, SwarmAssignment, SwarmState, VehicleState
from drone_interfaces.srv import (
    AddSwarmTarget,
    RemoveSwarmTarget,
    SwarmCommand,
    SwarmGimbalCommand,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavigationPath
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


COMMANDS = {
    "calculate": SwarmCommand.Request.CALCULATE,
    "start": SwarmCommand.Request.START_MISSION,
    "recalculate": SwarmCommand.Request.RECALCULATE,
    "clear": SwarmCommand.Request.CLEAR_PENDING_TARGETS,
    "cancel": SwarmCommand.Request.CANCEL_ACTIVE_MISSION,
    "home": SwarmCommand.Request.RETURN_HOME,
    "land": SwarmCommand.Request.LAND,
    "off": SwarmCommand.Request.DISABLE_DRONE,
    "rejoin": SwarmCommand.Request.ENABLE_DRONE,
    "arm": SwarmCommand.Request.ARM,
    "takeoff": SwarmCommand.Request.TAKEOFF,
    "force_disarm": SwarmCommand.Request.FORCE_DISARM,
}

ASSIGNMENT_STATES = {
    SwarmAssignment.WAITING: "waiting",
    SwarmAssignment.ACCEPTED: "accepted",
    SwarmAssignment.EXECUTING: "executing",
    SwarmAssignment.SUCCEEDED: "succeeded",
    SwarmAssignment.FAILED: "failed",
    SwarmAssignment.CANCELED: "canceled",
    SwarmAssignment.REJECTED: "rejected",
}

CAMERA_STALE_SEC = 3.0
MAX_RECORDING_BYTES = 512 * 1024 * 1024
FOLDER_PICKER_TIMEOUT_SEC = 300
RUNTIME_PROFILE_VERSION = 1
DEFAULT_SETTINGS_PROFILE_PATH = (
    Path.home() / ".config" / "drone-apf-navigation" / "runtime_profiles.json"
)
CAMERA_FPS_OPTIONS = (5.0, 15.0, 30.0, 60.0)
CAMERA_RESOLUTIONS = {
    "320x180": (320, 180),
    "640x360": (640, 360),
    "960x540": (960, 540),
}


# These are intentionally limited to values that the running nodes can apply
# safely. Startup-only values such as topic names, LiDAR FOV/samples, map
# resolution, and frame IDs stay in launch/configuration files.
RUNTIME_SETTING_DEFINITIONS = [
    {
        "key": "obstacle_influence_radius",
        "label": "Obstacle avoidance range",
        "node": "apf_safety",
        "parameter": "obstacle_influence_radius",
        "type": "double",
        "default": 70.0,
        "min": 1.0,
        "max": 300.0,
        "step": 1.0,
        "unit": "m",
        "category": "Obstacle avoidance",
        "description": "Maximum distance at which a detected obstacle contributes to the APF safety correction.",
    },
    {
        "key": "fw_avoid_trigger_dist",
        "label": "Fixed-wing avoidance activation distance",
        "node": "apf_safety",
        "parameter": "fw_avoid_trigger_dist",
        "type": "double",
        "default": 60.0,
        "min": 1.0,
        "max": 300.0,
        "step": 1.0,
        "unit": "m",
        "category": "Obstacle avoidance",
        "description": "Distance from an obstacle at which fixed-wing avoidance becomes active.",
    },
    {
        "key": "mc_attractive_gain",
        "label": "Multicopter goal pull strength",
        "node": "apf_safety",
        "parameter": "mc_attractive_gain",
        "type": "double",
        "default": 1.0,
        "min": 0.05,
        "max": 50.0,
        "step": 0.05,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Scales the force pulling a multicopter toward its commanded direction.",
    },
    {
        "key": "fw_attractive_gain",
        "label": "Fixed-wing goal pull strength",
        "node": "apf_safety",
        "parameter": "fw_attractive_gain",
        "type": "double",
        "default": 0.5106891373,
        "min": 0.05,
        "max": 100.0,
        "step": 0.01,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Scales the force pulling a fixed-wing vehicle toward its commanded direction.",
    },
    {
        "key": "mc_repulsive_gain",
        "label": "Multicopter obstacle push strength",
        "node": "apf_safety",
        "parameter": "mc_repulsive_gain",
        "type": "double",
        "default": 2.5,
        "min": 0.0,
        "max": 100.0,
        "step": 0.1,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Scales the force pushing a multicopter away from nearby obstacles.",
    },
    {
        "key": "fw_repulsive_gain",
        "label": "Fixed-wing obstacle push strength",
        "node": "apf_safety",
        "parameter": "fw_repulsive_gain",
        "type": "double",
        "default": 10.4140429356,
        "min": 0.0,
        "max": 200.0,
        "step": 0.1,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Scales the obstacle repulsion applied to the fixed-wing safety command.",
    },
    {
        "key": "fw_max_avoid_angle_deg",
        "label": "Maximum fixed-wing horizontal deviation",
        "node": "apf_safety",
        "parameter": "fw_max_avoid_angle_deg",
        "type": "double",
        "default": 16.8054438656,
        "min": 5.0,
        "max": 60.0,
        "step": 0.1,
        "unit": "deg",
        "category": "Obstacle avoidance",
        "description": "Maximum yaw deflection the APF may request from a fixed-wing vehicle to avoid an obstacle.",
    },
    {
        "key": "fw_max_avoid_pitch_deg",
        "label": "Maximum fixed-wing vertical deviation",
        "node": "apf_safety",
        "parameter": "fw_max_avoid_pitch_deg",
        "type": "double",
        "default": 13.9883463092,
        "min": 1.0,
        "max": 30.0,
        "step": 0.1,
        "unit": "deg",
        "category": "Obstacle avoidance",
        "description": "Maximum pitch deflection the APF may request from a fixed-wing vehicle during avoidance.",
    },
    {
        "key": "repulsive_distance_power",
        "label": "Obstacle proximity response",
        "node": "apf_safety",
        "parameter": "repulsive_distance_power",
        "type": "double",
        "default": 1.0546550712,
        "min": 0.25,
        "max": 4.0,
        "step": 0.01,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Controls how sharply obstacle repulsion grows as the vehicle gets closer.",
    },
    {
        "key": "apf_clearance_radius",
        "label": "Required obstacle clearance",
        "node": "apf_safety",
        "parameter": "apf_clearance_radius",
        "type": "double",
        "default": 1.5,
        "min": 0.1,
        "max": 20.0,
        "step": 0.1,
        "unit": "m",
        "category": "Obstacle avoidance",
        "description": "Safety radius around the vehicle used when deciding whether an obstacle is too close.",
    },
    {
        "key": "vertical_escape_pitch_gain",
        "label": "Vertical escape response",
        "node": "apf_safety",
        "parameter": "vertical_escape_pitch_gain",
        "type": "double",
        "default": 1.1609707637,
        "min": 0.1,
        "max": 5.0,
        "step": 0.01,
        "unit": "",
        "category": "Obstacle avoidance",
        "description": "Controls the pitch response used for a fixed-wing vertical escape maneuver.",
    },
    {
        "key": "mc_speed",
        "label": "Multicopter avoidance speed limit",
        "node": "apf_safety",
        "parameter": "mc_speed",
        "type": "double",
        "default": 4.0,
        "min": 0.2,
        "max": 50.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Obstacle avoidance",
        "description": "Maximum horizontal speed allowed in the APF multicopter safety command.",
    },
    {
        "key": "mc_climb_speed",
        "label": "Multicopter climb speed limit",
        "node": "apf_safety",
        "parameter": "mc_climb_speed",
        "type": "double",
        "default": 2.0,
        "min": 0.2,
        "max": 20.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Obstacle avoidance",
        "description": "Maximum vertical speed allowed during APF multicopter escape behavior.",
    },
    {
        "key": "avoidance_clear_hold_time",
        "label": "Clear-path confirmation time",
        "node": "apf_safety",
        "parameter": "avoidance_clear_hold_time",
        "type": "double",
        "default": 2.0,
        "min": 0.2,
        "max": 30.0,
        "step": 0.1,
        "unit": "s",
        "category": "Obstacle avoidance",
        "description": "How long the obstacle sector must stay clear before avoidance is released.",
    },
    {
        "key": "sector_margin_min_deg",
        "label": "Minimum obstacle viewing margin",
        "node": "apf_safety",
        "parameter": "sector_margin_min_deg",
        "type": "double",
        "default": 15.0,
        "min": 0.0,
        "max": 180.0,
        "step": 1.0,
        "unit": "deg",
        "category": "Obstacle avoidance",
        "description": "Smallest angular margin added around the current and desired travel directions.",
    },
    {
        "key": "sector_margin_max_deg",
        "label": "Maximum obstacle viewing margin",
        "node": "apf_safety",
        "parameter": "sector_margin_max_deg",
        "type": "double",
        "default": 35.0,
        "min": 0.0,
        "max": 180.0,
        "step": 1.0,
        "unit": "deg",
        "category": "Obstacle avoidance",
        "description": "Largest angular margin used at high travel speeds when filtering obstacles.",
    },
    {
        "key": "sector_margin_speed_min",
        "label": "Margin ramp-up speed",
        "node": "apf_safety",
        "parameter": "sector_margin_speed_min",
        "type": "double",
        "default": 3.0,
        "min": 0.0,
        "max": 100.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Obstacle avoidance",
        "description": "Speed at which the sector margin begins interpolating above its minimum.",
    },
    {
        "key": "sector_margin_speed_max",
        "label": "Full margin speed",
        "node": "apf_safety",
        "parameter": "sector_margin_speed_max",
        "type": "double",
        "default": 20.0,
        "min": 0.1,
        "max": 100.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Obstacle avoidance",
        "description": "Speed at which the sector margin reaches its configured maximum.",
    },
    {
        "key": "sector_direction_min_speed",
        "label": "Minimum movement speed for direction",
        "node": "apf_safety",
        "parameter": "sector_direction_min_speed",
        "type": "double",
        "default": 0.5,
        "min": 0.0,
        "max": 20.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Obstacle avoidance",
        "description": "Minimum speed used before a motion direction is considered reliable for sector filtering.",
    },
    {
        "key": "emergency_radius",
        "label": "Emergency obstacle distance",
        "node": "apf_safety",
        "parameter": "emergency_radius",
        "type": "double",
        "default": 5.0,
        "min": 0.1,
        "max": 70.0,
        "step": 0.1,
        "unit": "m",
        "category": "Obstacle avoidance",
        "description": "Distance below which the APF treats an obstacle as an emergency proximity event.",
    },
    {
        "key": "obstacle_timeout_s",
        "label": "Sensor data freshness limit",
        "node": "apf_safety",
        "parameter": "obstacle_timeout_s",
        "type": "double",
        "default": 0.5,
        "min": 0.1,
        "max": 10.0,
        "step": 0.1,
        "unit": "s",
        "category": "Obstacle avoidance",
        "description": "Maximum age of an obstacle cloud before APF considers the perception data stale.",
    },
    {
        "key": "avoidance_enabled",
        "label": "Obstacle avoidance enabled",
        "node": "apf_safety",
        "parameter": "avoidance_enabled",
        "type": "bool",
        "default": True,
        "category": "Obstacle avoidance",
        "description": "Enables or disables the APF safety correction while leaving the navigation command active.",
    },
    {
        "key": "goal_tolerance_m",
        "label": "Goal arrival distance",
        "node": "navigation_server",
        "parameter": "goal_tolerance_m",
        "type": "double",
        "default": 25.0,
        "min": 0.2,
        "max": 200.0,
        "step": 0.5,
        "unit": "m",
        "category": "Flight guidance",
        "description": "Horizontal distance from a goal at which the navigation action considers it reached.",
    },
    {
        "key": "altitude_tolerance_m",
        "label": "Goal height tolerance",
        "node": "navigation_server",
        "parameter": "altitude_tolerance_m",
        "type": "double",
        "default": 2.0,
        "min": 0.2,
        "max": 50.0,
        "step": 0.1,
        "unit": "m",
        "category": "Flight guidance",
        "description": "Vertical distance from a goal at which the navigation action considers altitude reached.",
    },
    {
        "key": "goal_chain_grace_period_s",
        "label": "Next-goal transition delay",
        "node": "navigation_server",
        "parameter": "goal_chain_grace_period_s",
        "type": "double",
        "default": 0.25,
        "min": 0.05,
        "max": 10.0,
        "step": 0.05,
        "unit": "s",
        "category": "Flight guidance",
        "description": "Delay before a completed goal enters the multicopter altitude-hold handoff state.",
    },
    {
        "key": "default_speed_m_s",
        "label": "Default travel speed",
        "node": "navigation_server",
        "parameter": "default_speed_m_s",
        "type": "double",
        "default": 15.0,
        "min": 0.2,
        "max": 100.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Flight guidance",
        "description": "Speed used when a goal does not provide a cruise-speed override.",
    },
    {
        "key": "max_speed_m_s",
        "label": "Maximum travel speed",
        "node": "navigation_server",
        "parameter": "max_speed_m_s",
        "type": "double",
        "default": 20.0,
        "min": 0.2,
        "max": 100.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Flight guidance",
        "description": "Upper limit applied to navigation cruise speed commands.",
    },
    {
        "key": "altitude_gain",
        "label": "Height correction strength",
        "node": "navigation_server",
        "parameter": "altitude_gain",
        "type": "double",
        "default": 0.8,
        "min": 0.1,
        "max": 10.0,
        "step": 0.05,
        "unit": "",
        "category": "Flight guidance",
        "description": "Proportional gain converting altitude error into multicopter vertical velocity.",
    },
    {
        "key": "multicopter_arrival_gain",
        "label": "Slowdown near goal",
        "node": "navigation_server",
        "parameter": "multicopter_arrival_gain",
        "type": "double",
        "default": 0.8,
        "min": 0.1,
        "max": 10.0,
        "step": 0.05,
        "unit": "",
        "category": "Flight guidance",
        "description": "Scales horizontal speed down as a multicopter approaches the goal.",
    },
    {
        "key": "navigation_max_vertical_speed_m_s",
        "label": "Navigation climb/descent limit",
        "node": "navigation_server",
        "parameter": "max_vertical_speed_m_s",
        "type": "double",
        "default": 3.0,
        "min": 0.2,
        "max": 30.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Flight guidance",
        "description": "Maximum vertical velocity generated by the navigation server.",
    },
    {
        "key": "transition_request_period_s",
        "label": "Flight-mode change interval",
        "node": "navigation_server",
        "parameter": "transition_request_period_s",
        "type": "double",
        "default": 1.0,
        "min": 0.2,
        "max": 10.0,
        "step": 0.1,
        "unit": "s",
        "category": "Flight guidance",
        "description": "Minimum time between repeated requests to change between fixed-wing and multicopter modes.",
    },
    {
        "key": "max_range_m",
        "label": "Obstacle sensing distance",
        "node": "lidar_processor",
        "parameter": "max_range_m",
        "type": "double",
        "default": 70.0,
        "min": 70.0,
        "max": 300.0,
        "step": 1.0,
        "unit": "m",
        "category": "Sensors & mapping",
        "description": "Maximum point range passed from the LiDAR processor to the obstacle pipeline.",
    },
    {
        "key": "active_lidar_apf_range",
        "label": "Active obstacle sensing range",
        "node": "swarm_command",
        "parameter": "",
        "type": "double",
        "default": 70.0,
        "min": 70.0,
        "max": 300.0,
        "step": 10.0,
        "unit": "m",
        "scope": "drone_command",
        "category": "Sensors & mapping",
        "description": "Active LiDAR/APF range used by the selected drone for obstacle avoidance. This is sent through the coordinator command and can be applied to one drone or all connected drones.",
    },
    {
        "key": "ground_height_m",
        "label": "Ground removal height",
        "node": "lidar_processor",
        "parameter": "ground_height_m",
        "type": "double",
        "default": 0.4,
        "min": -10.0,
        "max": 10.0,
        "step": 0.05,
        "unit": "m",
        "category": "Sensors & mapping",
        "description": "Points at or below this map height are removed as ground returns.",
    },
    {
        "key": "voxel_size_m",
        "label": "Obstacle detail size",
        "node": "lidar_processor",
        "parameter": "voxel_size_m",
        "type": "double",
        "default": 0.2,
        "min": 0.01,
        "max": 5.0,
        "step": 0.01,
        "unit": "m",
        "category": "Sensors & mapping",
        "description": "Grid size used to deduplicate transformed LiDAR points before publishing obstacles.",
    },
    {
        "key": "fw_lookahead_m",
        "label": "Fixed-wing path preview distance",
        "node": "px4_gateway",
        "parameter": "fw_lookahead_m",
        "type": "double",
        "default": 40.0,
        "min": 5.0,
        "max": 300.0,
        "step": 1.0,
        "unit": "m",
        "category": "Flight safety limits",
        "description": "Distance ahead of the fixed-wing vehicle used to project its altitude-hold position setpoint.",
    },
    {
        "key": "max_horizontal_speed_m_s",
        "label": "Horizontal speed safety limit",
        "node": "px4_gateway",
        "parameter": "max_horizontal_speed_m_s",
        "type": "double",
        "default": 25.0,
        "min": 1.0,
        "max": 100.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Flight safety limits",
        "description": "Final horizontal velocity limit applied immediately before publishing the PX4 setpoint.",
    },
    {
        "key": "px4_max_vertical_speed_m_s",
        "label": "Vertical speed safety limit",
        "node": "px4_gateway",
        "parameter": "max_vertical_speed_m_s",
        "type": "double",
        "default": 5.0,
        "min": 0.2,
        "max": 30.0,
        "step": 0.1,
        "unit": "m/s",
        "category": "Flight safety limits",
        "description": "Final vertical velocity limit applied immediately before publishing the PX4 setpoint.",
    },
    {
        "key": "minimum_altitude_m",
        "label": "Minimum flight height",
        "node": "px4_gateway",
        "parameter": "minimum_altitude_m",
        "type": "double",
        "default": 0.5,
        "min": 0.0,
        "max": 100.0,
        "step": 0.1,
        "unit": "m",
        "category": "Flight safety limits",
        "description": "Lowest altitude used by the gateway when creating altitude-hold position setpoints.",
    },
    {
        "key": "camera_fps",
        "label": "Live camera update rate",
        "node": "dashboard",
        "parameter": "camera_rate_hz",
        "type": "choice",
        "options": list(CAMERA_FPS_OPTIONS),
        "default": 60.0,
        "unit": "FPS",
        "scope": "dashboard",
        "category": "Camera display",
        "description": "How many new camera frames per second the local dashboard decodes, encodes, and serves to the browser. This does not change Gazebo.",
    },
    {
        "key": "camera_resolution",
        "label": "Live camera resolution",
        "node": "dashboard",
        "parameter": "camera_resolution",
        "type": "choice",
        "options": list(CAMERA_RESOLUTIONS),
        "default": "640x360",
        "unit": "",
        "scope": "dashboard",
        "category": "Camera display",
        "description": "Output resolution used by the local dashboard before JPEG encoding. It does not change the Gazebo camera sensor.",
    },
    {
        "key": "camera_image_quality",
        "label": "JPEG image quality",
        "node": "dashboard",
        "parameter": "jpeg_quality",
        "type": "integer",
        "default": 72,
        "min": 20,
        "max": 95,
        "step": 1,
        "unit": "%",
        "scope": "dashboard",
        "category": "Camera display",
        "description": "JPEG compression quality used for the local browser stream. Higher values improve detail but use more CPU, bandwidth, and memory.",
    },
]
RUNTIME_SETTINGS_BY_KEY = {setting["key"]: setting for setting in RUNTIME_SETTING_DEFINITIONS}


@dataclass
class HttpCommand:
    kind: str
    payload: dict
    completed: threading.Event = field(default_factory=threading.Event)
    result: dict = field(default_factory=dict)


@dataclass
class ServerVideoRecording:
    final_path: Path
    temporary_path: Path
    writer: object = None
    frame_size: tuple = ()
    frame_count: int = 0
    last_frame_at: float = 0.0
    error_message: str = ""


class DashboardNode(Node):
    """Own ROS clients/subscriptions and a small PC-local HTTP server."""

    def __init__(self):
        super().__init__("swarm_dashboard")
        self._host = self.declare_parameter("host", "127.0.0.1").value
        self._port = int(self.declare_parameter("port", 8765).value)
        self._takeoff_altitude_m = float(
            self.declare_parameter("takeoff_altitude_m", 15.0).value
        )
        self._takeoff_climb_speed_m_s = float(
            self.declare_parameter("takeoff_climb_speed_m_s", 2.0).value
        )
        self._jpeg_quality = int(self.declare_parameter("jpeg_quality", 72).value)
        self._dashboard_rate_hz = max(
            1.0, float(self.declare_parameter("dashboard_rate_hz", 60.0).value)
        )
        self._camera_rate_hz = max(
            0.5,
            float(
                self.declare_parameter("camera_rate_hz", self._dashboard_rate_hz).value
            ),
        )
        self._camera_resolution = str(
            self.declare_parameter("camera_resolution", "640x360").value
        )
        if self._camera_resolution not in CAMERA_RESOLUTIONS:
            raise ValueError(f"unsupported camera resolution '{self._camera_resolution}'")
        self._recording_rate_hz = max(
            1.0, float(self.declare_parameter("recording_rate_hz", 30.0).value)
        )
        self._photo_save_dir = Path(
            str(
                self.declare_parameter(
                    "photo_save_dir", str(Path.home() / "drone_dashboard_photos")
                ).value
            )
        ).expanduser()
        self._record_save_dir = Path(
            str(
                self.declare_parameter(
                    "record_save_dir", str(Path.home() / "drone_dashboard_recordings")
                ).value
            )
        ).expanduser()
        self._preconfigure_media_storage = bool(
            self.declare_parameter("preconfigure_media_storage", False).value
        )
        self._settings_profile_path = Path(
            str(
                self.declare_parameter(
                    "settings_profile_path", str(DEFAULT_SETTINGS_PROFILE_PATH)
                ).value
            )
        ).expanduser()
        self._web_root = Path(get_package_share_directory("drone_dashboard")) / "web"
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._camera_lock = threading.Lock()
        self._recording_lock = threading.Lock()
        self._requests = queue.Queue()
        self._state = self._empty_state()
        self._sensor_subscriptions = {}
        self._telemetry = {}
        self._motion = {}
        self._paths = {}
        self._camera_frames = {}
        self._last_camera_encode = {}
        self._camera_encoding = set()
        self._server_video_recordings = {}
        self._parameter_clients = {}
        self._apf_enabled_clients = {}
        self._runtime_setting_values = {}
        self._settings_profiles_lock = threading.Lock()
        self._settings_profiles = self._load_settings_profiles()
        self._camera_parameter_callback_handle = self.add_on_set_parameters_callback(
            self._on_parameters_set
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._state_subscription = self.create_subscription(
            SwarmState,
            "/swarm/state",
            self._on_swarm_state,
            state_qos,
            callback_group=self._callback_group,
        )
        self._add_target_client = self.create_client(
            AddSwarmTarget, "/swarm/add_target", callback_group=self._callback_group
        )
        self._remove_target_client = self.create_client(
            RemoveSwarmTarget,
            "/swarm/remove_target",
            callback_group=self._callback_group,
        )
        self._command_client = self.create_client(
            SwarmCommand, "/swarm/command", callback_group=self._callback_group
        )
        self._gimbal_command_client = self.create_client(
            SwarmGimbalCommand,
            "/swarm/gimbal_command",
            callback_group=self._callback_group,
        )
        self._request_timer = self.create_timer(
            1.0 / self._dashboard_rate_hz,
            self._process_requests,
            callback_group=self._callback_group,
        )

        handler = self._make_handler()
        self._http_server = ThreadingHTTPServer((self._host, self._port), handler)
        self._http_server.dashboard = self
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="swarm-dashboard-http",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(f"Swarm dashboard ready at http://{self._host}:{self._port}")

    @staticmethod
    def _empty_state():
        return {
            "connected": False,
            "status_message": "waiting for /swarm/state",
            "pending_target_count": 0,
            "active_target_count": 0,
            "available_drone_count": 0,
            "dispatch_in_progress": False,
            "mission_active": False,
            "plan_ready": False,
            "plan_is_recalculation": False,
            "planned_drone_count": 0,
            "pending_targets": [],
            "active_targets": [],
            "assignments": [],
            "planned_assignments": [],
            "drones": [],
            "updated_at": 0.0,
        }

    def _make_handler(self):
        web_root = self._web_root

        class RequestHandler(DashboardRequestHandler):
            dashboard_web_root = web_root

        return RequestHandler

    def submit(self, kind: str, payload: dict, timeout_sec: float = 5.0):
        request = HttpCommand(kind=kind, payload=payload)
        self._requests.put(request)
        if not request.completed.wait(timeout=timeout_sec):
            return HTTPStatus.GATEWAY_TIMEOUT, {
                "ok": False,
                "message": "ROS command timed out",
            }
        return (
            HTTPStatus.OK if request.result.get("ok") else HTTPStatus.CONFLICT,
            request.result,
        )

    def snapshot(self):
        with self._lock:
            # Callbacks replace values; copy only the containers they mutate.
            # JSON serialization belongs outside the callback lock.
            snapshot = dict(self._state)
            snapshot["telemetry"] = dict(self._telemetry)
            snapshot["motion"] = dict(self._motion)
            snapshot["paths"] = {key: dict(value) for key, value in self._paths.items()}
        now = time.monotonic()
        with self._camera_lock:
            snapshot["camera_drones"] = sorted(
                key for key, (received_at, _) in self._camera_frames.items()
                if now - received_at <= CAMERA_STALE_SEC
            )
        return snapshot

    def camera_frame(self, drone_id: str):
        with self._camera_lock:
            frame = self._camera_frames.get(drone_id)
            if frame is None or time.monotonic() - frame[0] > CAMERA_STALE_SEC:
                return None
            return frame[1]

    def save_camera_snapshot(self, drone_id: str):
        """Save the most recent non-stale JPEG frame for one drone."""
        if not DashboardNode._valid_drone_id(drone_id):
            raise ValueError("invalid drone id")

        frame = DashboardNode.camera_frame(self, drone_id)
        if frame is None:
            return None

        drone_dir = self._photo_save_dir / drone_id
        drone_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"snapshot_{timestamp}.jpg"
        final_path = drone_dir / filename
        temporary_path = drone_dir / f".{filename}.tmp"
        try:
            temporary_path.write_bytes(frame)
            temporary_path.replace(final_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
        return final_path

    def configure_media_directory(self, kind: str, directory: str):
        """Create and select a server-side media directory for browser fallbacks."""
        if kind not in {"photo", "record"}:
            raise ValueError("media kind must be 'photo' or 'record'")
        selected_directory = Path(str(directory)).expanduser()
        if not selected_directory.is_absolute():
            raise ValueError("media directory must be an absolute path")
        selected_directory.mkdir(parents=True, exist_ok=True)
        if kind == "photo":
            self._photo_save_dir = selected_directory
        else:
            self._record_save_dir = selected_directory
        self.get_logger().info(f"Dashboard {kind} storage: {selected_directory}")
        return selected_directory

    def pick_media_directory(self, kind: str):
        """Open the dashboard host's native folder picker and select storage."""
        if kind not in {"photo", "record"}:
            raise ValueError("media kind must be 'photo' or 'record'")
        title = "Choose snapshot folder" if kind == "photo" else "Choose recording folder"
        picker = shutil.which("zenity")
        if picker:
            command = [picker, "--file-selection", "--directory", f"--title={title}"]
        else:
            picker = shutil.which("kdialog")
            if not picker:
                raise RuntimeError("no native folder picker found; install zenity or kdialog")
            command = [picker, "--getexistingdirectory", str(Path.home()), "--title", title]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                check=False,
                text=True,
                timeout=FOLDER_PICKER_TIMEOUT_SEC,
            )
        except subprocess.TimeoutExpired as exception:
            raise RuntimeError("folder selection timed out") from exception
        if result.returncode == 1:
            return None
        if result.returncode != 0:
            reason = result.stderr.strip() or f"folder picker exited with code {result.returncode}"
            raise RuntimeError(reason)
        selected_directory = result.stdout.strip()
        if not selected_directory:
            return None
        return DashboardNode.configure_media_directory(self, kind, selected_directory)

    def start_server_recording(self, drone_id: str):
        """Start recording decoded ROS camera frames for one drone."""
        if not DashboardNode._valid_drone_id(drone_id):
            raise ValueError("invalid drone id")
        if DashboardNode.camera_frame(self, drone_id) is None:
            raise ValueError("camera frame unavailable or stale")

        with self._recording_lock:
            if drone_id in self._server_video_recordings:
                raise ValueError(f"{drone_id} recording is already active")
            drone_dir = self._record_save_dir / drone_id
            drone_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"recording_{timestamp}.avi"
            self._server_video_recordings[drone_id] = ServerVideoRecording(
                final_path=drone_dir / filename,
                temporary_path=drone_dir / f".{filename}.tmp.avi",
            )
        self.get_logger().info(f"Started {drone_id} server recording")

    def stop_server_recording(self, drone_id: str):
        """Stop one server recording and atomically publish the AVI file."""
        with self._recording_lock:
            recording = self._server_video_recordings.pop(drone_id, None)
            if recording is None:
                raise ValueError(f"{drone_id} recording is not active")
            if recording.writer is not None:
                recording.writer.release()

        if recording.error_message:
            recording.temporary_path.unlink(missing_ok=True)
            raise RuntimeError(recording.error_message)
        if recording.frame_count == 0 or not recording.temporary_path.exists():
            recording.temporary_path.unlink(missing_ok=True)
            raise ValueError("recording did not receive camera frames")
        recording.temporary_path.replace(recording.final_path)
        self.get_logger().info(
            f"Saved {drone_id} recording ({recording.frame_count} frames): "
            f"{recording.final_path}"
        )
        return recording.final_path

    def _record_camera_frame(self, drone_id: str, image, now: float):
        if not hasattr(self, "_server_video_recordings"):
            return
        with self._recording_lock:
            recording = self._server_video_recordings.get(drone_id)
            if recording is None or recording.error_message:
                return
            if now - recording.last_frame_at < 1.0 / self._recording_rate_hz:
                return
            try:
                frame = image
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                height, width = frame.shape[:2]
                if recording.writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
                    recording.writer = cv2.VideoWriter(
                        str(recording.temporary_path),
                        fourcc,
                        self._recording_rate_hz,
                        (width, height),
                    )
                    if not recording.writer.isOpened():
                        recording.writer.release()
                        recording.writer = None
                        raise RuntimeError("OpenCV could not open the MJPG video writer")
                    recording.frame_size = (width, height)
                if recording.frame_size != (width, height):
                    frame = cv2.resize(frame, recording.frame_size)
                recording.writer.write(frame)
                recording.frame_count += 1
                recording.last_frame_at = now
            except (RuntimeError, cv2.error) as exception:
                recording.error_message = str(exception)
                if recording.writer is not None:
                    recording.writer.release()
                    recording.writer = None
                self.get_logger().error(
                    f"Cannot record {drone_id} camera: {recording.error_message}"
                )

    def save_recording(self, drone_id: str, content: bytes):
        """Save a browser-recorded WebM file on the dashboard host."""
        if not DashboardNode._valid_drone_id(drone_id):
            raise ValueError("invalid drone id")
        if not content:
            raise ValueError("recording is empty")

        drone_dir = self._record_save_dir / drone_id
        drone_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"recording_{timestamp}.webm"
        final_path = drone_dir / filename
        temporary_path = drone_dir / f".{filename}.tmp"
        try:
            temporary_path.write_bytes(content)
            temporary_path.replace(final_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise
        self.get_logger().info(f"Saved {drone_id} recording: {final_path}")
        return final_path

    @staticmethod
    def _valid_drone_id(drone_id: str):
        return (
            bool(drone_id)
            and drone_id not in {".", ".."}
            and "/" not in drone_id
            and "\\" not in drone_id
        )

    def _on_swarm_state(self, message: SwarmState):
        for drone in message.drones:
            self._ensure_drone(drone.drone_id, drone.drone_namespace)
        state = {
            "connected": True,
            "status_message": message.status_message,
            "pending_target_count": message.pending_target_count,
            "active_target_count": message.active_target_count,
            "available_drone_count": message.available_drone_count,
            "dispatch_in_progress": message.dispatch_in_progress,
            "mission_active": message.mission_active,
            "plan_ready": message.plan_ready,
            "plan_is_recalculation": message.plan_is_recalculation,
            "planned_drone_count": message.planned_drone_count,
            "pending_targets": [self._target_dict(target) for target in message.pending_targets],
            "active_targets": [self._target_dict(target) for target in message.active_targets],
            "assignments": [self._assignment_dict(item) for item in message.assignments],
            "planned_assignments": [
                self._assignment_dict(item) for item in message.planned_assignments
            ],
            "drones": [self._drone_dict(drone) for drone in message.drones],
            "updated_at": time.time(),
        }
        with self._lock:
            self._state = state

    def _ensure_drone(self, drone_id: str, namespace: str):
        if not drone_id or not namespace:
            return
        if drone_id in self._sensor_subscriptions:
            return
        normalized = "/" + namespace.strip("/")
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        subscriptions = [
            self.create_subscription(
                Image,
                normalized + "/gimbal_camera",
                lambda message, current_id=drone_id: self._on_camera(current_id, message),
                sensor_qos,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                ApfTelemetry,
                normalized + "/apf/telemetry",
                lambda message, current_id=drone_id: self._on_telemetry(current_id, message),
                sensor_qos,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                VehicleState,
                normalized + "/vehicle/state",
                lambda message, current_id=drone_id: self._on_vehicle_state(
                    current_id, message
                ),
                sensor_qos,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                NavigationPath,
                normalized + "/navigation/nominal_path",
                lambda message, current_id=drone_id: self._on_path(
                    current_id, "nominal", message
                ),
                path_qos,
                callback_group=self._callback_group,
            ),
            self.create_subscription(
                NavigationPath,
                normalized + "/navigation/flown_path",
                lambda message, current_id=drone_id: self._on_path(
                    current_id, "flown", message
                ),
                path_qos,
                callback_group=self._callback_group,
            ),
        ]
        self._sensor_subscriptions[drone_id] = subscriptions
        self.get_logger().info(f"Dashboard discovered {drone_id} at {normalized}")

    def _on_parameters_set(self, parameters):
        result = SetParametersResult()
        result.successful = True
        try:
            updated_rate = self._camera_rate_hz
            updated_quality = self._jpeg_quality
            updated_resolution = self._camera_resolution
            for parameter in parameters:
                if parameter.name == "camera_rate_hz":
                    updated_rate = float(parameter.value)
                elif parameter.name == "jpeg_quality":
                    updated_quality = int(parameter.value)
                elif parameter.name == "camera_resolution":
                    updated_resolution = str(parameter.value)

            if not 0.5 <= updated_rate <= 60.0:
                raise ValueError("camera FPS must be between 0.5 and 60")
            if not 20 <= updated_quality <= 95:
                raise ValueError("JPEG quality must be between 20 and 95")
            if updated_resolution not in CAMERA_RESOLUTIONS:
                raise ValueError("unsupported camera resolution")
            with self._camera_lock:
                self._camera_rate_hz = updated_rate
                self._jpeg_quality = updated_quality
                self._camera_resolution = updated_resolution
        except (TypeError, ValueError) as exception:
            result.successful = False
            result.reason = str(exception)
        return result

    def _on_camera(self, drone_id: str, message: Image):
        now = time.monotonic()
        with self._camera_lock:
            if drone_id in self._camera_encoding:
                return
            camera_rate_hz = self._camera_rate_hz
            jpeg_quality = self._jpeg_quality
            camera_resolution = self._camera_resolution
            if now - self._last_camera_encode.get(drone_id, 0.0) < 1.0 / camera_rate_hz:
                return
            self._camera_encoding.add(drone_id)
            self._last_camera_encode[drone_id] = now
        try:
            image = self._decode_image(message)
            DashboardNode._record_camera_frame(self, drone_id, image, now)
            output_width, output_height = CAMERA_RESOLUTIONS[camera_resolution]
            if image.shape[1] != output_width or image.shape[0] != output_height:
                image = cv2.resize(image, (output_width, output_height), interpolation=cv2.INTER_AREA)
            encoded, jpeg = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
            )
            if not encoded:
                return
            frame = jpeg.tobytes()
            with self._camera_lock:
                self._camera_frames[drone_id] = (now, frame)
        except (ValueError, cv2.error) as exception:
            self.get_logger().warning(f"Cannot encode {drone_id} camera: {exception}")
        finally:
            with self._camera_lock:
                self._camera_encoding.discard(drone_id)

    @staticmethod
    def _decode_image(message: Image):
        encodings = {
            "mono8": (1, None),
            "8uc1": (1, None),
            "rgb8": (3, cv2.COLOR_RGB2BGR),
            "bgr8": (3, None),
            "rgba8": (4, cv2.COLOR_RGBA2BGR),
            "bgra8": (4, cv2.COLOR_BGRA2BGR),
        }
        encoding = message.encoding.lower()
        if encoding not in encodings:
            raise ValueError(f"unsupported image encoding '{message.encoding}'")
        channels, conversion = encodings[encoding]
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, : message.width * channels]
        image = image.reshape(message.height, message.width, channels)
        if channels == 1:
            image = image[:, :, 0]
        if conversion is not None:
            image = cv2.cvtColor(image, conversion)
        return image

    def _on_telemetry(self, drone_id: str, message: ApfTelemetry):
        telemetry = {
            "avoidance_active": message.avoidance_active,
            "active_mode": message.active_mode,
            "nearest_obstacle_m": self._finite(message.nearest_path_obstacle_distance_m),
            "attractive": self._vector_dict(message.attractive_force_enu),
            "repulsive": self._vector_dict(message.repulsive_force_enu),
            "safe_command": self._vector_dict(message.safe_command_enu),
        }
        with self._lock:
            self._telemetry[drone_id] = telemetry

    def _on_vehicle_state(self, drone_id: str, message: VehicleState):
        motion = {
            "velocity": self._vector_dict(message.velocity_enu),
            "speed_m_s": self._speed_m_s(message.velocity_enu),
        }
        with self._lock:
            self._motion[drone_id] = motion

    def _on_path(self, drone_id: str, path_kind: str, message: NavigationPath):
        points = [self._point_dict(pose.pose.position) for pose in message.poses[-1000:]]
        with self._lock:
            self._paths.setdefault(drone_id, {})[path_kind] = points

    def _process_requests(self):
        for _ in range(20):
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            try:
                self._dispatch_request(request)
            except Exception as exception:  # keep HTTP errors away from the ROS executor
                self._finish(request, False, f"command failed: {exception}")

    def _setting_targets(self, requested_target="ALL"):
        with self._lock:
            drones = [dict(drone) for drone in self._state.get("drones", [])]
        connected = [drone for drone in drones if drone.get("connected")]
        target = str(requested_target or "ALL")
        if target.upper() == "ALL":
            return sorted(connected, key=lambda drone: drone.get("drone_id", ""))
        selected = [drone for drone in connected if drone.get("drone_id") == target]
        if not selected:
            raise ValueError(f"drone '{target}' is not connected")
        return selected

    def _parameter_client(self, drone, node_name):
        drone_id = drone["drone_id"]
        key = (drone_id, node_name)
        client = self._parameter_clients.get(key)
        if client is None:
            namespace = str(drone.get("namespace") or drone_id).strip("/")
            client = AsyncParameterClient(self, f"/{namespace}/{node_name}")
            self._parameter_clients[key] = client
        return client

    def _apf_enabled_client(self, drone):
        drone_id = drone["drone_id"]
        client = self._apf_enabled_clients.get(drone_id)
        if client is None:
            namespace = str(drone.get("namespace") or drone_id).strip("/")
            client = self.create_client(SetBool, f"/{namespace}/apf/set_enabled")
            self._apf_enabled_clients[drone_id] = client
        return client

    @staticmethod
    def _parameter_value(value):
        if isinstance(value, Parameter):
            return value.value
        if value.type == Parameter.Type.BOOL:
            return bool(value.bool_value)
        if value.type == Parameter.Type.DOUBLE:
            return float(value.double_value)
        if value.type == Parameter.Type.INTEGER:
            return int(value.integer_value)
        if value.type == Parameter.Type.STRING:
            return value.string_value
        return None

    @staticmethod
    def _profile_name(raw_name):
        name = " ".join(str(raw_name or "").split())
        if not name:
            raise ValueError("profile name cannot be empty")
        if len(name) > 80:
            raise ValueError("profile name must be 80 characters or fewer")
        return name

    def _load_settings_profiles(self):
        if not self._settings_profile_path.exists():
            return {}
        try:
            document = json.loads(self._settings_profile_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exception:
            self.get_logger().warning(
                f"Could not load settings profiles from {self._settings_profile_path}: {exception}"
            )
            return {}

        raw_profiles = document.get("profiles", {}) if isinstance(document, dict) else {}
        if not isinstance(raw_profiles, dict):
            self.get_logger().warning("Settings profile file has an invalid profiles object")
            return {}

        profiles = {}
        for raw_name, raw_profile in raw_profiles.items():
            try:
                name = self._profile_name(raw_name)
                raw_values = raw_profile.get("values", {})
                if not isinstance(raw_values, dict):
                    raise ValueError("profile values must be an object")
                values = {}
                for key, raw_value in raw_values.items():
                    setting = RUNTIME_SETTINGS_BY_KEY.get(str(key))
                    if setting is None:
                        continue
                    values[setting["key"]] = self._coerce_setting_value(setting, raw_value)
                profiles[name] = {
                    "name": name,
                    "saved_at": str(raw_profile.get("saved_at", "")),
                    "values": values,
                }
            except (AttributeError, TypeError, ValueError) as exception:
                self.get_logger().warning(
                    f"Ignoring invalid settings profile '{raw_name}': {exception}"
                )
        return profiles

    def _write_settings_profiles(self):
        document = {
            "version": RUNTIME_PROFILE_VERSION,
            "profiles": self._settings_profiles,
        }
        self._settings_profile_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self._settings_profile_path.with_name(
            f".{self._settings_profile_path.name}.tmp"
        )
        try:
            temporary_path.write_text(
                json.dumps(document, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            temporary_path.replace(self._settings_profile_path)
        except Exception:
            temporary_path.unlink(missing_ok=True)
            raise

    def _settings_profiles_response(self):
        with self._settings_profiles_lock:
            profiles = [
                {
                    "name": profile["name"],
                    "saved_at": profile["saved_at"],
                    "values": dict(profile["values"]),
                }
                for profile in sorted(
                    self._settings_profiles.values(),
                    key=lambda profile: profile["name"].lower(),
                )
            ]
        return {
            "ok": True,
            "message": "Settings profiles loaded",
            "profiles": profiles,
        }

    def _get_settings_profiles(self, command):
        command.result = self._settings_profiles_response()
        command.completed.set()

    def _save_settings_profile(self, command):
        payload = command.payload
        try:
            name = self._profile_name(payload.get("name"))
            raw_values = payload.get("values")
            if not isinstance(raw_values, dict) or not raw_values:
                raise ValueError("profile must contain at least one setting")
            values = {}
            for key, raw_value in raw_values.items():
                setting = RUNTIME_SETTINGS_BY_KEY.get(str(key))
                if setting is None:
                    raise ValueError(f"unknown runtime setting '{key}'")
                values[setting["key"]] = self._coerce_setting_value(setting, raw_value)
        except (TypeError, ValueError) as exception:
            self._finish(command, False, str(exception))
            return

        profile = {
            "name": name,
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "values": values,
        }
        try:
            with self._settings_profiles_lock:
                self._settings_profiles[name] = profile
                self._write_settings_profiles()
        except OSError as exception:
            self._finish(command, False, f"profile could not be saved: {exception}")
            return
        self._finish(command, True, f"Profile '{name}' saved")

    def _delete_settings_profile(self, command):
        try:
            name = self._profile_name(command.payload.get("name"))
        except ValueError as exception:
            self._finish(command, False, str(exception))
            return
        try:
            with self._settings_profiles_lock:
                if name not in self._settings_profiles:
                    self._finish(command, False, f"Profile '{name}' does not exist")
                    return
                del self._settings_profiles[name]
                self._write_settings_profiles()
        except OSError as exception:
            self._finish(command, False, f"profile could not be deleted: {exception}")
            return
        self._finish(command, True, f"Profile '{name}' deleted")

    def _settings_response(self, targets, values):
        target_ids = [drone["drone_id"] for drone in targets]
        response_values = {
            drone_id: dict(values.get(drone_id, {})) for drone_id in target_ids
        }
        if "dashboard" in values:
            response_values["dashboard"] = dict(values["dashboard"])
        return {
            "ok": True,
            "message": "Runtime settings loaded",
            "settings": [dict(setting) for setting in RUNTIME_SETTING_DEFINITIONS],
            "targets": target_ids,
            "values": response_values,
        }

    def _get_settings(self, command):
        targets = self._setting_targets("ALL")
        values = {
            drone["drone_id"]: dict(self._runtime_setting_values.get(drone["drone_id"], {}))
            for drone in targets
        }
        values["dashboard"] = dict(self._runtime_setting_values.get("dashboard", {}))
        jobs = []
        grouped = {}
        for setting in RUNTIME_SETTING_DEFINITIONS:
            if setting.get("scope") == "dashboard":
                try:
                    value = self._parameter_value(self.get_parameter(setting["parameter"]))
                    if value is not None:
                        values["dashboard"][setting["key"]] = value
                        self._runtime_setting_values.setdefault("dashboard", {})[
                            setting["key"]
                        ] = value
                except Exception as exception:
                    self.get_logger().warning(
                        f"Could not read dashboard setting {setting['key']}: {exception}"
                    )
                continue
            if setting.get("scope") == "drone_command":
                state_drones = {
                    drone.get("drone_id"): drone for drone in targets
                }
                for drone_id, drone in state_drones.items():
                    value = drone.get("lidar_range_m")
                    if value is not None:
                        values[drone_id][setting["key"]] = value
                        self._runtime_setting_values.setdefault(drone_id, {})[
                            setting["key"]
                        ] = value
                continue
            grouped.setdefault(setting["node"], []).append(setting)
        for drone in targets:
            for node_name, settings in grouped.items():
                client = self._parameter_client(drone, node_name)
                if not client.services_are_ready():
                    continue
                future = client.get_parameters([setting["parameter"] for setting in settings])
                jobs.append((future, drone["drone_id"], settings))

        if not jobs:
            command.result = self._settings_response(targets, values)
            command.completed.set()
            return

        pending = len(jobs)
        result_lock = threading.Lock()

        def completed(result_future, drone_id, settings):
            nonlocal pending
            try:
                parameter_values = result_future.result().values
                for setting, parameter_value in zip(settings, parameter_values):
                    value = self._parameter_value(parameter_value)
                    if value is not None:
                        values[drone_id][setting["key"]] = value
                        self._runtime_setting_values.setdefault(drone_id, {})[
                            setting["key"]
                        ] = value
            except Exception as exception:
                self.get_logger().warning(
                    f"Could not read runtime settings from {drone_id}: {exception}"
                )
            with result_lock:
                pending -= 1
                if pending == 0:
                    command.result = self._settings_response(targets, values)
                    command.completed.set()

        for future, drone_id, settings in jobs:
            future.add_done_callback(
                lambda result_future, current_id=drone_id, current_settings=settings:
                completed(result_future, current_id, current_settings)
            )

    @staticmethod
    def _coerce_setting_value(setting, raw_value):
        if setting["type"] == "bool":
            if isinstance(raw_value, bool):
                return raw_value
            if isinstance(raw_value, str) and raw_value.lower() in {"true", "false"}:
                return raw_value.lower() == "true"
            raise ValueError("boolean setting requires true or false")
        if setting["type"] == "choice":
            for option in setting.get("options", []):
                if isinstance(option, (int, float)) and not isinstance(option, bool):
                    try:
                        if float(raw_value) == float(option):
                            return option
                    except (TypeError, ValueError):
                        continue
                elif str(raw_value) == str(option):
                    return option
            options = ", ".join(str(option) for option in setting.get("options", []))
            raise ValueError(f"setting must be one of: {options}")
        if setting["type"] == "integer":
            try:
                value = int(raw_value)
            except (TypeError, ValueError) as exception:
                raise ValueError("integer setting requires a whole number") from exception
            if str(raw_value).strip() != str(value) and not isinstance(raw_value, int):
                raise ValueError("integer setting requires a whole number")
        else:
            try:
                value = float(raw_value)
            except (TypeError, ValueError) as exception:
                raise ValueError("numeric setting requires a number") from exception
        if setting["type"] == "integer":
            numeric_value = float(value)
        else:
            numeric_value = value
        if not math.isfinite(numeric_value):
            raise ValueError("setting must be finite")
        if "min" in setting and numeric_value < setting["min"]:
            raise ValueError(f"setting must be at least {setting['min']}")
        if "max" in setting and numeric_value > setting["max"]:
            raise ValueError(f"setting must be at most {setting['max']}")
        return value

    def _set_apf_enabled(self, command, setting, value, targets):
        ready_clients = []
        results = []
        for drone in targets:
            client = self._apf_enabled_client(drone)
            if not client.service_is_ready():
                results.append((
                    drone["drone_id"], False, "/apf/set_enabled service unavailable"))
                continue
            ready_clients.append((drone, client))

        pending = len(ready_clients)
        if pending == 0:
            failed = "; ".join(
                f"{drone_id}: {reason}" for drone_id, _, reason in results)
            self._finish(command, False, f"Setting rejected; {failed}")
            return
        result_lock = threading.Lock()

        def completed(result_future, drone_id):
            nonlocal pending
            try:
                response = result_future.result()
                successful = bool(response.success)
                reason = "" if successful else str(response.message)
            except Exception as exception:
                successful = False
                reason = str(exception)
            with result_lock:
                results.append((drone_id, successful, reason))
                pending -= 1
                if pending != 0:
                    return
                successful_ids = [item[0] for item in results if item[1]]
                failed = [f"{item[0]}: {item[2]}" for item in results if not item[1]]
                for current_id in successful_ids:
                    self._runtime_setting_values.setdefault(current_id, {})[
                        setting["key"]
                    ] = value
                if failed:
                    message = (
                        f"Applied {setting['label']} to "
                        f"{len(successful_ids)}/{len(targets)} drone(s)"
                    )
                    if successful_ids:
                        message += "; " + ", ".join(failed)
                    else:
                        message = "Setting rejected; " + ", ".join(failed)
                    self._finish(command, bool(successful_ids), message)
                else:
                    self._finish(
                        command, True,
                        f"Applied {setting['label']} to {', '.join(successful_ids)}",
                    )

        for drone, client in ready_clients:
            request = SetBool.Request()
            request.data = value
            future = client.call_async(request)
            future.add_done_callback(
                lambda result_future, current_id=drone["drone_id"]:
                completed(result_future, current_id)
            )

    def _set_setting(self, command):
        """Apply one or more runtime settings as a single dashboard action.

        The legacy key/value payload is still accepted for compatibility with
        older dashboard bundles. The current UI sends a list of changes. All
        values are validated before any ROS service is called.
        """
        payload = command.payload
        raw_changes = payload.get("changes")
        if raw_changes is None:
            if "key" not in payload:
                self._finish(command, False, "no runtime setting changes supplied")
                return
            raw_changes = [{"key": payload.get("key"), "value": payload.get("value")}]
        if not isinstance(raw_changes, list) or not raw_changes:
            self._finish(command, False, "changes must be a non-empty list")
            return

        changes = []
        try:
            for raw_change in raw_changes:
                if not isinstance(raw_change, dict):
                    raise ValueError("each runtime setting change must be an object")
                key = str(raw_change.get("key", ""))
                setting = RUNTIME_SETTINGS_BY_KEY.get(key)
                if setting is None:
                    raise ValueError(f"unknown runtime setting '{key}'")
                value = self._coerce_setting_value(setting, raw_change.get("value"))
                changes.append((key, setting, value))
            targets = self._setting_targets(payload.get("target", "ALL"))
        except (TypeError, ValueError) as exception:
            self._finish(command, False, str(exception))
            return
        local_changes = [change for change in changes if change[1].get("scope") == "dashboard"]
        drone_changes = [change for change in changes if change[1].get("scope") != "dashboard"]
        if not targets and drone_changes:
            self._finish(command, False, "no connected drones are available")
            return

        results = []
        operations = []
        result_lock = threading.Lock()
        parameter_changes = [
            change for change in drone_changes
            if change[0] != "avoidance_enabled" and
            change[1].get("scope") != "drone_command"
        ]
        enabled_changes = [change for change in drone_changes if change[0] == "avoidance_enabled"]
        command_changes = [
            change for change in drone_changes
            if change[1].get("scope") == "drone_command"
        ]

        if local_changes:
            try:
                local_results = self.set_parameters([
                    Parameter(setting["parameter"], value=value)
                    for _, setting, value in local_changes
                ])
                for (key, _, _), result in zip(local_changes, local_results):
                    results.append(("dashboard", key, bool(result.successful), str(result.reason)))
                if len(local_results) < len(local_changes):
                    for key, _, _ in local_changes[len(local_results):]:
                        results.append(("dashboard", key, False, "dashboard returned no result"))
            except Exception as exception:
                results.extend(
                    ("dashboard", key, False, str(exception))
                    for key, _, _ in local_changes
                )

        for drone in targets:
            drone_id = drone["drone_id"]
            if parameter_changes:
                grouped = {}
                for key, setting, value in parameter_changes:
                    grouped.setdefault(setting["node"], []).append((key, setting, value))
                for node_name, node_changes in grouped.items():
                    client = self._parameter_client(drone, node_name)
                    if not client.services_are_ready():
                        for key, _, _ in node_changes:
                            results.append((drone_id, key, False,
                                            f"/{node_name} parameter service unavailable"))
                    else:
                        operations.append(("parameters", drone, client, node_changes))

            if enabled_changes:
                setting = enabled_changes[0][1]
                value = enabled_changes[0][2]
                client = self._apf_enabled_client(drone)
                if not client.service_is_ready():
                    results.append((drone_id, setting["key"], False,
                                    "/apf/set_enabled service unavailable"))
                else:
                    operations.append(("avoidance_enabled", drone, client,
                                       [(setting["key"], setting, value)]))

            if command_changes:
                if not self._command_client.service_is_ready():
                    for key, _, _ in command_changes:
                        results.append((drone_id, key, False,
                                        "/swarm/command service unavailable"))
                else:
                    operations.append(("drone_command", drone, self._command_client,
                                       command_changes))

        pending = len(operations)
        total_updates = len(drone_changes) * len(targets) + len(local_changes)

        def finish_all():
            successful = [item for item in results if item[2]]
            failed = [item for item in results if not item[2]]
            for scope_id, key, _, _ in successful:
                value = next(value for current_key, _, value in changes if current_key == key)
                self._runtime_setting_values.setdefault(scope_id, {})[key] = value
            if failed:
                failure_text = "; ".join(
                    f"{scope_id}/{key}: {reason}" for scope_id, key, _, reason in failed
                )
                message = f"Applied {len(successful)}/{total_updates} setting updates"
                if successful:
                    message += f"; {failure_text}"
                else:
                    message = f"Settings rejected; {failure_text}"
                self._finish(command, bool(successful), message)
            else:
                self._finish(
                    command, True,
                    f"Applied {len(successful)} setting updates to {len(targets)} drone(s)",
                )

        def finish_operation(operation_results):
            nonlocal pending
            with result_lock:
                results.extend(operation_results)
                pending -= 1
                if pending != 0:
                    return
            finish_all()

        if pending == 0:
            finish_all()
            return

        for operation_kind, drone, client, node_changes in operations:
            drone_id = drone["drone_id"]
            if operation_kind == "avoidance_enabled":
                request = SetBool.Request()
                request.data = node_changes[0][2]
                future = client.call_async(request)

                def completed_enabled(result_future, current_id=drone_id, key=node_changes[0][0]):
                    try:
                        response = result_future.result()
                        successful = bool(response.success)
                        reason = "" if successful else str(response.message)
                    except Exception as exception:
                        successful = False
                        reason = str(exception)
                    finish_operation([(current_id, key, successful, reason)])

                future.add_done_callback(completed_enabled)
                continue

            if operation_kind == "drone_command":
                request = SwarmCommand.Request()
                request.command = SwarmCommand.Request.SET_DRONE_LIDAR_RANGE
                request.drone_id = drone_id
                request.lidar_range_m = float(node_changes[0][2])
                future = client.call_async(request)

                def completed_drone_command(
                    result_future, current_id=drone_id, current_changes=node_changes
                ):
                    operation_results = []
                    try:
                        response = result_future.result()
                        successful = bool(response.accepted)
                        reason = "" if successful else str(response.message)
                    except Exception as exception:
                        successful = False
                        reason = str(exception)
                    for key, _, _ in current_changes:
                        operation_results.append((current_id, key, successful, reason))
                    finish_operation(operation_results)

                future.add_done_callback(completed_drone_command)
                continue

            future = client.set_parameters([
                Parameter(setting["parameter"], value=value)
                for _, setting, value in node_changes
            ])

            def completed_parameters(
                result_future, current_id=drone_id, current_changes=node_changes
            ):
                operation_results = []
                try:
                    parameter_results = result_future.result().results
                    for (key, _, _), result in zip(current_changes, parameter_results):
                        operation_results.append((
                            current_id, key, bool(result.successful), str(result.reason)
                        ))
                    if len(parameter_results) < len(current_changes):
                        for key, _, _ in current_changes[len(parameter_results):]:
                            operation_results.append((
                                current_id, key, False, "parameter service returned no result"
                            ))
                except Exception as exception:
                    operation_results = [
                        (current_id, key, False, str(exception))
                        for key, _, _ in current_changes
                    ]
                finish_operation(operation_results)

            future.add_done_callback(completed_parameters)

    def _dispatch_request(self, command: HttpCommand):
        if command.kind == "get_settings":
            self._get_settings(command)
        elif command.kind == "get_settings_profiles":
            self._get_settings_profiles(command)
        elif command.kind == "set_setting":
            self._set_setting(command)
        elif command.kind == "save_settings_profile":
            self._save_settings_profile(command)
        elif command.kind == "delete_settings_profile":
            self._delete_settings_profile(command)
        elif command.kind == "add_target":
            self._add_target(command)
        elif command.kind == "remove_target":
            self._remove_target(command)
        elif command.kind == "swarm_command":
            self._swarm_command(command)
        elif command.kind == "set_drone_speed":
            self._set_drone_speed(command)
        elif command.kind == "set_drone_lidar_range":
            self._set_drone_lidar_range(command)
        elif command.kind == "gimbal_command":
            self._gimbal_command(command)
        else:
            self._finish(command, False, "unknown dashboard command")

    def _add_target(self, command: HttpCommand):
        if not self._add_target_client.service_is_ready():
            self._finish(command, False, "/swarm/add_target is unavailable")
            return
        request = AddSwarmTarget.Request()
        request.target = PoseStamped()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x = float(command.payload["x"])
        request.target.pose.position.y = float(command.payload["y"])
        request.target.pose.position.z = float(command.payload["z"])
        request.target.pose.orientation.w = 1.0
        request.cruise_speed_m_s = float(command.payload.get("cruise_speed_m_s", 15.0))
        request.use_fixed_wing = bool(command.payload.get("use_fixed_wing", True))
        self._complete_service(
            command,
            self._add_target_client.call_async(request),
            lambda response: (response.accepted, response.message),
        )

    def _remove_target(self, command: HttpCommand):
        if not self._remove_target_client.service_is_ready():
            self._finish(command, False, "/swarm/remove_target is unavailable")
            return
        request = RemoveSwarmTarget.Request()
        request.target_id = int(command.payload["target_id"])
        self._complete_service(
            command,
            self._remove_target_client.call_async(request),
            lambda response: (response.removed, response.message),
        )

    def _swarm_command(self, command: HttpCommand):
        name = str(command.payload.get("command", "")).lower()
        if name not in COMMANDS:
            self._finish(command, False, f"unknown swarm command '{name}'")
            return
        if not self._command_client.service_is_ready():
            self._finish(command, False, "/swarm/command is unavailable")
            return
        request = SwarmCommand.Request()
        request.command = COMMANDS[name]
        request.drone_id = str(command.payload.get("drone_id", ""))
        request.takeoff_altitude_m = float(
            command.payload.get("altitude_m", self._takeoff_altitude_m)
        )
        request.takeoff_climb_speed_m_s = float(
            command.payload.get("climb_speed_m_s", self._takeoff_climb_speed_m_s)
        )
        self._complete_service(
            command,
            self._command_client.call_async(request),
            lambda response: (response.accepted, response.message),
        )

    def _set_drone_speed(self, command: HttpCommand):
        if not self._command_client.service_is_ready():
            self._finish(command, False, "/swarm/command is unavailable")
            return
        clear_override = bool(command.payload.get("clear", False))
        request = SwarmCommand.Request()
        request.command = (
            SwarmCommand.Request.CLEAR_DRONE_SPEED
            if clear_override
            else SwarmCommand.Request.SET_DRONE_SPEED
        )
        request.drone_id = str(command.payload.get("drone_id", ""))
        request.cruise_speed_m_s = (
            0.0 if clear_override else float(command.payload["cruise_speed_m_s"])
        )
        self._complete_service(
            command,
            self._command_client.call_async(request),
            lambda response: (response.accepted, response.message),
        )

    def _set_drone_lidar_range(self, command: HttpCommand):
        if not self._command_client.service_is_ready():
            self._finish(command, False, "/swarm/command is unavailable")
            return
        request = SwarmCommand.Request()
        request.command = SwarmCommand.Request.SET_DRONE_LIDAR_RANGE
        request.drone_id = str(command.payload.get("drone_id", ""))
        request.lidar_range_m = float(command.payload["lidar_range_m"])
        self._complete_service(
            command,
            self._command_client.call_async(request),
            lambda response: (response.accepted, response.message),
        )

    def _gimbal_command(self, command: HttpCommand):
        if not self._gimbal_command_client.service_is_ready():
            self._finish(command, False, "/swarm/gimbal_command is unavailable")
            return
        command_names = {
            "up": SwarmGimbalCommand.Request.COMMAND_UP,
            "down": SwarmGimbalCommand.Request.COMMAND_DOWN,
            "left": SwarmGimbalCommand.Request.COMMAND_LEFT,
            "right": SwarmGimbalCommand.Request.COMMAND_RIGHT,
            "home": SwarmGimbalCommand.Request.COMMAND_HOME,
            "stop": SwarmGimbalCommand.Request.COMMAND_STOP,
        }
        target_names = {
            "all": SwarmGimbalCommand.Request.TARGET_ALL,
            "drone": SwarmGimbalCommand.Request.TARGET_DRONE,
        }
        command_name = str(command.payload.get("command", "")).lower()
        target_name = str(command.payload.get("target_mode", "all")).lower()
        if command_name not in command_names:
            self._finish(command, False, f"unknown gimbal command '{command_name}'")
            return
        if target_name not in target_names:
            self._finish(command, False, f"unknown gimbal target '{target_name}'")
            return
        request = SwarmGimbalCommand.Request()
        request.target_mode = target_names[target_name]
        request.drone_id = str(command.payload.get("drone_id", ""))
        request.command = command_names[command_name]
        request.pressed = bool(command.payload.get("pressed", False))
        self._complete_service(
            command,
            self._gimbal_command_client.call_async(request),
            lambda response: (response.accepted, response.message),
        )

    def _complete_service(self, command: HttpCommand, future, parser: Callable):
        def completed(result_future):
            try:
                accepted, message = parser(result_future.result())
                self._finish(command, accepted, message)
            except Exception as exception:
                self._finish(command, False, f"service request failed: {exception}")

        future.add_done_callback(completed)

    @staticmethod
    def _finish(command: HttpCommand, accepted: bool, message: str):
        command.result = {"ok": bool(accepted), "message": str(message)}
        command.completed.set()

    @staticmethod
    def _finite(value):
        return float(value) if math.isfinite(value) else None

    @classmethod
    def _point_dict(cls, point):
        return {"x": float(point.x), "y": float(point.y), "z": float(point.z)}

    @classmethod
    def _vector_dict(cls, vector):
        return {"x": float(vector.x), "y": float(vector.y), "z": float(vector.z)}

    @staticmethod
    def _speed_m_s(velocity):
        return math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )

    @classmethod
    def _target_dict(cls, target):
        return {
            "target_id": int(target.target_id),
            "position": cls._point_dict(target.target.pose.position),
            "cruise_speed_m_s": float(target.cruise_speed_m_s),
            "use_fixed_wing": bool(target.use_fixed_wing),
        }

    @classmethod
    def _assignment_dict(cls, assignment):
        return {
            "target_id": int(assignment.target_id),
            "drone_id": assignment.drone_id,
            "position": cls._point_dict(assignment.target.pose.position),
            "cost": cls._finite(assignment.cost),
            "route_total_cost": cls._finite(assignment.route_total_cost),
            "distance_remaining_m": cls._finite(assignment.distance_remaining_m),
            "state": ASSIGNMENT_STATES.get(assignment.state, "unknown"),
            "message": assignment.message,
        }

    @classmethod
    def _drone_dict(cls, drone):
        return {
            "drone_id": drone.drone_id,
            "namespace": drone.drone_namespace,
            "registered": drone.registered,
            "connected": drone.connected,
            "localized": drone.localized,
            "navigation_ready": drone.navigation_ready,
            "operator_enabled": drone.operator_enabled,
            "available": drone.available,
            "busy": drone.busy,
            "armed": drone.armed,
            "offboard": drone.offboard,
            "has_lidar": drone.has_lidar,
            "has_gimbal": drone.has_gimbal,
            "supports_fixed_wing": drone.supports_fixed_wing,
            "supports_vtol": drone.supports_vtol,
            "has_speed_override": drone.has_speed_override,
            "speed_override_m_s": float(drone.speed_override_m_s),
            "lidar_range_m": float(drone.lidar_range_m),
            "battery_valid": bool(drone.battery_valid),
            "battery_remaining_pct": cls._finite(drone.battery_remaining_pct),
            "battery_time_remaining_s": cls._finite(drone.battery_time_remaining_s),
            "battery_power_w": cls._finite(drone.battery_power_w),
            "battery_capacity_wh": cls._finite(drone.battery_capacity_wh),
            "battery_remaining_energy_wh": cls._finite(drone.battery_remaining_energy_wh),
            "battery_state": int(drone.battery_state),
            "safety_excluded": bool(drone.safety_excluded),
            "return_home_active": bool(drone.return_home_active),
            "last_update_age_sec": cls._finite(drone.last_update_age_sec),
            "position": cls._point_dict(drone.position),
            "heading_valid": bool(drone.heading_valid),
            "heading_ned_rad": cls._finite(drone.heading_ned_rad),
        }

    def destroy_node(self):
        with self._recording_lock:
            active_recordings = list(self._server_video_recordings)
        for drone_id in active_recordings:
            try:
                self.stop_server_recording(drone_id)
            except (OSError, RuntimeError, ValueError) as exception:
                self.get_logger().warning(
                    f"Could not finalize {drone_id} recording during shutdown: {exception}"
                )
        self._http_server.shutdown()
        self._http_server.server_close()
        self._http_thread.join(timeout=2.0)
        return super().destroy_node()


class DashboardRequestHandler(BaseHTTPRequestHandler):
    """Serve static files and a deliberately small JSON API."""

    dashboard_web_root: Optional[Path] = None

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path == "/api/state":
            self._send_json(HTTPStatus.OK, self.server.dashboard.snapshot())
            return
        if path == "/api/settings":
            status, response = self.server.dashboard.submit("get_settings", {}, timeout_sec=4.0)
            self._send_json(status, response)
            return
        if path == "/api/settings/profiles":
            status, response = self.server.dashboard.submit(
                "get_settings_profiles", {}, timeout_sec=4.0
            )
            self._send_json(status, response)
            return
        if path == "/api/storage":
            dashboard = self.server.dashboard
            payload = {"ok": True, "preconfigured": dashboard._preconfigure_media_storage}
            if dashboard._preconfigure_media_storage:
                payload["photo_path"] = str(dashboard._photo_save_dir)
                payload["record_path"] = str(dashboard._record_save_dir)
            self._send_json(HTTPStatus.OK, payload)
            return
        if path.startswith("/api/camera/") and path.endswith(".jpg"):
            drone_id = path[len("/api/camera/") : -len(".jpg")]
            frame = self.server.dashboard.camera_frame(drone_id)
            if frame is None:
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"ok": False, "message": "camera frame unavailable or stale"},
                )
                return
            self._send_bytes(HTTPStatus.OK, "image/jpeg", frame, no_cache=True)
            return
        files = {
            "/": "index.html", "/app.js": "app.js",
            "/polling.js": "polling.js", "/styles.css": "styles.css",
        }
        filename = files.get(path)
        if filename is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_types = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}
        file_path = self.dashboard_web_root / filename
        self._send_bytes(
            HTTPStatus.OK,
            content_types[file_path.suffix],
            file_path.read_bytes(),
            no_cache=True,
        )

    def do_POST(self):
        path = unquote(urlparse(self.path).path)
        snapshot_prefix = "/api/camera/"
        snapshot_suffix = "/snapshot"
        if path.startswith(snapshot_prefix) and path.endswith(snapshot_suffix):
            drone_id = path[len(snapshot_prefix) : -len(snapshot_suffix)]
            try:
                saved_path = self.server.dashboard.save_camera_snapshot(drone_id)
            except (OSError, ValueError) as exception:
                self._send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"ok": False, "message": f"snapshot could not be saved: {exception}"},
                )
                return
            if saved_path is None:
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"ok": False, "message": "camera frame unavailable or stale"},
                )
                return
            relative_path = saved_path.relative_to(self.server.dashboard._photo_save_dir)
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": f"Snapshot saved: {relative_path}",
                    "path": str(saved_path),
                },
            )
            return
        recording_start_suffix = "/recording/start"
        recording_stop_suffix = "/recording/stop"
        if path.startswith(snapshot_prefix) and path.endswith(recording_start_suffix):
            drone_id = path[len(snapshot_prefix) : -len(recording_start_suffix)]
            try:
                self.server.dashboard.start_server_recording(drone_id)
            except (OSError, RuntimeError, ValueError) as exception:
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"ok": False, "message": f"recording could not start: {exception}"},
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "message": f"Recording started: {drone_id}"},
            )
            return
        if path.startswith(snapshot_prefix) and path.endswith(recording_stop_suffix):
            drone_id = path[len(snapshot_prefix) : -len(recording_stop_suffix)]
            try:
                saved_path = self.server.dashboard.stop_server_recording(drone_id)
            except (OSError, RuntimeError, ValueError) as exception:
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"ok": False, "message": f"recording could not stop: {exception}"},
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": f"Recording saved: {saved_path}",
                    "path": str(saved_path),
                },
            )
            return
        recording_suffix = "/recording"
        if path.startswith(snapshot_prefix) and path.endswith(recording_suffix):
            drone_id = path[len(snapshot_prefix) : -len(recording_suffix)]
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > MAX_RECORDING_BYTES:
                    raise ValueError("invalid recording size")
                content = self.rfile.read(length)
                if len(content) != length:
                    raise ValueError("incomplete recording upload")
                saved_path = self.server.dashboard.save_recording(drone_id, content)
            except (OSError, ValueError) as exception:
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {"ok": False, "message": f"recording could not be saved: {exception}"},
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": f"Recording saved: {saved_path}",
                    "path": str(saved_path),
                },
            )
            return
        if path == "/api/storage/pick":
            try:
                payload = self._read_json_body()
                selected_directory = self.server.dashboard.pick_media_directory(
                    payload.get("kind", "")
                )
            except (OSError, RuntimeError, TypeError, ValueError) as exception:
                self._send_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    {"ok": False, "message": f"folder picker failed: {exception}"},
                )
                return
            if selected_directory is None:
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "cancelled": True, "message": "Folder selection cancelled"},
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "cancelled": False,
                    "message": f"Storage directory ready: {selected_directory}",
                    "path": str(selected_directory),
                },
            )
            return
        if path == "/api/storage":
            try:
                payload = self._read_json_body()
                selected_directory = self.server.dashboard.configure_media_directory(
                    payload.get("kind", ""), payload.get("path", "")
                )
            except (OSError, TypeError, ValueError, json.JSONDecodeError) as exception:
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {"ok": False, "message": f"invalid storage directory: {exception}"},
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "message": f"Storage directory ready: {selected_directory}"},
            )
            return
        routes = {
            "/api/targets": "add_target",
            "/api/targets/remove": "remove_target",
            "/api/swarm-command": "swarm_command",
            "/api/drone/speed": "set_drone_speed",
            "/api/drone/lidar-range": "set_drone_lidar_range",
            "/api/gimbal-command": "gimbal_command",
            "/api/settings": "set_setting",
            "/api/settings/profiles/save": "save_settings_profile",
            "/api/settings/profiles/delete": "delete_settings_profile",
        }
        kind = routes.get(path)
        if kind is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            payload = self._read_json_body()
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            status, response = self.server.dashboard.submit(kind, payload)
            self._send_json(status, response)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exception:
            self._send_json(
                HTTPStatus.BAD_REQUEST,
                {"ok": False, "message": f"invalid request: {exception}"},
            )

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 65536:
            raise ValueError("invalid request size")
        payload = json.loads(self.rfile.read(length))
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def _send_json(self, status, payload):
        self._send_bytes(
            status,
            "application/json",
            json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            no_cache=True,
        )

    def _send_bytes(self, status, content_type, content, no_cache):
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            if no_cache:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError):
            # Browser navigation and request timeouts can close the connection.
            self.close_connection = True

    def log_message(self, format_string, *args):
        if args and str(args[1]).startswith("4"):
            self.server.dashboard.get_logger().warning(format_string % args)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
