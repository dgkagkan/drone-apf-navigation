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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


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

    def _on_camera(self, drone_id: str, message: Image):
        now = time.monotonic()
        with self._camera_lock:
            if drone_id in self._camera_encoding:
                return
            if now - self._last_camera_encode.get(drone_id, 0.0) < 1.0 / self._camera_rate_hz:
                return
            self._camera_encoding.add(drone_id)
            self._last_camera_encode[drone_id] = now
        try:
            image = self._decode_image(message)
            DashboardNode._record_camera_frame(self, drone_id, image, now)
            encoded, jpeg = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
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

    def _dispatch_request(self, command: HttpCommand):
        if command.kind == "add_target":
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
