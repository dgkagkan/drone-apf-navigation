import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import Vector3
from drone_interfaces.msg import SwarmAssignment, SwarmDroneState
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from drone_dashboard.dashboard_node import (
    COMMANDS,
    HttpCommand,
    RUNTIME_SETTING_DEFINITIONS,
    DashboardNode,
)


def test_decode_rgb_image_to_opencv_bgr():
    message = Image()
    message.height = 1
    message.width = 2
    message.encoding = "rgb8"
    message.step = 6
    message.data = [255, 0, 0, 0, 255, 0]

    image = DashboardNode._decode_image(message)

    assert image.shape == (1, 2, 3)
    assert image[0, 0].tolist() == [0, 0, 255]
    assert image[0, 1].tolist() == [0, 255, 0]


def test_non_finite_ros_values_become_json_null_values():
    assert DashboardNode._finite(12.5) == 12.5
    assert DashboardNode._finite(math.nan) is None
    assert DashboardNode._finite(math.inf) is None


def test_parameter_value_accepts_rclpy_parameter_objects():
    parameter = Parameter("camera_fps", Parameter.Type.DOUBLE, 30.0)

    assert DashboardNode._parameter_value(parameter) == 30.0


def test_speed_uses_all_three_velocity_axes():
    velocity = Vector3(x=3.0, y=4.0, z=12.0)

    assert DashboardNode._speed_m_s(velocity) == 13.0


def test_drone_state_exposes_battery_safety_to_dashboard():
    drone = SwarmDroneState()
    drone.drone_id = "drone_4"
    drone.battery_valid = True
    drone.battery_remaining_pct = 24.5
    drone.battery_time_remaining_s = 130.0
    drone.battery_power_w = 220.0
    drone.battery_capacity_wh = 250.0
    drone.battery_remaining_energy_wh = 61.25
    drone.heading_valid = True
    drone.heading_ned_rad = 1.25
    drone.battery_state = SwarmDroneState.BATTERY_STATE_RETURN_HOME
    drone.safety_excluded = True
    drone.return_home_active = True

    result = DashboardNode._drone_dict(drone)

    assert result["battery_remaining_pct"] == 24.5
    assert result["battery_time_remaining_s"] == 130.0
    assert result["battery_power_w"] == 220.0
    assert result["battery_capacity_wh"] == 250.0
    assert result["battery_remaining_energy_wh"] == 61.25
    assert result["heading_valid"] is True
    assert result["heading_ned_rad"] == 1.25
    assert result["battery_state"] == SwarmDroneState.BATTERY_STATE_RETURN_HOME
    assert result["safety_excluded"] is True
    assert result["return_home_active"] is True


def test_start_command_is_separate_from_calculate():
    assert COMMANDS["calculate"] != COMMANDS["start"]


def test_assignment_exposes_total_route_cost():
    assignment = SwarmAssignment()
    assignment.cost = 12.0
    assignment.route_total_cost = 48.0

    result = DashboardNode._assignment_dict(assignment)

    assert result["cost"] == 12.0
    assert result["route_total_cost"] == 48.0


def test_runtime_settings_batch_applies_every_dashboard_parameter():
    class Future:
        def __init__(self, result):
            self._result = result

        def add_done_callback(self, callback):
            callback(self)

        def result(self):
            return self._result

    class ParameterClient:
        def __init__(self, node_name):
            self.node_name = node_name
            self.calls = []

        def services_are_ready(self):
            return True

        def set_parameters(self, parameters):
            self.calls.append(parameters)
            return Future(SimpleNamespace(
                results=[SimpleNamespace(successful=True, reason="") for _ in parameters]
            ))

    class EnabledClient:
        def service_is_ready(self):
            return True

        def call_async(self, request):
            return Future(SimpleNamespace(success=True, accepted=True, message=""))

    class CommandClient(EnabledClient):
        def __init__(self):
            self.calls = []

        def call_async(self, request):
            self.calls.append(request)
            return super().call_async(request)

    node = DashboardNode.__new__(DashboardNode)
    node._runtime_setting_values = {}
    parameter_clients = {}
    node.set_parameters = lambda parameters: [
        SimpleNamespace(successful=True, reason="") for _ in parameters
    ]
    node._setting_targets = lambda target: [{"drone_id": "drone_1"}]
    node._parameter_client = lambda drone, node_name: parameter_clients.setdefault(
        node_name, ParameterClient(node_name)
    )
    node._apf_enabled_client = lambda drone: EnabledClient()
    command_client = CommandClient()
    node._command_client = command_client
    node._finish = lambda command, ok, message: (
        command.result.update(ok=ok, message=message), command.completed.set()
    )

    command = HttpCommand(
        "set_setting",
        {
            "target": "ALL",
            "changes": [
                {"key": setting["key"], "value": setting["default"]}
                for setting in RUNTIME_SETTING_DEFINITIONS
            ],
        },
    )
    DashboardNode._set_setting(node, command)

    assert command.completed.is_set()
    assert command.result["ok"] is True
    drone_settings = [
        setting for setting in RUNTIME_SETTING_DEFINITIONS
        if setting.get("scope") != "dashboard"
    ]
    dashboard_settings = [
        setting for setting in RUNTIME_SETTING_DEFINITIONS
        if setting.get("scope") == "dashboard"
    ]
    assert len(node._runtime_setting_values["drone_1"]) == len(drone_settings)
    assert set(node._runtime_setting_values["drone_1"]) == {
        setting["key"] for setting in drone_settings
    }
    assert set(node._runtime_setting_values["dashboard"]) == {
        setting["key"] for setting in dashboard_settings
    }
    assert set(parameter_clients) == {
        "apf_safety", "navigation_server", "lidar_processor", "px4_gateway"
    }
    assert len(command_client.calls) == 1
    assert command_client.calls[0].drone_id == "drone_1"
    assert command_client.calls[0].lidar_range_m == 70.0


def test_numeric_choice_accepts_browser_serialized_number():
    setting = next(
        setting for setting in RUNTIME_SETTING_DEFINITIONS
        if setting["key"] == "camera_fps"
    )

    assert DashboardNode._coerce_setting_value(setting, "60") == 60.0


def test_active_lidar_range_uses_coordinator_command_scope():
    setting = next(
        setting for setting in RUNTIME_SETTING_DEFINITIONS
        if setting["key"] == "active_lidar_apf_range"
    )

    assert setting["scope"] == "drone_command"
    assert DashboardNode._coerce_setting_value(setting, "120") == 120.0


def test_settings_profile_round_trip_is_atomic_and_validated(tmp_path):
    node = DashboardNode.__new__(DashboardNode)
    node._settings_profile_path = tmp_path / "runtime_profiles.json"
    node._settings_profiles_lock = threading.Lock()
    node._settings_profiles = {}

    command = HttpCommand(
        "save_settings_profile",
        {
            "name": "Inspection safe",
            "values": {
                "obstacle_influence_radius": 80,
                "camera_fps": "30",
            },
        },
    )
    DashboardNode._save_settings_profile(node, command)

    assert command.result["ok"] is True
    assert node._settings_profile_path.exists()

    loaded = DashboardNode.__new__(DashboardNode)
    loaded._settings_profile_path = node._settings_profile_path
    profiles = DashboardNode._load_settings_profiles(loaded)

    assert profiles["Inspection safe"]["values"] == {
        "camera_fps": 30.0,
        "obstacle_influence_radius": 80.0,
    }
