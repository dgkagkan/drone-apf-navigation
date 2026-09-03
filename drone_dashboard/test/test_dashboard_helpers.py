import math

from geometry_msgs.msg import Vector3
from drone_interfaces.msg import SwarmAssignment, SwarmDroneState
from sensor_msgs.msg import Image

from drone_dashboard.dashboard_node import COMMANDS, DashboardNode


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
    drone.battery_state = SwarmDroneState.BATTERY_STATE_RETURN_HOME
    drone.safety_excluded = True
    drone.return_home_active = True

    result = DashboardNode._drone_dict(drone)

    assert result["battery_remaining_pct"] == 24.5
    assert result["battery_time_remaining_s"] == 130.0
    assert result["battery_power_w"] == 220.0
    assert result["battery_capacity_wh"] == 250.0
    assert result["battery_remaining_energy_wh"] == 61.25
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
