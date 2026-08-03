from dataclasses import asdict, dataclass
import math
from typing import Optional

from drone_interfaces.msg import AutomatedMissionTelemetry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .geometry import ObstacleCourse


NON_APF_FAILURES = frozenset(
    {
        'controller_exit',
        'no_telemetry',
        'simulator_exit',
        'stalled',
        'startup_timeout',
        'timeout',
    }
)

STABILITY_BENCHMARK_OUTCOMES = frozenset({'collision', 'success'})


def startup_stage(
    *,
    sim_time_advanced: bool,
    vehicle_state_received: bool,
    attitude_ready: bool,
    obstacles_ready: bool,
    mission_state: str,
    armed: bool,
    offboard: bool,
    flight_started: bool,
) -> str:
    if not sim_time_advanced:
        return 'waiting_for_sim_time'
    if flight_started:
        return 'flight_started'
    if not vehicle_state_received:
        return 'waiting_for_vehicle_state'
    if not attitude_ready:
        return 'waiting_for_attitude'
    if not obstacles_ready:
        return 'waiting_for_obstacles'
    if mission_state == 'priming_offboard' and not (armed and offboard):
        return 'waiting_for_arm_offboard'
    return mission_state or 'unknown'


class _AxisOscillationTracker:
    def __init__(self, minimum_excursion_degrees: float) -> None:
        self.minimum_excursion = math.radians(minimum_excursion_degrees)
        self._previous_angle: Optional[float] = None
        self._direction = 0
        self._excursion = 0.0
        self.reversal_excursion = 0.0

    @staticmethod
    def _angle_difference(current: float, reference: float) -> float:
        return math.atan2(math.sin(current - reference), math.cos(current - reference))

    def pause(self) -> None:
        self._previous_angle = None
        self._direction = 0
        self._excursion = 0.0

    def add_sample(self, angle: float) -> None:
        if self._previous_angle is None:
            self._previous_angle = angle
            return

        delta = self._angle_difference(angle, self._previous_angle)
        self._previous_angle = angle
        if abs(delta) < math.radians(0.05):
            return

        direction = 1 if delta > 0.0 else -1
        if self._direction == 0 or direction == self._direction:
            self._direction = direction
            self._excursion += abs(delta)
            return

        if self._excursion >= self.minimum_excursion:
            self.reversal_excursion += self._excursion
        self._direction = direction
        self._excursion = abs(delta)


class AttitudeOscillationTracker:
    def __init__(self, minimum_excursion_degrees: float = 1.0) -> None:
        if minimum_excursion_degrees <= 0.0:
            raise ValueError('minimum_excursion_degrees must be positive')
        self._roll = _AxisOscillationTracker(minimum_excursion_degrees)
        self._pitch = _AxisOscillationTracker(minimum_excursion_degrees)
        self._previous_time: Optional[float] = None
        self._measurement_duration = 0.0

    def pause(self) -> None:
        self._roll.pause()
        self._pitch.pause()
        self._previous_time = None

    def add_sample(self, elapsed_time: float, roll: float, pitch: float) -> None:
        if not all(math.isfinite(value) for value in (elapsed_time, roll, pitch)):
            self.pause()
            return
        if self._previous_time is None:
            self._roll.add_sample(roll)
            self._pitch.add_sample(pitch)
            self._previous_time = elapsed_time
            return

        dt = elapsed_time - self._previous_time
        if dt <= 0.0 or dt > 1.0:
            self.pause()
            self.add_sample(elapsed_time, roll, pitch)
            return

        self._roll.add_sample(roll)
        self._pitch.add_sample(pitch)
        self._measurement_duration += dt
        self._previous_time = elapsed_time

    def degrees_per_second(self) -> float:
        if self._measurement_duration <= 0.0:
            return 0.0
        reversal_excursion = self._roll.reversal_excursion + self._pitch.reversal_excursion
        return math.degrees(reversal_excursion) / self._measurement_duration


@dataclass
class TrialMetrics:
    outcome: str = 'no_telemetry'
    elapsed_time: float = 0.0
    goal_distance: float = -1.0
    initial_goal_distance: float = -1.0
    closest_goal_distance: float = -1.0
    progress_distance: float = 0.0
    mission_remaining_distance: float = -1.0
    path_length: float = 0.0
    max_cross_track_error: float = 0.0
    minimum_lidar_distance: float = -1.0
    minimum_geometric_clearance: float = -1.0
    avoidance_activations: int = 0
    repulsive_force_variation: float = 0.0
    fw_attitude_oscillation_deg_per_s: float = 0.0
    samples: int = 0
    startup_stage: str = 'waiting_for_telemetry'
    vehicle_state_received: bool = False
    attitude_ready: bool = False
    obstacles_ready: bool = False
    armed: bool = False
    offboard: bool = False
    sim_time_advanced: bool = False

    def as_dict(self) -> dict:
        return asdict(self)


class TrialEvaluator(Node):
    def __init__(
        self,
        course: ObstacleCourse,
        vehicle_horizontal_radius: float,
        vehicle_vertical_radius: float,
        takeoff_timeout: float = 45.0,
        stall_min_progress: float = 20.0,
    ) -> None:
        super().__init__('apf_trial_evaluator')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscription = self.create_subscription(
            AutomatedMissionTelemetry,
            '/automated_controller/telemetry',
            self._on_telemetry,
            qos,
        )
        self.latest: Optional[AutomatedMissionTelemetry] = None
        self.outcome: Optional[str] = None
        self.started = False
        self.max_altitude = 0.0
        self.force_variation = 0.0
        self.attitude_oscillation = AttitudeOscillationTracker()
        self._previous_force: Optional[tuple[float, float, float]] = None
        self.samples = 0
        self.course = course
        self.vehicle_horizontal_radius = vehicle_horizontal_radius
        self.vehicle_vertical_radius = vehicle_vertical_radius
        self.takeoff_timeout = takeoff_timeout
        self.stall_min_progress = stall_min_progress
        self.minimum_geometric_clearance = math.inf
        self.initial_goal_distance = math.inf
        self.closest_goal_distance = math.inf
        self.max_mission_progress_distance = 0.0
        self._first_stamp_ns: Optional[int] = None
        self._last_stamp_ns: Optional[int] = None

    def _on_telemetry(self, msg: AutomatedMissionTelemetry) -> None:
        self.latest = msg
        self.samples += 1
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._first_stamp_ns is None:
            self._first_stamp_ns = stamp_ns
        self._last_stamp_ns = stamp_ns
        self.started = self.started or msg.result == 'running'
        self.max_altitude = max(self.max_altitude, msg.altitude)
        if msg.goal_distance >= 0.0:
            if not math.isfinite(self.initial_goal_distance):
                self.initial_goal_distance = msg.goal_distance
            self.closest_goal_distance = min(self.closest_goal_distance, msg.goal_distance)
        if msg.mission_progress_distance >= 0.0:
            self.max_mission_progress_distance = max(
                self.max_mission_progress_distance,
                msg.mission_progress_distance,
            )
        geometric_clearance = self.course.clearance(
            msg.east,
            msg.north,
            msg.altitude,
            self.vehicle_horizontal_radius,
            self.vehicle_vertical_radius,
        )
        self.minimum_geometric_clearance = min(
            self.minimum_geometric_clearance,
            geometric_clearance,
        )

        force = (
            msg.repulsive_force_enu.x,
            msg.repulsive_force_enu.y,
            msg.repulsive_force_enu.z,
        )
        if self._previous_force is not None:
            self.force_variation += math.sqrt(
                sum(
                    (current - previous) ** 2
                    for current, previous in zip(force, self._previous_force)
                )
            )
        self._previous_force = force

        if msg.fixed_wing and msg.state == 'cruise_fw':
            self.attitude_oscillation.add_sample(
                msg.elapsed_time,
                msg.roll_rad,
                msg.pitch_rad,
            )
        else:
            self.attitude_oscillation.pause()

        if msg.result == 'success':
            self.outcome = 'success'
            return
        if msg.result == 'landing_miss':
            self.outcome = 'landing_miss'
            return

        takeoff_stalled = (
            self.started
            and msg.state == 'takeoff'
            and msg.elapsed_time >= self.takeoff_timeout
            and self.max_altitude < 5.0
        )
        if takeoff_stalled:
            self.outcome = 'stalled'
            return

        active_flight_states = {
            'takeoff',
            'transition_to_fw',
            'cruise_fw',
            'transition_to_mc',
            'approach_mc',
        }
        unexpected_disarm = self.started and not msg.armed and msg.state in active_flight_states
        ground_impact = (
            self.max_altitude > 5.0 and msg.altitude < 0.5 and msg.state in active_flight_states
        )
        if geometric_clearance <= 0.0:
            self.outcome = 'collision'
        elif unexpected_disarm or ground_impact:
            self.outcome = 'crash'

    def finished(self) -> bool:
        return self.outcome is not None

    def startup_diagnostics(self) -> dict:
        if self.latest is None:
            return {
                'stage': 'waiting_for_telemetry',
                'telemetry_received': False,
                'vehicle_state_received': False,
                'attitude_ready': False,
                'obstacles_ready': False,
                'armed': False,
                'offboard': False,
                'sim_time_advanced': False,
                'samples': 0,
            }

        msg = self.latest
        sim_time_advanced = (
            self._first_stamp_ns is not None
            and self._last_stamp_ns is not None
            and self._last_stamp_ns - self._first_stamp_ns >= 500_000_000
        )
        stage = startup_stage(
            sim_time_advanced=sim_time_advanced,
            vehicle_state_received=msg.vehicle_state_received,
            attitude_ready=msg.attitude_ready,
            obstacles_ready=msg.obstacles_ready,
            mission_state=msg.state,
            armed=msg.armed,
            offboard=msg.offboard,
            flight_started=self.started,
        )

        return {
            'stage': stage,
            'telemetry_received': True,
            'vehicle_state_received': bool(msg.vehicle_state_received),
            'attitude_ready': bool(msg.attitude_ready),
            'obstacles_ready': bool(msg.obstacles_ready),
            'armed': bool(msg.armed),
            'offboard': bool(msg.offboard),
            'sim_time_advanced': sim_time_advanced,
            'samples': self.samples,
        }

    def metrics(self, fallback_outcome: str) -> TrialMetrics:
        if self.latest is None:
            return TrialMetrics(outcome=fallback_outcome)
        msg = self.latest
        startup = self.startup_diagnostics()
        initial_goal_distance = (
            self.initial_goal_distance if math.isfinite(self.initial_goal_distance) else -1.0
        )
        closest_goal_distance = (
            self.closest_goal_distance if math.isfinite(self.closest_goal_distance) else -1.0
        )
        progress_distance = self.max_mission_progress_distance
        if progress_distance <= 0.0:
            progress_distance = (
                max(0.0, initial_goal_distance - closest_goal_distance)
                if initial_goal_distance >= 0.0 and closest_goal_distance >= 0.0
                else 0.0
            )
        outcome = self.outcome or fallback_outcome
        if outcome == 'timeout' and progress_distance < self.stall_min_progress:
            outcome = 'stalled'
        return TrialMetrics(
            outcome=outcome,
            elapsed_time=msg.elapsed_time,
            goal_distance=msg.goal_distance,
            initial_goal_distance=initial_goal_distance,
            closest_goal_distance=closest_goal_distance,
            progress_distance=progress_distance,
            mission_remaining_distance=msg.mission_remaining_distance,
            path_length=msg.path_length,
            max_cross_track_error=msg.max_cross_track_error,
            minimum_lidar_distance=msg.minimum_lidar_distance,
            minimum_geometric_clearance=(
                self.minimum_geometric_clearance
                if math.isfinite(self.minimum_geometric_clearance)
                else -1.0
            ),
            avoidance_activations=msg.avoidance_activations,
            repulsive_force_variation=self.force_variation,
            fw_attitude_oscillation_deg_per_s=(self.attitude_oscillation.degrees_per_second()),
            samples=self.samples,
            startup_stage=startup['stage'],
            vehicle_state_received=startup['vehicle_state_received'],
            attitude_ready=startup['attitude_ready'],
            obstacles_ready=startup['obstacles_ready'],
            armed=startup['armed'],
            offboard=startup['offboard'],
            sim_time_advanced=startup['sim_time_advanced'],
        )


def score_trial(
    metrics: TrialMetrics,
    minimum_clearance: float = 2.0,
    stability_weight: float = 1.0,
) -> float:
    remaining_distance = max(
        0.0,
        metrics.mission_remaining_distance
        if metrics.mission_remaining_distance >= 0.0
        else (
            metrics.closest_goal_distance
            if metrics.closest_goal_distance >= 0.0
            else metrics.goal_distance
        ),
    )
    if metrics.outcome == 'stalled':
        return 2_500_000.0 + 1_000.0 * remaining_distance
    if metrics.outcome in NON_APF_FAILURES:
        return 3_000_000.0
    if metrics.outcome == 'crash':
        return 1_200_000.0 + 1_000.0 * remaining_distance
    if metrics.outcome == 'collision':
        return 1_100_000.0 + 1_000.0 * remaining_distance
    if metrics.outcome != 'success':
        return 1_000_000.0 + 1_000.0 * remaining_distance

    clearance_penalty = 0.0
    measured_clearance = metrics.minimum_geometric_clearance
    if 0.0 < measured_clearance < minimum_clearance:
        clearance_penalty = 2_000.0 * (minimum_clearance - measured_clearance) ** 2

    return (
        metrics.elapsed_time
        + 0.05 * metrics.path_length
        + 0.5 * metrics.max_cross_track_error
        + clearance_penalty
        + 0.01 * metrics.repulsive_force_variation
        + max(0.0, stability_weight) * metrics.fw_attitude_oscillation_deg_per_s
    )
