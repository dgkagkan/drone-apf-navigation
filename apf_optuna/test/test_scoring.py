import math

from apf_optuna.evaluator import (
    AttitudeOscillationTracker,
    NON_APF_FAILURES,
    score_trial,
    startup_stage,
    TrialMetrics,
)


def test_timeout_is_ignored_while_collision_is_scored():
    success = TrialMetrics(
        outcome='success',
        elapsed_time=200.0,
        goal_distance=0.0,
        path_length=800.0,
        max_cross_track_error=50.0,
        minimum_geometric_clearance=0.2,
    )
    timeout = TrialMetrics(outcome='timeout', goal_distance=0.0)
    collision = TrialMetrics(outcome='collision', goal_distance=0.0)

    assert score_trial(success) < score_trial(collision)
    assert score_trial(collision) < score_trial(timeout)


def test_infrastructure_failure_is_never_selected_as_best_failure():
    collision = TrialMetrics(outcome='collision', goal_distance=600.0)
    startup_timeout = TrialMetrics(outcome='startup_timeout', goal_distance=-1.0)

    assert score_trial(collision) < score_trial(startup_timeout)


def test_stalled_vehicle_is_not_rewarded_for_avoiding_collision():
    stalled = TrialMetrics(outcome='stalled', goal_distance=650.0)
    collision_after_progress = TrialMetrics(outcome='collision', goal_distance=550.0)

    assert score_trial(collision_after_progress) < score_trial(stalled)


def test_failure_score_uses_closest_distance_to_goal():
    collision = TrialMetrics(
        outcome='collision',
        goal_distance=400.0,
        closest_goal_distance=300.0,
    )

    assert score_trial(collision) == 1_400_000.0


def test_multi_goal_failure_score_uses_remaining_mission_distance():
    collision = TrialMetrics(
        outcome='collision',
        goal_distance=100.0,
        closest_goal_distance=20.0,
        mission_remaining_distance=900.0,
    )

    assert score_trial(collision) == 2_000_000.0


def test_non_apf_failures_are_excluded_from_parameter_learning():
    assert NON_APF_FAILURES == {
        'controller_exit',
        'no_telemetry',
        'simulator_exit',
        'stalled',
        'startup_timeout',
        'timeout',
    }


def test_low_clearance_is_penalized():
    safe = TrialMetrics(outcome='success', minimum_geometric_clearance=3.0)
    close = TrialMetrics(outcome='success', minimum_geometric_clearance=1.0)

    assert score_trial(safe) < score_trial(close)


def test_attitude_oscillation_is_penalized_for_successful_trials():
    stable = TrialMetrics(outcome='success', fw_attitude_oscillation_deg_per_s=1.0)
    oscillating = TrialMetrics(outcome='success', fw_attitude_oscillation_deg_per_s=6.0)

    assert score_trial(stable) < score_trial(oscillating)
    assert score_trial(oscillating) - score_trial(stable) == 5.0


def test_landing_miss_is_not_scored_as_success():
    success = TrialMetrics(outcome='success', closest_goal_distance=0.5)
    landing_miss = TrialMetrics(outcome='landing_miss', closest_goal_distance=0.5)

    assert score_trial(success) < score_trial(landing_miss)
    assert score_trial(landing_miss) == 1_000_500.0


def test_attitude_tracker_ignores_constant_bank_angle():
    tracker = AttitudeOscillationTracker()

    for sample in range(100):
        tracker.add_sample(sample * 0.1, math.radians(20.0), math.radians(3.0))

    assert tracker.degrees_per_second() == 0.0


def test_attitude_tracker_ignores_one_direction_attitude_change():
    tracker = AttitudeOscillationTracker()

    for sample in range(100):
        tracker.add_sample(sample * 0.1, math.radians(sample * 0.2), 0.0)

    assert tracker.degrees_per_second() == 0.0


def test_attitude_tracker_detects_back_and_forth_motion():
    tracker = AttitudeOscillationTracker()

    for sample in range(100):
        elapsed_time = sample * 0.1
        roll = math.radians(10.0) * math.sin(2.0 * math.pi * elapsed_time)
        tracker.add_sample(elapsed_time, roll, 0.0)

    assert tracker.degrees_per_second() > 20.0


def test_attitude_tracker_handles_roll_angle_wraparound():
    tracker = AttitudeOscillationTracker()
    tracker.add_sample(0.0, math.radians(179.0), 0.0)
    tracker.add_sample(0.1, math.radians(-179.0), 0.0)

    assert tracker.degrees_per_second() == 0.0


def test_startup_stage_reports_first_missing_readiness_signal():
    common = {
        'sim_time_advanced': True,
        'vehicle_state_received': True,
        'attitude_ready': True,
        'obstacles_ready': True,
        'mission_state': 'waiting_for_fcu',
        'armed': False,
        'offboard': False,
        'flight_started': False,
    }

    assert startup_stage(**(common | {'vehicle_state_received': False})) == (
        'waiting_for_vehicle_state'
    )
    assert startup_stage(**(common | {'obstacles_ready': False})) == 'waiting_for_obstacles'
    assert startup_stage(**(common | {'mission_state': 'priming_offboard'})) == (
        'waiting_for_arm_offboard'
    )
    assert (
        startup_stage(**(common | {'flight_started': True, 'obstacles_ready': False}))
        == 'flight_started'
    )
