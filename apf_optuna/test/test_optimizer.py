from argparse import Namespace
import os
import sqlite3

from apf_optuna.optimizer import (
    configure_sqlite_database,
    configure_worker_environment,
    current_controller_parameters,
    should_retry_startup,
)


def test_parallel_worker_environment_is_isolated(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    monkeypatch.delenv('GZ_PARTITION', raising=False)
    args = Namespace(
        worker_id=2,
        base_domain_id=40,
        base_agent_port=9000,
        run_id='stress/test',
    )

    configure_worker_environment(args)

    assert os.environ['ROS_DOMAIN_ID'] == '42'
    assert os.environ['GZ_PARTITION'] == 'apf_optuna_stress_test_worker_2'


def test_sqlite_database_uses_wal_mode(tmp_path):
    database_path = tmp_path / 'study.db'

    configure_sqlite_database(database_path)

    with sqlite3.connect(database_path) as connection:
        journal_mode = connection.execute('PRAGMA journal_mode').fetchone()[0]
    assert journal_mode == 'wal'


def test_current_controller_parameters_match_trigger_ratio():
    parameters = current_controller_parameters()

    assert set(parameters) == {
        'obstacle_influence_radius',
        'fw_avoid_trigger_ratio',
        'fw_attractive_gain',
        'fw_repulsive_gain',
        'repulsive_distance_power',
        'fw_trail_half_width',
        'fw_max_avoid_angle_deg',
        'fw_max_avoid_pitch_deg',
        'vertical_escape_pitch_gain',
    }
    assert 0.0 < parameters['fw_avoid_trigger_ratio'] <= 1.0


def test_transient_startup_failure_retries_before_limit():
    assert should_retry_startup('startup_timeout', 1, 3)
    assert should_retry_startup('stalled', 2, 3)


def test_startup_failure_stops_retrying_at_limit():
    assert not should_retry_startup('startup_timeout', 3, 3)


def test_flight_outcomes_are_never_internal_startup_retries():
    assert not should_retry_startup('success', 1, 3)
    assert not should_retry_startup('collision', 1, 3)
    assert not should_retry_startup('crash', 1, 3)
