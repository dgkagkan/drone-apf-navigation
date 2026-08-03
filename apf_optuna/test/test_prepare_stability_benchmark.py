import json

import optuna

from apf_optuna.prepare_stability_benchmark import is_valid_repetition


def make_trial(state: optuna.trial.TrialState) -> optuna.trial.FrozenTrial:
    value = 1.0 if state == optuna.trial.TrialState.COMPLETE else None
    return optuna.trial.create_trial(state=state, value=value)


def test_waiting_repetition_is_reserved(tmp_path):
    result_path = tmp_path / 'missing.json'

    assert is_valid_repetition(make_trial(optuna.trial.TrialState.WAITING), result_path)


def test_startup_timeout_is_retried(tmp_path):
    result_path = tmp_path / 'result.json'
    result_path.write_text(
        json.dumps({'metrics': {'outcome': 'startup_timeout'}}),
        encoding='utf-8',
    )

    assert not is_valid_repetition(make_trial(optuna.trial.TrialState.FAIL), result_path)


def test_mission_timeout_is_retried(tmp_path):
    result_path = tmp_path / 'result.json'
    result_path.write_text(
        json.dumps({'metrics': {'outcome': 'timeout'}}),
        encoding='utf-8',
    )

    assert not is_valid_repetition(make_trial(optuna.trial.TrialState.COMPLETE), result_path)


def test_collision_counts_as_a_valid_repetition(tmp_path):
    result_path = tmp_path / 'result.json'
    result_path.write_text(
        json.dumps({'metrics': {'outcome': 'collision'}}),
        encoding='utf-8',
    )

    assert is_valid_repetition(make_trial(optuna.trial.TrialState.COMPLETE), result_path)


def test_completed_trial_without_result_is_retried(tmp_path):
    assert not is_valid_repetition(
        make_trial(optuna.trial.TrialState.COMPLETE),
        tmp_path / 'missing.json',
    )


def test_crash_is_retried_instead_of_counting_as_collision(tmp_path):
    result_path = tmp_path / 'result.json'
    result_path.write_text(
        json.dumps({'metrics': {'outcome': 'crash'}}),
        encoding='utf-8',
    )

    assert not is_valid_repetition(make_trial(optuna.trial.TrialState.COMPLETE), result_path)


def test_landing_miss_is_retried(tmp_path):
    result_path = tmp_path / 'result.json'
    result_path.write_text(
        json.dumps({'metrics': {'outcome': 'landing_miss'}}),
        encoding='utf-8',
    )

    assert not is_valid_repetition(make_trial(optuna.trial.TrialState.COMPLETE), result_path)
