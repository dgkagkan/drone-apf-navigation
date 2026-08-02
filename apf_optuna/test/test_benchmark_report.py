from apf_optuna.benchmark_report import rank_candidates


def make_record(
    source_trial: int,
    source_score: float,
    outcome: str,
    score: float,
    oscillation: float = 2.0,
) -> dict:
    return {
        'source_trial': source_trial,
        'source_score': source_score,
        'score': score,
        'metrics': {
            'outcome': outcome,
            'elapsed_time': 100.0,
            'fw_attitude_oscillation_deg_per_s': oscillation,
            'minimum_geometric_clearance': 3.0,
        },
    }


def test_success_rate_is_ranked_before_single_run_score():
    records = [
        make_record(10, 100.0, 'success', 120.0),
        make_record(10, 100.0, 'success', 125.0),
        make_record(20, 90.0, 'success', 90.0),
        make_record(20, 90.0, 'collision', 1_100_000.0),
    ]

    ranking = rank_candidates(records)

    assert [candidate['source_trial'] for candidate in ranking] == [10, 20]
    assert ranking[0]['success_rate_percent'] == 100.0
    assert ranking[1]['success_rate_percent'] == 50.0


def test_consistent_scores_break_equal_success_rate_tie():
    records = [
        make_record(10, 100.0, 'success', 100.0),
        make_record(10, 100.0, 'success', 140.0),
        make_record(20, 110.0, 'success', 120.0),
        make_record(20, 110.0, 'success', 120.0),
    ]

    ranking = rank_candidates(records)

    assert ranking[0]['source_trial'] == 20
    assert ranking[0]['score_standard_deviation'] == 0.0


def test_failed_candidate_has_no_success_only_metrics():
    ranking = rank_candidates([
        make_record(10, 100.0, 'collision', 1_100_000.0),
    ])

    assert ranking[0]['success_rate_percent'] == 0.0
    assert ranking[0]['mean_fw_attitude_oscillation_deg_per_s'] is None


def test_infrastructure_failure_is_not_counted_as_an_attempt():
    ranking = rank_candidates([
        make_record(10, 100.0, 'success', 120.0),
        make_record(10, 100.0, 'startup_timeout', 3_000_000.0),
    ])

    assert ranking[0]['attempts'] == 1
    assert ranking[0]['successes'] == 1
    assert ranking[0]['success_rate_percent'] == 100.0
    assert ranking[0]['infrastructure_failures'] == 1
    assert ranking[0]['infrastructure_outcomes'] == {'startup_timeout': 1}


def test_mission_timeout_is_retried_without_reducing_success_rate():
    ranking = rank_candidates([
        make_record(10, 100.0, 'success', 120.0),
        make_record(10, 100.0, 'timeout', 1_000_000.0),
    ])

    assert ranking[0]['attempts'] == 1
    assert ranking[0]['success_rate_percent'] == 100.0
    assert ranking[0]['infrastructure_failures'] == 1
    assert ranking[0]['infrastructure_outcomes'] == {'timeout': 1}
