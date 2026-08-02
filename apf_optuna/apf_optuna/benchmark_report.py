import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import statistics

import optuna

from .evaluator import NON_APF_FAILURES
from .optimizer import configure_sqlite_database, create_optuna_storage


def _mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _median(values: list[float]) -> float | None:
    return statistics.median(values) if values else None


def _standard_deviation(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def summarize_candidate(source_trial: int, records: list[dict]) -> dict:
    infrastructure_failures = [
        record
        for record in records
        if record['metrics']['outcome'] in NON_APF_FAILURES
    ]
    valid_records = [
        record
        for record in records
        if record['metrics']['outcome'] not in NON_APF_FAILURES
    ]
    successful = [
        record for record in valid_records if record['metrics']['outcome'] == 'success'
    ]
    scores = [float(record['score']) for record in valid_records]
    success_scores = [float(record['score']) for record in successful]
    oscillations = [
        float(record['metrics']['fw_attitude_oscillation_deg_per_s'])
        for record in successful
    ]
    elapsed_times = [float(record['metrics']['elapsed_time']) for record in successful]
    clearances = [
        float(record['metrics']['minimum_geometric_clearance'])
        for record in successful
    ]
    outcomes = Counter(record['metrics']['outcome'] for record in valid_records)
    infrastructure_outcomes = Counter(
        record['metrics']['outcome'] for record in infrastructure_failures
    )
    attempts = len(valid_records)

    return {
        'source_trial': source_trial,
        'source_score': float(records[0]['source_score']),
        'attempts': attempts,
        'successes': len(successful),
        'success_rate_percent': 100.0 * len(successful) / attempts if attempts else 0.0,
        'infrastructure_failures': len(infrastructure_failures),
        'mean_score': _mean(scores),
        'median_score': _median(scores),
        'score_standard_deviation': _standard_deviation(scores),
        'mean_success_score': _mean(success_scores),
        'mean_elapsed_time': _mean(elapsed_times),
        'mean_fw_attitude_oscillation_deg_per_s': _mean(oscillations),
        'mean_minimum_geometric_clearance': _mean(clearances),
        'outcomes': dict(sorted(outcomes.items())),
        'infrastructure_outcomes': dict(sorted(infrastructure_outcomes.items())),
    }


def load_benchmark_records(
    output_directory: Path,
    study: optuna.Study,
) -> tuple[list[dict], int]:
    records = []
    pending = 0
    for trial in study.get_trials(deepcopy=False):
        source_trial = trial.user_attrs.get('seed_source_trial')
        if source_trial is None:
            continue
        result_path = output_directory / f'trial_{trial.number:05d}' / 'result.json'
        if not result_path.is_file():
            if trial.state in {
                optuna.trial.TrialState.RUNNING,
                optuna.trial.TrialState.WAITING,
            }:
                pending += 1
            continue

        record = json.loads(result_path.read_text(encoding='utf-8'))
        record['source_trial'] = int(source_trial)
        record['source_score'] = float(trial.user_attrs['seed_source_score'])
        record['repeat_index'] = trial.user_attrs.get('seed_repeat_index')
        records.append(record)
    return records, pending


def rank_candidates(records: list[dict]) -> list[dict]:
    grouped: dict[int, list[dict]] = {}
    for record in records:
        grouped.setdefault(record['source_trial'], []).append(record)

    summaries = [
        summarize_candidate(source_trial, candidate_records)
        for source_trial, candidate_records in grouped.items()
    ]
    summaries.sort(
        key=lambda summary: (
            -summary['success_rate_percent'],
            summary['mean_score'] if summary['mean_score'] is not None else float('inf'),
            summary['score_standard_deviation']
            if summary['score_standard_deviation'] is not None
            else float('inf'),
            summary['source_score'],
        )
    )
    for rank, summary in enumerate(summaries, start=1):
        summary['rank'] = rank
    return summaries


def write_csv(path: Path, summaries: list[dict]) -> None:
    fieldnames = [
        'rank',
        'source_trial',
        'source_score',
        'attempts',
        'successes',
        'success_rate_percent',
        'infrastructure_failures',
        'mean_score',
        'median_score',
        'score_standard_deviation',
        'mean_success_score',
        'mean_elapsed_time',
        'mean_fw_attitude_oscillation_deg_per_s',
        'mean_minimum_geometric_clearance',
        'outcomes',
        'infrastructure_outcomes',
    ]
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for summary in summaries:
            row = dict(summary)
            row['outcomes'] = json.dumps(row['outcomes'], sort_keys=True)
            row['infrastructure_outcomes'] = json.dumps(
                row['infrastructure_outcomes'], sort_keys=True
            )
            writer.writerow(row)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Rank repeated APF evaluations by success rate and consistency.'
    )
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--study-name', required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    output_directory = args.output_directory.resolve()
    database_path = output_directory / 'study.db'
    if not database_path.is_file():
        raise FileNotFoundError(f'Benchmark database does not exist: {database_path}')

    configure_sqlite_database(database_path)
    study = optuna.load_study(
        study_name=args.study_name,
        storage=create_optuna_storage(database_path),
    )
    records, pending = load_benchmark_records(output_directory, study)
    if not records:
        raise RuntimeError('The benchmark does not have any completed result files')

    summaries = rank_candidates(records)
    infrastructure_failures = sum(
        summary['infrastructure_failures'] for summary in summaries
    )
    report = {
        'study_name': args.study_name,
        'stored_results': len(records),
        'valid_completed_runs': len(records) - infrastructure_failures,
        'infrastructure_failures': infrastructure_failures,
        'pending_runs': pending,
        'ranking': summaries,
    }
    summary_path = output_directory / 'stability_summary.json'
    csv_path = output_directory / 'stability_ranking.csv'
    summary_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    write_csv(csv_path, summaries)

    print('Rank  Trial  Success    Infra    Mean score    Score std    Oscillation')
    for summary in summaries:
        oscillation = summary['mean_fw_attitude_oscillation_deg_per_s']
        oscillation_text = 'n/a' if oscillation is None else f'{oscillation:.2f}'
        print(
            f"{summary['rank']:>4}  {summary['source_trial']:>5}  "
            f"{summary['successes']:>2}/{summary['attempts']:<2} "
            f"({summary['success_rate_percent']:>5.1f}%)  "
            f"{summary['infrastructure_failures']:>5}  "
            f"{summary['mean_score'] if summary['mean_score'] is not None else float('nan'):>12.2f}  "
            f"{summary['score_standard_deviation'] if summary['score_standard_deviation'] is not None else float('nan'):>11.2f}  "
            f"{oscillation_text:>11}"
        )
    print(f'Reports: {csv_path} and {summary_path}')


if __name__ == '__main__':
    main()
