import argparse
import json
from pathlib import Path

import optuna

from .evaluator import NON_APF_FAILURES
from .optimizer import configure_sqlite_database, create_optuna_storage
from .seed_study import successful_unique_trials


def is_valid_repetition(trial: optuna.trial.FrozenTrial, result_path: Path) -> bool:
    if trial.state in {
        optuna.trial.TrialState.RUNNING,
        optuna.trial.TrialState.WAITING,
    }:
        return True
    if not result_path.is_file():
        return trial.state == optuna.trial.TrialState.COMPLETE

    result = json.loads(result_path.read_text(encoding='utf-8'))
    outcome = result.get('metrics', {}).get('outcome')
    return outcome not in NON_APF_FAILURES


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Queue repeated evaluations of the best successful APF trials.'
    )
    parser.add_argument('--source-directory', type=Path, required=True)
    parser.add_argument('--source-study', required=True)
    parser.add_argument('--target-directory', type=Path, required=True)
    parser.add_argument('--target-study', required=True)
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--repetitions', type=int, default=15)
    parser.add_argument('--summary-file', type=Path, required=True)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error('--count must be positive')
    if args.repetitions <= 0:
        parser.error('--repetitions must be positive')
    return args


def main() -> None:
    args = parse_arguments()
    source_database = (args.source_directory / 'study.db').resolve()
    target_database = (args.target_directory / 'study.db').resolve()
    if not source_database.is_file():
        raise FileNotFoundError(f'Source database does not exist: {source_database}')

    configure_sqlite_database(source_database)
    configure_sqlite_database(target_database)
    source_study = optuna.load_study(
        study_name=args.source_study,
        storage=create_optuna_storage(source_database),
    )
    target_study = optuna.create_study(
        study_name=args.target_study,
        storage=create_optuna_storage(target_database),
        direction='minimize',
        load_if_exists=True,
    )

    selected = successful_unique_trials(source_study, args.count)
    if not selected:
        raise RuntimeError('The source study has no successful completed trials')

    existing_repetitions = set()
    for trial in target_study.get_trials(deepcopy=False):
        result_path = args.target_directory / f'trial_{trial.number:05d}' / 'result.json'
        if not is_valid_repetition(trial, result_path):
            continue
        existing_repetitions.add((
            trial.user_attrs.get('seed_source_study'),
            trial.user_attrs.get('seed_source_trial'),
            trial.user_attrs.get('seed_repeat_index'),
        ))
    queued = 0
    for source_trial in selected:
        for repeat_index in range(args.repetitions):
            repetition_key = (args.source_study, source_trial.number, repeat_index)
            if repetition_key in existing_repetitions:
                continue
            target_study.enqueue_trial(
                source_trial.params,
                user_attrs={
                    'seed_source_study': args.source_study,
                    'seed_source_trial': source_trial.number,
                    'seed_source_score': source_trial.value,
                    'seed_repeat_index': repeat_index,
                },
                skip_if_exists=False,
            )
            existing_repetitions.add(repetition_key)
            queued += 1

    waiting_total = sum(
        trial.state == optuna.trial.TrialState.WAITING
        for trial in target_study.get_trials(deepcopy=False)
    )
    summary = {
        'source_study': args.source_study,
        'target_study': args.target_study,
        'count': len(selected),
        'repetitions': args.repetitions,
        'queued': queued,
        'waiting_total': waiting_total,
        'selected_trials': [
            {'trial': trial.number, 'score': trial.value}
            for trial in selected
        ],
    }
    args.summary_file.parent.mkdir(parents=True, exist_ok=True)
    args.summary_file.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    print(
        f'Queued {queued} repeated runs in {args.target_study}; '
        f'{waiting_total} waiting runs total.'
    )
    for trial in selected:
        print(f'  source trial {trial.number}: score={trial.value:.3f}')


if __name__ == '__main__':
    main()
