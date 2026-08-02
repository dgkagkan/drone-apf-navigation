import argparse
import json
from pathlib import Path

import optuna

from .optimizer import configure_sqlite_database, create_optuna_storage


def successful_unique_trials(
    study: optuna.Study,
    count: int,
) -> list[optuna.trial.FrozenTrial]:
    candidates = [
        trial
        for trial in study.get_trials(deepcopy=False)
        if trial.state == optuna.trial.TrialState.COMPLETE
        and trial.value is not None
        and trial.user_attrs.get('outcome') == 'success'
    ]
    candidates.sort(key=lambda trial: trial.value)

    selected = []
    seen_parameters = set()
    for trial in candidates:
        parameter_key = json.dumps(trial.params, sort_keys=True)
        if parameter_key in seen_parameters:
            continue
        seen_parameters.add(parameter_key)
        selected.append(trial)
        if len(selected) == count:
            break
    return selected


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Re-evaluate the best successful trials in a new Optuna study.'
    )
    parser.add_argument('--source-directory', type=Path, required=True)
    parser.add_argument('--source-study', required=True)
    parser.add_argument('--target-directory', type=Path, required=True)
    parser.add_argument('--target-study', required=True)
    parser.add_argument('--count', type=int, default=10)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error('--count must be positive')
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

    waiting_before = sum(
        trial.state == optuna.trial.TrialState.WAITING
        for trial in target_study.get_trials(deepcopy=False)
    )
    for trial in selected:
        target_study.enqueue_trial(
            trial.params,
            user_attrs={
                'seed_source_study': args.source_study,
                'seed_source_trial': trial.number,
                'seed_source_score': trial.value,
            },
            skip_if_exists=True,
        )
    waiting_after = sum(
        trial.state == optuna.trial.TrialState.WAITING
        for trial in target_study.get_trials(deepcopy=False)
    )

    print(
        f'Queued {waiting_after - waiting_before} trial(s) in {args.target_study}; '
        f'{waiting_after} waiting trial(s) total.'
    )
    for trial in selected:
        print(f'  source trial {trial.number}: score={trial.value:.3f}')


if __name__ == '__main__':
    main()
