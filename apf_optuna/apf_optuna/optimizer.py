import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time
from typing import TextIO

from ament_index_python.packages import get_package_share_directory
import optuna
import rclpy

from .evaluator import NON_APF_FAILURES, score_trial, TrialEvaluator
from .geometry import ObstacleCourse


SQLITE_BUSY_TIMEOUT_SECONDS = 120.0


class NonApfTrialError(RuntimeError):
    pass


def configure_sqlite_database(database_path: Path) -> None:
    database_path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(database_path, timeout=SQLITE_BUSY_TIMEOUT_SECONDS) as connection:
        connection.execute(
            f'PRAGMA busy_timeout = {int(SQLITE_BUSY_TIMEOUT_SECONDS * 1000)}'
        )
        journal_mode = connection.execute('PRAGMA journal_mode').fetchone()[0]
        if journal_mode.lower() != 'wal':
            journal_mode = connection.execute('PRAGMA journal_mode=WAL').fetchone()[0]
        if journal_mode.lower() != 'wal':
            raise RuntimeError(f'Failed to enable SQLite WAL mode: {journal_mode}')
        connection.execute('PRAGMA synchronous=NORMAL')


def create_optuna_storage(database_path: Path) -> optuna.storages.RDBStorage:
    return optuna.storages.RDBStorage(
        url=f'sqlite:///{database_path}',
        engine_kwargs={
            'connect_args': {'timeout': SQLITE_BUSY_TIMEOUT_SECONDS},
            'pool_pre_ping': True,
        },
    )


class ManagedProcess:
    def __init__(self, command: list[str], log_path: Path, environment: dict[str, str]) -> None:
        self._log: TextIO = log_path.open('w', encoding='utf-8')
        self.process = subprocess.Popen(
            command,
            env=environment,
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )

    def stop(self, grace_seconds: float = 8.0) -> None:
        try:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=grace_seconds)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    try:
                        self.process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(self.process.pid, signal.SIGKILL)
                        self.process.wait(timeout=3.0)
        except ProcessLookupError:
            pass
        finally:
            self._log.close()


class OptunaMissionRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.output_directory = Path(args.output_directory).resolve()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        course_path = (
            Path(get_package_share_directory('drone_bringup')) / 'worlds' / 'optuna_course.sdf'
        )
        self.course = ObstacleCourse.from_sdf(course_path)

    def suggest_parameters(self, trial: optuna.Trial) -> dict[str, float]:
        influence_radius = trial.suggest_float('obstacle_influence_radius', 45.0, 78.0)
        trigger_ratio = trial.suggest_float('fw_avoid_trigger_ratio', 0.70, 1.0)
        parameters = {
            'fw_attractive_gain': trial.suggest_float('fw_attractive_gain', 0.2, 5.0, log=True),
            'fw_repulsive_gain': trial.suggest_float('fw_repulsive_gain', 2.0, 250.0, log=True),
            'repulsive_distance_power': trial.suggest_float('repulsive_distance_power', 0.5, 2.5),
            'obstacle_influence_radius': influence_radius,
            'fw_avoid_trigger_dist': influence_radius * trigger_ratio,
            'fw_trail_half_width': trial.suggest_float('fw_trail_half_width', 4.0, 15.0),
            'fw_max_avoid_angle_deg': trial.suggest_float('fw_max_avoid_angle_deg', 15.0, 50.0),
            'fw_max_avoid_pitch_deg': trial.suggest_float('fw_max_avoid_pitch_deg', 4.0, 14.0),
            'vertical_escape_pitch_gain': trial.suggest_float(
                'vertical_escape_pitch_gain', 1.0, 3.0
            ),
        }
        trial.set_user_attr('fw_avoid_trigger_dist', parameters['fw_avoid_trigger_dist'])
        return parameters

    def run_trial(self, trial: optuna.Trial, parameters: dict[str, float]) -> float:
        trial_directory = self.output_directory / f'trial_{trial.number:05d}'
        trial_directory.mkdir(parents=True, exist_ok=True)
        environment = os.environ.copy()
        if self.args.worker_id is not None:
            base_partition = environment.get('GZ_PARTITION', 'apf_optuna')
            environment['GZ_PARTITION'] = f'{base_partition}_trial_{trial.number}'
        environment['ROS_LOG_DIR'] = str(trial_directory / 'ros_logs')
        environment['PYTHONUNBUFFERED'] = '1'

        sim_command = [
            'ros2',
            'launch',
            'drone_bringup',
            'sim.launch.py',
            'world:=optuna_course',
            f'headless:={1 if self.args.headless else 0}',
            'px4_terminal:=inline',
            'use_rviz:=false',
        ]
        if self.args.worker_id is not None:
            sim_command.extend(
                [
                    f'parallel_worker_id:={self.args.worker_id}',
                    f'agent_port:={self.args.base_agent_port + self.args.worker_id}',
                    f'px4_work_dir:={trial_directory / "px4"}',
                ]
            )
        controller_command = [
            'ros2',
            'launch',
            'drone_bringup',
            'automated_controller.launch.py',
            f'goal_x:={self.args.goal_x}',
            f'goal_y:={self.args.goal_y}',
            f'cruise_altitude:={self.args.cruise_altitude}',
        ]
        if self.args.worker_id is not None:
            controller_command.append(f'target_system:={self.args.worker_id + 1}')
        controller_command.extend(f'{name}:={value}' for name, value in parameters.items())

        evaluator = TrialEvaluator(
            self.course,
            self.args.vehicle_horizontal_radius,
            self.args.vehicle_vertical_radius,
            self.args.takeoff_timeout,
            self.args.stall_min_progress,
        )
        sim_process = ManagedProcess(sim_command, trial_directory / 'simulation.log', environment)
        controller_process = None
        wall_started = time.monotonic()
        fallback_outcome = 'timeout'

        try:
            time.sleep(self.args.sim_start_delay)
            if sim_process.process.poll() is not None:
                fallback_outcome = 'simulator_exit'
            else:
                controller_process = ManagedProcess(
                    controller_command,
                    trial_directory / 'controller.log',
                    environment,
                )

            while controller_process is not None and fallback_outcome == 'timeout':
                rclpy.spin_once(evaluator, timeout_sec=0.1)
                wall_elapsed = time.monotonic() - wall_started
                if evaluator.finished():
                    fallback_outcome = evaluator.outcome or 'unknown'
                    break
                if sim_process.process.poll() is not None:
                    fallback_outcome = 'simulator_exit'
                    break
                if controller_process.process.poll() is not None:
                    fallback_outcome = 'controller_exit'
                    break
                if not evaluator.started and wall_elapsed >= self.args.startup_timeout:
                    fallback_outcome = 'startup_timeout'
                    break
                if wall_elapsed >= self.args.trial_timeout:
                    break
        finally:
            if controller_process is not None:
                controller_process.stop()
            sim_process.stop()
            evaluator.destroy_node()
            time.sleep(self.args.reset_delay)

        metrics = evaluator.metrics(fallback_outcome)
        score = score_trial(
            metrics,
            self.args.minimum_clearance,
            self.args.stability_weight,
        )
        record = {
            'trial': trial.number,
            'score': score,
            'parameters': parameters,
            'metrics': metrics.as_dict(),
            'scoring': {
                'minimum_clearance': self.args.minimum_clearance,
                'stability_weight': self.args.stability_weight,
            },
        }
        (trial_directory / 'result.json').write_text(
            json.dumps(record, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        for name, value in metrics.as_dict().items():
            trial.set_user_attr(name, value)
        if metrics.outcome in NON_APF_FAILURES:
            raise NonApfTrialError(
                f'{metrics.outcome} is unrelated to APF parameters and must be ignored'
            )
        return score

    def objective(self, trial: optuna.Trial) -> float:
        parameters = self.suggest_parameters(trial)
        return self.run_trial(trial, parameters)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Optimize VTOL 3D APF parameters in Gazebo.')
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--study-name', default='vtol_3d_apf')
    parser.add_argument('--output-directory', default='optuna_results')
    parser.add_argument('--goal-x', type=float, default=700.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--cruise-altitude', type=float, default=15.0)
    parser.add_argument('--minimum-clearance', type=float, default=2.0)
    parser.add_argument(
        '--stability-weight',
        type=float,
        default=1.0,
        help='Score penalty per degree/second of FW roll/pitch reversal.',
    )
    parser.add_argument('--vehicle-horizontal-radius', type=float, default=1.1)
    parser.add_argument('--vehicle-vertical-radius', type=float, default=0.3)
    parser.add_argument('--sim-start-delay', type=float, default=5.0)
    parser.add_argument('--reset-delay', type=float, default=3.0)
    parser.add_argument('--startup-timeout', type=float, default=90.0)
    parser.add_argument('--trial-timeout', type=float, default=240.0)
    parser.add_argument('--takeoff-timeout', type=float, default=45.0)
    parser.add_argument('--stall-min-progress', type=float, default=20.0)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--worker-id',
        type=int,
        help='Enable isolated parallel mode with this zero-based worker ID.',
    )
    parser.add_argument('--base-domain-id', type=int, default=40)
    parser.add_argument('--base-agent-port', type=int, default=9000)
    parser.add_argument(
        '--run-id',
        default='',
        help='Shared identifier used to isolate this parallel Gazebo run.',
    )
    parser.add_argument(
        '--headless',
        action='store_true',
        help='Run Gazebo without its GUI. The GUI is shown by default.',
    )
    parser.add_argument(
        '--initialize-only',
        action='store_true',
        help='Create and configure the shared Optuna study, then exit.',
    )
    args = parser.parse_args()
    if args.stability_weight < 0.0:
        parser.error('--stability-weight must be non-negative')
    return args


def configure_worker_environment(args: argparse.Namespace) -> None:
    if args.worker_id is None:
        return
    if args.worker_id < 0:
        raise ValueError('--worker-id must be non-negative')

    domain_id = args.base_domain_id + args.worker_id
    agent_port = args.base_agent_port + args.worker_id
    if not 0 <= domain_id <= 232:
        raise ValueError('worker ROS_DOMAIN_ID must be between 0 and 232')
    if not 1 <= agent_port <= 65535:
        raise ValueError('worker Micro-XRCE-DDS port must be between 1 and 65535')

    run_id = ''.join(character if character.isalnum() else '_' for character in args.run_id)
    run_id = run_id.strip('_') or str(os.getpid())
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = f'apf_optuna_{run_id}_worker_{args.worker_id}'
    print(
        f'Worker {args.worker_id}: ROS domain {domain_id}, agent UDP {agent_port}, '
        f'Gazebo partition {os.environ["GZ_PARTITION"]}'
    )


def main() -> None:
    args = parse_arguments()
    configure_worker_environment(args)
    output_directory = Path(args.output_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    database_path = output_directory / 'study.db'
    configure_sqlite_database(database_path)
    storage = create_optuna_storage(database_path)
    sampler = optuna.samplers.TPESampler(seed=args.seed)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        sampler=sampler,
        direction='minimize',
        load_if_exists=True,
    )
    if args.initialize_only:
        print(f'Optuna study ready: {args.study_name} ({database_path})')
        return

    rclpy.init()
    try:
        runner = OptunaMissionRunner(args)
        study.optimize(
            runner.objective,
            n_trials=args.trials,
            n_jobs=1,
            catch=(NonApfTrialError,),
        )
    except KeyboardInterrupt:
        print('Optimization interrupted; completed trials remain stored in SQLite.')
        return
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    completed_trials = [
        trial
        for trial in study.get_trials(deepcopy=False)
        if trial.state == optuna.trial.TrialState.COMPLETE
    ]
    if not completed_trials:
        print('No valid APF trial completed; infrastructure failures were ignored.')
        return

    print(f'Best score: {study.best_value:.3f}')
    print(json.dumps(study.best_params, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
