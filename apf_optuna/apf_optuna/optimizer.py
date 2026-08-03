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
import yaml

from .evaluator import NON_APF_FAILURES, score_trial, TrialEvaluator, TrialMetrics
from .geometry import ObstacleCourse


SQLITE_BUSY_TIMEOUT_SECONDS = 120.0
OPTUNA_PARAMETER_NAMES = (
    'obstacle_influence_radius',
    'fw_avoid_trigger_ratio',
    'fw_attractive_gain',
    'fw_repulsive_gain',
    'repulsive_distance_power',
    'fw_trail_half_width',
    'fw_max_avoid_angle_deg',
    'fw_max_avoid_pitch_deg',
    'vertical_escape_pitch_gain',
)


class NonApfTrialError(RuntimeError):
    pass


STARTUP_RETRY_OUTCOMES = frozenset(
    {
        'controller_exit',
        'no_telemetry',
        'simulator_exit',
        'stalled',
        'startup_timeout',
    }
)


def should_retry_startup(outcome: str, attempt_number: int, maximum_attempts: int) -> bool:
    return attempt_number < maximum_attempts and outcome in STARTUP_RETRY_OUTCOMES


def configure_sqlite_database(database_path: Path) -> None:
    database_path.parent.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(database_path, timeout=SQLITE_BUSY_TIMEOUT_SECONDS) as connection:
        connection.execute(f'PRAGMA busy_timeout = {int(SQLITE_BUSY_TIMEOUT_SECONDS * 1000)}')
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


def current_controller_parameters() -> dict[str, float]:
    config_path = Path(get_package_share_directory('drone_control')) / 'config' / 'controller.yaml'
    with config_path.open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    apf_parameters = config['apf_safety']['ros__parameters']
    influence_radius = float(apf_parameters['obstacle_influence_radius'])
    trigger_distance = float(apf_parameters['fw_avoid_trigger_dist'])
    if influence_radius <= 0.0 or not 0.0 < trigger_distance <= influence_radius:
        raise ValueError('controller.yaml contains an invalid APF trigger distance')

    parameters = {
        'obstacle_influence_radius': influence_radius,
        'fw_avoid_trigger_ratio': trigger_distance / influence_radius,
    }
    for name in OPTUNA_PARAMETER_NAMES[2:]:
        parameters[name] = float(apf_parameters[name])
    return parameters


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

    def attempt_environment(
        self,
        trial: optuna.Trial,
        attempt_number: int,
        attempt_directory: Path,
    ) -> dict[str, str]:
        environment = os.environ.copy()
        if self.args.worker_id is not None:
            base_partition = environment.get('GZ_PARTITION', 'apf_optuna')
            environment['GZ_PARTITION'] = (
                f'{base_partition}_trial_{trial.number}_attempt_{attempt_number}'
            )
        environment['ROS_LOG_DIR'] = str(attempt_directory / 'ros_logs')
        environment['PYTHONUNBUFFERED'] = '1'
        return environment

    def simulation_command(self, attempt_directory: Path) -> list[str]:
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
                    f'px4_work_dir:={attempt_directory / "px4"}',
                ]
            )
        return sim_command

    def controller_command(self, parameters: dict[str, float]) -> list[str]:
        controller_command = [
            'ros2',
            'launch',
            'drone_bringup',
            'automated_controller.launch.py',
            f'mission_profile:={self.args.mission_profile}',
            f'goal_x:={self.args.goal_x}',
            f'goal_y:={self.args.goal_y}',
            f'cruise_altitude:={self.args.cruise_altitude}',
            f'goal_2_x:={self.args.goal_2_x}',
            f'goal_2_y:={self.args.goal_2_y}',
            f'goal_2_altitude:={self.args.goal_2_altitude}',
            f'goal_3_x:={self.args.goal_3_x}',
            f'goal_3_y:={self.args.goal_3_y}',
            f'goal_3_altitude:={self.args.goal_3_altitude}',
            f'intermediate_goal_tolerance:={self.args.intermediate_goal_tolerance}',
        ]
        if self.args.worker_id is not None:
            controller_command.append(f'target_system:={self.args.worker_id + 1}')
        controller_command.extend(f'{name}:={value}' for name, value in parameters.items())
        return controller_command

    def execute_attempt(
        self,
        trial: optuna.Trial,
        parameters: dict[str, float],
        attempt_number: int,
        attempt_directory: Path,
    ) -> tuple[TrialMetrics, dict]:
        environment = self.attempt_environment(trial, attempt_number, attempt_directory)
        evaluator = TrialEvaluator(
            self.course,
            self.args.vehicle_horizontal_radius,
            self.args.vehicle_vertical_radius,
            self.args.takeoff_timeout,
            self.args.stall_min_progress,
        )
        sim_process = None
        controller_process = None
        wall_started = time.monotonic()
        fallback_outcome = 'timeout'

        try:
            sim_process = ManagedProcess(
                self.simulation_command(attempt_directory),
                attempt_directory / 'simulation.log',
                environment,
            )
            time.sleep(self.args.sim_start_delay)
            if sim_process.process.poll() is not None:
                fallback_outcome = 'simulator_exit'
            else:
                controller_process = ManagedProcess(
                    self.controller_command(parameters),
                    attempt_directory / 'controller.log',
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
            return evaluator.metrics(fallback_outcome), evaluator.startup_diagnostics()
        finally:
            if controller_process is not None:
                controller_process.stop()
            if sim_process is not None:
                sim_process.stop()
            evaluator.destroy_node()
            time.sleep(self.args.reset_delay)

    def run_trial(self, trial: optuna.Trial, parameters: dict[str, float]) -> float:
        trial_directory = self.output_directory / f'trial_{trial.number:05d}'
        trial_directory.mkdir(parents=True, exist_ok=True)
        attempt_history = []
        metrics = TrialMetrics()

        for attempt_number in range(1, self.args.startup_attempts + 1):
            attempt_directory = trial_directory / f'attempt_{attempt_number:02d}'
            attempt_directory.mkdir(parents=True, exist_ok=True)
            metrics, readiness = self.execute_attempt(
                trial,
                parameters,
                attempt_number,
                attempt_directory,
            )
            attempt_summary = {
                'attempt': attempt_number,
                'outcome': metrics.outcome,
                'readiness': readiness,
                'metrics': metrics.as_dict(),
            }
            (attempt_directory / 'attempt_summary.json').write_text(
                json.dumps(attempt_summary, indent=2, sort_keys=True) + '\n',
                encoding='utf-8',
            )
            attempt_history.append(
                {
                    'attempt': attempt_number,
                    'outcome': metrics.outcome,
                    'readiness': readiness,
                    'log_directory': attempt_directory.name,
                }
            )

            if not should_retry_startup(
                metrics.outcome,
                attempt_number,
                self.args.startup_attempts,
            ):
                break
            print(
                f'Trial {trial.number} startup attempt {attempt_number}/'
                f'{self.args.startup_attempts} failed at {readiness["stage"]} '
                f'({metrics.outcome}); restarting cleanly.',
                flush=True,
            )
            time.sleep(self.args.startup_retry_delay)

        score = score_trial(
            metrics,
            self.args.minimum_clearance,
            self.args.stability_weight,
        )
        record = {
            'trial': trial.number,
            'score': score,
            'parameters': parameters,
            'mission': {
                'profile': self.args.mission_profile,
                'goals': self.mission_goals(),
                'intermediate_goal_tolerance': self.args.intermediate_goal_tolerance,
            },
            'metrics': metrics.as_dict(),
            'scoring': {
                'minimum_clearance': self.args.minimum_clearance,
                'stability_weight': self.args.stability_weight,
            },
            'startup': {
                'attempts_used': len(attempt_history),
                'maximum_attempts': self.args.startup_attempts,
                'history': attempt_history,
            },
        }
        (trial_directory / 'result.json').write_text(
            json.dumps(record, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        for name, value in metrics.as_dict().items():
            trial.set_user_attr(name, value)
        trial.set_user_attr('startup_attempts_used', len(attempt_history))
        trial.set_user_attr('startup_retry_count', max(0, len(attempt_history) - 1))
        if metrics.outcome in NON_APF_FAILURES:
            raise NonApfTrialError(
                f'{metrics.outcome} is unrelated to APF parameters and must be ignored'
            )
        return score

    def objective(self, trial: optuna.Trial) -> float:
        parameters = self.suggest_parameters(trial)
        return self.run_trial(trial, parameters)

    def mission_goals(self) -> list[dict[str, float]]:
        goals = [
            {
                'x': self.args.goal_x,
                'y': self.args.goal_y,
                'altitude': self.args.cruise_altitude,
            }
        ]
        if self.args.mission_profile == 'three_goal':
            goals.extend(
                [
                    {
                        'x': self.args.goal_2_x,
                        'y': self.args.goal_2_y,
                        'altitude': self.args.goal_2_altitude,
                    },
                    {
                        'x': self.args.goal_3_x,
                        'y': self.args.goal_3_y,
                        'altitude': self.args.goal_3_altitude,
                    },
                ]
            )
        return goals


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Optimize VTOL 3D APF parameters in Gazebo.')
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--study-name', default='vtol_3d_apf')
    parser.add_argument('--output-directory', default='optuna_results')
    parser.add_argument(
        '--mission-profile',
        choices=('single', 'three_goal'),
        default='single',
    )
    parser.add_argument('--goal-x', type=float, default=700.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--cruise-altitude', type=float, default=15.0)
    parser.add_argument('--goal-2-x', type=float, default=0.0)
    parser.add_argument('--goal-2-y', type=float, default=0.0)
    parser.add_argument('--goal-2-altitude', type=float, default=15.0)
    parser.add_argument('--goal-3-x', type=float, default=750.0)
    parser.add_argument('--goal-3-y', type=float, default=15.0)
    parser.add_argument('--goal-3-altitude', type=float, default=15.0)
    parser.add_argument('--intermediate-goal-tolerance', type=float, default=25.0)
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
    parser.add_argument('--reset-delay', type=float, default=6.0)
    parser.add_argument('--startup-timeout', type=float, default=90.0)
    parser.add_argument('--startup-attempts', type=int, default=3)
    parser.add_argument('--startup-retry-delay', type=float, default=8.0)
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
        '--enqueue-current-controller-parameters',
        action='store_true',
        help='Run the next trial with the APF values currently stored in controller.yaml.',
    )
    parser.add_argument(
        '--initialize-only',
        action='store_true',
        help='Create and configure the shared Optuna study, then exit.',
    )
    args = parser.parse_args()
    if args.stability_weight < 0.0:
        parser.error('--stability-weight must be non-negative')
    if args.intermediate_goal_tolerance <= 0.0:
        parser.error('--intermediate-goal-tolerance must be positive')
    if args.startup_attempts <= 0:
        parser.error('--startup-attempts must be positive')
    if args.startup_retry_delay < 0.0:
        parser.error('--startup-retry-delay must be non-negative')
    if args.reset_delay < 0.0:
        parser.error('--reset-delay must be non-negative')
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
        if args.enqueue_current_controller_parameters:
            parameters = current_controller_parameters()
            study.enqueue_trial(
                parameters,
                user_attrs={'parameter_source': 'controller.yaml'},
            )
            print('Queued current controller.yaml APF parameters for the next trial.')
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
