# APF Optuna

The optimizer launches a fresh PX4/Gazebo simulation for every trial, runs the
automated 3D APF controller, evaluates telemetry against the collision boxes in
`optuna_course.sdf`, and stores the study in SQLite. Gazebo opens with its GUI by
default, and Optuna chooses every trial's parameter values automatically.

## Run

Close any existing PX4, Gazebo, controller, or mission-manager process first.

```bash
cd /home/dimitris-gkagkanakis/ros2_work_ws/src/drone-apf-navigation
source /opt/ros/jazzy/setup.bash
source /home/dimitris-gkagkanakis/ros2_work_ws/install/setup.bash
source .venv/bin/activate

python -m apf_optuna.optimizer --trials 5
```

For a long unattended run without the Gazebo window:

```bash
python -m apf_optuna.optimizer --trials 100 --headless
```

To run four isolated headless workers against one shared Optuna study:

```bash
./scripts/run_4_headless.sh 1
```

The argument is the number of trials per worker, so `1` runs four trials in
total and `25` runs one hundred. The script prints CPU, memory, load average,
and completed-trial counts every ten seconds. Press `Ctrl+C` once to stop all
workers and their simulations cleanly.

Results are written to `optuna_results/` by default. Each trial has separate
simulation and controller logs plus a machine-readable `result.json`. No fixed
baseline is inserted: the seeded Optuna sampler chooses all APF values from the
configured search ranges, including attractive and repulsive gains.

## Dashboard

```bash
optuna-dashboard sqlite:///optuna_results/study.db
```

Open `http://127.0.0.1:8080` while the study is running or after it finishes.

## Replay a trial with Gazebo GUI

Replay a stored trial with the exact APF parameters from its `result.json`:

```bash
./scripts/replay_trial_gui.sh 4 optuna_results/apf_1000_v2
```

The first argument is the trial number. Press `Ctrl+C` to stop the controller,
PX4, and Gazebo together.
