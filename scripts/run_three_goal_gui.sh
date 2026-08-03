#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

TRIALS="${1:-1}"
OUTPUT_DIRECTORY="${2:-$REPO_ROOT/optuna_results/apf_three_goal}"
STUDY_NAME="${APF_STUDY_NAME:-apf_three_goal}"
TRIAL_TIMEOUT="${APF_TRIAL_TIMEOUT:-420}"

if ! [[ "$TRIALS" =~ ^[1-9][0-9]*$ ]]; then
    echo "TRIALS must be a positive integer." >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$REPO_ROOT/.venv/bin/activate"

set -u

# The simulation is local, so DDS and Gazebo must not publish LiDAR traffic on
# Wi-Fi, Ethernet, Tailscale, or Docker interfaces.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_local_agent.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1

echo "Starting visible three-goal Optuna trial."
echo "Mission: (750, 0, 15) -> (0, 0, 15) -> (750, 15, 15)"
echo "Study: $STUDY_NAME"
echo "Results: $OUTPUT_DIRECTORY"

python -m apf_optuna.optimizer \
    --trials "$TRIALS" \
    --study-name "$STUDY_NAME" \
    --output-directory "$OUTPUT_DIRECTORY" \
    --mission-profile three_goal \
    --goal-x 750.0 \
    --goal-y 0.0 \
    --cruise-altitude 15.0 \
    --goal-2-x 0.0 \
    --goal-2-y 0.0 \
    --goal-2-altitude 15.0 \
    --goal-3-x 750.0 \
    --goal-3-y 15.0 \
    --goal-3-altitude 15.0 \
    --intermediate-goal-tolerance 25.0 \
    --enqueue-current-controller-parameters \
    --startup-timeout 150 \
    --trial-timeout "$TRIAL_TIMEOUT"
