#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

TRIALS_PER_WORKER="${1:-1}"
OUTPUT_DIRECTORY="${2:-$REPO_ROOT/optuna_results/apf_three_goal}"
STUDY_NAME="${APF_STUDY_NAME:-apf_three_goal}"
RUN_ID="${APF_RUN_ID:-apf_three_goal}"
BASE_DOMAIN_ID="${APF_BASE_DOMAIN_ID:-40}"
BASE_AGENT_PORT="${APF_BASE_AGENT_PORT:-9000}"
STARTUP_TIMEOUT="${APF_STARTUP_TIMEOUT:-150}"
TRIAL_TIMEOUT="${APF_TRIAL_TIMEOUT:-420}"
WORKER_START_STAGGER="${APF_WORKER_START_STAGGER:-12}"
STABILITY_WEIGHT="${APF_STABILITY_WEIGHT:-1.0}"

if ! [[ "$TRIALS_PER_WORKER" =~ ^[1-9][0-9]*$ ]]; then
    echo "TRIALS_PER_WORKER must be a positive integer." >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$REPO_ROOT/.venv/bin/activate"

set -u

# Keep high-rate ROS 2 and Gazebo traffic on this computer. Without the
# loopback whitelist, DDS can use Wi-Fi, Ethernet, Tailscale, or Docker.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset CYCLONEDDS_URI
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTDDS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_local_agent.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1

mkdir -p "$OUTPUT_DIRECTORY"
pids=()

stop_workers() {
    trap - INT TERM
    echo
    echo "Stopping parallel workers..."
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    exit 130
}

trap stop_workers INT TERM

start_worker() {
    local worker_id="$1"
    local worker_log="$OUTPUT_DIRECTORY/worker_${worker_id}.log"

    setsid python -m apf_optuna.optimizer \
        --trials "$TRIALS_PER_WORKER" \
        --study-name "$STUDY_NAME" \
        --output-directory "$OUTPUT_DIRECTORY" \
        --seed "$((42 + worker_id))" \
        --worker-id "$worker_id" \
        --base-domain-id "$BASE_DOMAIN_ID" \
        --base-agent-port "$BASE_AGENT_PORT" \
        --run-id "$RUN_ID" \
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
        --startup-timeout "$STARTUP_TIMEOUT" \
        --trial-timeout "$TRIAL_TIMEOUT" \
        --stability-weight "$STABILITY_WEIGHT" \
        --headless \
        >"$worker_log" 2>&1 &
    pids+=("$!")
    echo "Worker $worker_id started: PID ${pids[-1]}, log $worker_log"
}

echo "Starting four isolated headless PX4/Gazebo workers."
echo "Trials: $TRIALS_PER_WORKER per worker, $((4 * TRIALS_PER_WORKER)) total"
echo "Mission: (750, 0, 15) -> (0, 0, 15) -> (750, 15, 15)"
echo "Study: $STUDY_NAME"
echo "Results: $OUTPUT_DIRECTORY"

echo "Initializing shared Optuna study..."
if ! python -m apf_optuna.optimizer \
    --study-name "$STUDY_NAME" \
    --output-directory "$OUTPUT_DIRECTORY" \
    --initialize-only; then
    echo "Failed to initialize the shared Optuna study." >&2
    exit 1
fi

start_worker 0
sleep "$WORKER_START_STAGGER"
start_worker 1
sleep "$WORKER_START_STAGGER"
start_worker 2
sleep "$WORKER_START_STAGGER"
start_worker 3

while true; do
    running=0
    for pid in "${pids[@]}"; do
        kill -0 "$pid" 2>/dev/null && running=$((running + 1))
    done
    [[ "$running" -eq 0 ]] && break

    completed="$(find "$OUTPUT_DIRECTORY" -name result.json -type f | wc -l)"
    memory="$(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
    load="$(awk '{print $1 "," $2 "," $3}' /proc/loadavg)"
    echo "[$(date +%H:%M:%S)] workers=$running completed=$completed memory=$memory load=$load"
    sleep 10
done

failed=0
for index in "${!pids[@]}"; do
    if wait "${pids[$index]}"; then
        echo "Worker $index completed successfully."
    else
        echo "Worker $index failed; inspect $OUTPUT_DIRECTORY/worker_${index}.log" >&2
        failed=1
    fi
done

completed="$(find "$OUTPUT_DIRECTORY" -name result.json -type f | wc -l)"
echo "Finished: $completed results stored in $OUTPUT_DIRECTORY"
if command -v jq >/dev/null 2>&1; then
    find "$OUTPUT_DIRECTORY" -name result.json -type f -print0 \
        | xargs -0 -r -n1 jq -r '.metrics.outcome' \
        | sort | uniq -c
fi
exit "$failed"
