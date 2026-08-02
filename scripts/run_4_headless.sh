#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

TRIALS_PER_WORKER="${1:-1}"
RUN_ID="${APF_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIRECTORY="${2:-$REPO_ROOT/optuna_results/stress4_$RUN_ID}"
STUDY_NAME="${APF_STUDY_NAME:-apf_stress4_$RUN_ID}"
BASE_DOMAIN_ID="${APF_BASE_DOMAIN_ID:-40}"
BASE_AGENT_PORT="${APF_BASE_AGENT_PORT:-9000}"
STARTUP_TIMEOUT="${APF_STARTUP_TIMEOUT:-150}"
WORKER_START_STAGGER="${APF_WORKER_START_STAGGER:-12}"
STABILITY_WEIGHT="${APF_STABILITY_WEIGHT:-1.0}"
GOAL_X="${APF_GOAL_X:-700.0}"

if ! [[ "$TRIALS_PER_WORKER" =~ ^[1-9][0-9]*$ ]]; then
    echo "TRIALS_PER_WORKER must be a positive integer." >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$REPO_ROOT/.venv/bin/activate"

set -u

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
        --goal-x "$GOAL_X" \
        --startup-timeout "$STARTUP_TIMEOUT" \
        --stability-weight "$STABILITY_WEIGHT" \
        --headless \
        >"$worker_log" 2>&1 &
    pids+=("$!")
    echo "Worker $worker_id started: PID ${pids[-1]}, log $worker_log"
}

echo "Starting four isolated headless PX4/Gazebo workers."
echo "Trials: $TRIALS_PER_WORKER per worker, $((4 * TRIALS_PER_WORKER)) total"
echo "Study: $STUDY_NAME"
echo "Results: $OUTPUT_DIRECTORY"
echo "Goal X: $GOAL_X m"
echo "Stability weight: $STABILITY_WEIGHT points per deg/s of attitude reversal"

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
    gpu="unavailable"
    if command -v nvidia-smi >/dev/null 2>&1; then
        gpu="$(nvidia-smi --query-gpu=utilization.gpu,memory.used \
            --format=csv,noheader,nounits 2>/dev/null | awk -F, '{print $1 "%/" $2 "MiB"}')"
        gpu="${gpu:-unavailable}"
    fi
    echo "[$(date +%H:%M:%S)] workers=$running completed=$completed memory=$memory load=$load gpu=$gpu"
    ps -o pid=,pcpu=,pmem=,rss=,etime= -p "$(IFS=,; echo "${pids[*]}")" 2>/dev/null || true
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
