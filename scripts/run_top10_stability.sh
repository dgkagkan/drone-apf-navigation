#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

SOURCE_DIRECTORY="${1:-$REPO_ROOT/optuna_results/apf_stability}"
SOURCE_STUDY="${2:-apf_stability}"
TOP_COUNT="${3:-10}"
REPETITIONS="${4:-15}"
OUTPUT_DIRECTORY="${5:-$REPO_ROOT/optuna_results/apf_top10_stability}"
TARGET_STUDY="${APF_BENCHMARK_STUDY:-apf_top10_stability}"
WORKER_COUNT="${APF_BENCHMARK_WORKERS:-4}"
RUN_ID="${APF_RUN_ID:-top_stability_$(date +%Y%m%d_%H%M%S)}"
BASE_DOMAIN_ID="${APF_BASE_DOMAIN_ID:-40}"
BASE_AGENT_PORT="${APF_BASE_AGENT_PORT:-9000}"
STARTUP_TIMEOUT="${APF_STARTUP_TIMEOUT:-150}"
WORKER_START_STAGGER="${APF_WORKER_START_STAGGER:-12}"
STABILITY_WEIGHT="${APF_STABILITY_WEIGHT:-1.0}"
GOAL_X="${APF_GOAL_X:-700.0}"
PREPARATION_SUMMARY="$OUTPUT_DIRECTORY/benchmark_plan.json"

for value_name in TOP_COUNT REPETITIONS WORKER_COUNT; do
    value="${!value_name}"
    if ! [[ "$value" =~ ^[1-9][0-9]*$ ]]; then
        echo "$value_name must be a positive integer." >&2
        exit 2
    fi
done
if ((WORKER_COUNT > 4)); then
    echo "APF_BENCHMARK_WORKERS cannot exceed 4 with the current isolated setup." >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$REPO_ROOT/.venv/bin/activate"

set -u

# The agent and ROS nodes use the same local-only Fast DDS transport. This keeps
# high-rate LiDAR traffic away from Ethernet, Tailscale, ZeroTier, and Docker.
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

write_report() {
    python -m apf_optuna.benchmark_report \
        --output-directory "$OUTPUT_DIRECTORY" \
        --study-name "$TARGET_STUDY" || true
}

stop_workers() {
    trap - INT TERM
    echo
    echo "Stopping stability workers..."
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        fi
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    write_report
    exit 130
}

trap stop_workers INT TERM

start_worker() {
    local worker_id="$1"
    local trial_count="$2"
    local worker_log="$OUTPUT_DIRECTORY/benchmark_worker_${worker_id}.log"

    setsid python -m apf_optuna.optimizer \
        --trials "$trial_count" \
        --study-name "$TARGET_STUDY" \
        --output-directory "$OUTPUT_DIRECTORY" \
        --seed "$((4200 + worker_id))" \
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
    echo "Worker $worker_id: $trial_count runs, PID ${pids[-1]}, log $worker_log"
}

expected_valid_runs=$((TOP_COUNT * REPETITIONS))
failed=0
batch=0

echo "Starting repeated stability benchmark."
echo "Candidates: $TOP_COUNT, valid repetitions per candidate: $REPETITIONS"
echo "Study: $TARGET_STUDY, results: $OUTPUT_DIRECTORY"

while true; do
    python -m apf_optuna.prepare_stability_benchmark \
        --source-directory "$SOURCE_DIRECTORY" \
        --source-study "$SOURCE_STUDY" \
        --target-directory "$OUTPUT_DIRECTORY" \
        --target-study "$TARGET_STUDY" \
        --count "$TOP_COUNT" \
        --repetitions "$REPETITIONS" \
        --summary-file "$PREPARATION_SUMMARY"

    waiting_total="$(jq -r '.waiting_total' "$PREPARATION_SUMMARY")"
    if ((waiting_total == 0)); then
        echo "Collected all $expected_valid_runs valid benchmark runs."
        break
    fi

    batch=$((batch + 1))
    active_workers=$((waiting_total < WORKER_COUNT ? waiting_total : WORKER_COUNT))
    trials_per_worker=$((waiting_total / active_workers))
    extra_trials=$((waiting_total % active_workers))
    pids=()

    echo "Batch $batch: $waiting_total waiting runs across $active_workers workers."
    for ((worker_id = 0; worker_id < active_workers; ++worker_id)); do
        worker_trials="$trials_per_worker"
        ((worker_id < extra_trials)) && worker_trials=$((worker_trials + 1))
        start_worker "$worker_id" "$worker_trials"
        if ((worker_id + 1 < active_workers)); then
            sleep "$WORKER_START_STAGGER"
        fi
    done

    while true; do
        running=0
        for pid in "${pids[@]}"; do
            kill -0 "$pid" 2>/dev/null && running=$((running + 1))
        done
        ((running == 0)) && break

        progress="$({
            find "$OUTPUT_DIRECTORY" -name result.json -type f -print0 |
                xargs -0 -r jq -r '.metrics.outcome // "unknown"'
        } | awk '
            BEGIN {stored = 0; valid = 0; success = 0; retry = 0}
            {
                stored++
                if ($0 == "controller_exit" || $0 == "no_telemetry" ||
                    $0 == "simulator_exit" || $0 == "stalled" ||
                    $0 == "startup_timeout" || $0 == "timeout") {
                    retry++
                } else {
                    valid++
                    if ($0 == "success") success++
                }
            }
            END {
                printf "stored=%d valid=%d success=%d retry=%d", \
                    stored, valid, success, retry
            }
        ')"
        memory="$(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
        load="$(awk '{print $1 "," $2 "," $3}' /proc/loadavg)"
        echo "[$(date +%H:%M:%S)] workers=$running $progress target_valid=$expected_valid_runs memory=$memory load=$load"
        sleep 10
    done

    batch_failed=0
    for index in "${!pids[@]}"; do
        if wait "${pids[$index]}"; then
            echo "Worker $index completed successfully."
        else
            echo "Worker $index failed; inspect benchmark_worker_${index}.log" >&2
            batch_failed=1
        fi
    done
    if ((batch_failed != 0)); then
        failed=1
        break
    fi

    echo "Batch $batch complete; checking whether retry runs are needed."
done

write_report
exit "$failed"
