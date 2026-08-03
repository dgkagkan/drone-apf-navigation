#!/usr/bin/env bash

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"

TRIAL_NUMBER="${1:-}"
RESULTS_DIRECTORY="${2:-$REPO_ROOT/optuna_results/apf_1000_v2}"
REPLAY_DOMAIN_ID="${APF_REPLAY_DOMAIN_ID:-60}"
REPLAY_AGENT_PORT="${APF_REPLAY_AGENT_PORT:-9100}"
REPLAY_WORKER_ID="${APF_REPLAY_WORKER_ID:-10}"
STARTUP_TIMEOUT="${APF_REPLAY_STARTUP_TIMEOUT:-120}"
CAMERA_RETRY_INTERVAL="${APF_REPLAY_CAMERA_RETRY_INTERVAL:-3}"
CAMERA_RETRY_ATTEMPTS="${APF_REPLAY_CAMERA_RETRY_ATTEMPTS:-40}"
REPLAY_USE_RVIZ="${APF_REPLAY_USE_RVIZ:-true}"

if ! [[ "$TRIAL_NUMBER" =~ ^[0-9]+$ ]]; then
    echo "Usage: $0 TRIAL_NUMBER [RESULTS_DIRECTORY]" >&2
    exit 2
fi
if ! [[ "$REPLAY_DOMAIN_ID" =~ ^[0-9]+$ ]] || ((REPLAY_DOMAIN_ID > 232)); then
    echo "APF_REPLAY_DOMAIN_ID must be between 0 and 232." >&2
    exit 2
fi
if ! [[ "$REPLAY_AGENT_PORT" =~ ^[0-9]+$ ]] ||
    ((REPLAY_AGENT_PORT < 1 || REPLAY_AGENT_PORT > 65535)); then
    echo "APF_REPLAY_AGENT_PORT must be between 1 and 65535." >&2
    exit 2
fi
if ! [[ "$REPLAY_WORKER_ID" =~ ^[0-9]+$ ]] || ((REPLAY_WORKER_ID > 254)); then
    echo "APF_REPLAY_WORKER_ID must be between 0 and 254." >&2
    exit 2
fi
if ! [[ "$STARTUP_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "APF_REPLAY_STARTUP_TIMEOUT must be a positive integer." >&2
    exit 2
fi
if ! [[ "$CAMERA_RETRY_INTERVAL" =~ ^[1-9][0-9]*$ ]] ||
    ! [[ "$CAMERA_RETRY_ATTEMPTS" =~ ^[1-9][0-9]*$ ]]; then
    echo "Camera retry interval and attempts must be positive integers." >&2
    exit 2
fi
if [[ "$REPLAY_USE_RVIZ" != "true" && "$REPLAY_USE_RVIZ" != "false" ]]; then
    echo "APF_REPLAY_USE_RVIZ must be true or false." >&2
    exit 2
fi

TRIAL_DIRECTORY="$(printf '%s/trial_%05d' "$RESULTS_DIRECTORY" "$TRIAL_NUMBER")"
RESULT_FILE="$TRIAL_DIRECTORY/result.json"
if [[ ! -f "$RESULT_FILE" ]]; then
    echo "Trial result not found: $RESULT_FILE" >&2
    exit 2
fi
if ! command -v jq >/dev/null 2>&1; then
    echo "jq is required to read the stored trial parameters." >&2
    exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"

set -u

mapfile -t controller_arguments < <(
    jq -r '.parameters | to_entries[] | "\(.key):=\(.value)"' "$RESULT_FILE"
)
mission_profile="$(jq -r '.mission.profile // "single"' "$RESULT_FILE")"
stored_goal_x="$(jq -r '.mission.goals[0].x // 700.0' "$RESULT_FILE")"
stored_goal_y="$(jq -r '.mission.goals[0].y // 0.0' "$RESULT_FILE")"
stored_goal_altitude="$(jq -r '.mission.goals[0].altitude // 15.0' "$RESULT_FILE")"
stored_goal_2_x="$(jq -r '.mission.goals[1].x // 0.0' "$RESULT_FILE")"
stored_goal_2_y="$(jq -r '.mission.goals[1].y // 0.0' "$RESULT_FILE")"
stored_goal_2_altitude="$(jq -r '.mission.goals[1].altitude // 15.0' "$RESULT_FILE")"
stored_goal_3_x="$(jq -r '.mission.goals[2].x // 750.0' "$RESULT_FILE")"
stored_goal_3_y="$(jq -r '.mission.goals[2].y // 15.0' "$RESULT_FILE")"
stored_goal_3_altitude="$(jq -r '.mission.goals[2].altitude // 15.0' "$RESULT_FILE")"
stored_intermediate_tolerance="$(
    jq -r '.mission.intermediate_goal_tolerance // 25.0' "$RESULT_FILE"
)"

export ROS_DOMAIN_ID="$REPLAY_DOMAIN_ID"
export GZ_PARTITION="apf_replay_${REPLAY_WORKER_ID}_trial_${TRIAL_NUMBER}_$$"
export ROS_LOG_DIR="$TRIAL_DIRECTORY/replay_ros_logs_$$"
export ROS2CLI_DISABLE_DAEMON=1

target_system="$((REPLAY_WORKER_ID + 1))"
replay_work_dir="$TRIAL_DIRECTORY/replay_px4_${REPLAY_WORKER_ID}_$$"

replay_log="$TRIAL_DIRECTORY/replay_simulation.log"
sim_pid=""
controller_pid=""
camera_pid=""
cleanup_started=0

matching_agent_pids() {
    pgrep -f "MicroXRCEAgent udp4 -p ${REPLAY_AGENT_PORT}([[:space:]]|$)" || true
}

matching_px4_pids() {
    pgrep -f "/bin/px4.*-i ${REPLAY_WORKER_ID}([[:space:]]|$)" || true
}

terminate_matching_processes() {
    local signal="$1"
    local matcher="$2"
    local -a pids=()

    mapfile -t pids < <("$matcher")
    if ((${#pids[@]} > 0)); then
        kill "-$signal" "${pids[@]}" 2>/dev/null || true
    fi
}

wait_for_process_groups() {
    local deadline=$((SECONDS + 8))
    local pid
    local any_alive

    while ((SECONDS < deadline)); do
        any_alive=0
        for pid in "$controller_pid" "$sim_pid"; do
            if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
                any_alive=1
            fi
        done
        ((any_alive == 0)) && return
        sleep 0.25
    done
}

stop_replay() {
    local pid

    ((cleanup_started != 0)) && return
    cleanup_started=1
    trap - EXIT INT TERM
    echo
    echo "Stopping trial replay..."

    if [[ -n "$camera_pid" ]] && kill -0 "$camera_pid" 2>/dev/null; then
        kill -TERM "$camera_pid" 2>/dev/null || true
    fi
    for pid in "$controller_pid" "$sim_pid"; do
        if [[ -n "$pid" ]]; then
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done
    wait_for_process_groups

    for pid in "$controller_pid" "$sim_pid"; do
        if [[ -n "$pid" ]]; then
            kill -TERM -- "-$pid" 2>/dev/null || true
        fi
    done
    sleep 1
    for pid in "$controller_pid" "$sim_pid"; do
        if [[ -n "$pid" ]]; then
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    done

    # The launch process can exit before these children. Match only the replay
    # instance and port so the optimization workers remain untouched.
    terminate_matching_processes TERM matching_agent_pids
    terminate_matching_processes TERM matching_px4_pids
    sleep 0.5
    terminate_matching_processes KILL matching_agent_pids
    terminate_matching_processes KILL matching_px4_pids

    [[ -n "$camera_pid" ]] && wait "$camera_pid" 2>/dev/null || true
    [[ -n "$controller_pid" ]] && wait "$controller_pid" 2>/dev/null || true
    [[ -n "$sim_pid" ]] && wait "$sim_pid" 2>/dev/null || true
}

mapfile -t active_px4_pids < <(matching_px4_pids)
if ((${#active_px4_pids[@]} > 0)); then
    echo "Replay PX4 instance $REPLAY_WORKER_ID is already running (PID ${active_px4_pids[*]})." >&2
    echo "Stop the existing replay before starting another one." >&2
    exit 1
fi

mapfile -t stale_agent_pids < <(matching_agent_pids)
if ((${#stale_agent_pids[@]} > 0)); then
    echo "Removing stale replay agent on UDP port $REPLAY_AGENT_PORT (PID ${stale_agent_pids[*]})."
    terminate_matching_processes TERM matching_agent_pids
    sleep 1
    terminate_matching_processes KILL matching_agent_pids
fi

trap stop_replay EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

camera_follow_loop() {
    local target="standard_vtol_lidar_${REPLAY_WORKER_ID}"
    local attempt

    for ((attempt = 1; attempt <= CAMERA_RETRY_ATTEMPTS; ++attempt)); do
        kill -0 "$sim_pid" 2>/dev/null || return
        gz topic -t /gui/track -m gz.msgs.CameraTrack -p \
            "track_mode: FOLLOW, follow_target: {name: '${target}'}, follow_offset: {x: -8.0, y: 0.0, z: 3.0}, follow_pgain: 1.0, track_pgain: 1.0" \
            >>"$replay_log" 2>&1 || true
        sleep "$CAMERA_RETRY_INTERVAL"
    done
}

wait_for_replay_topics() {
    local deadline=$((SECONDS + STARTUP_TIMEOUT))
    local topics=""
    local topic
    local all_ready
    local -a required_topics=(
        "/fmu/out/vehicle_status_v4"
        "/fmu/out/vehicle_local_position_v1"
        "/fmu/out/vehicle_attitude"
        "/scan_3d/points"
    )

    while ((SECONDS < deadline)); do
        kill -0 "$sim_pid" 2>/dev/null || return 1
        topics="$(timeout 5 ros2 topic list 2>/dev/null || true)"
        all_ready=1
        for topic in "${required_topics[@]}"; do
            if ! grep -Fxq "$topic" <<<"$topics"; then
                all_ready=0
                break
            fi
        done
        ((all_ready != 0)) && return 0
        sleep 2
    done

    return 1
}

echo "Replaying trial $TRIAL_NUMBER from $RESULT_FILE"
echo "Gazebo/PX4 log: $replay_log"
echo "Isolation: ROS domain $ROS_DOMAIN_ID, PX4 instance $REPLAY_WORKER_ID, agent $REPLAY_AGENT_PORT"
echo "RViz APF overlays: $REPLAY_USE_RVIZ"
echo "Mission profile: $mission_profile"
echo "Press Ctrl+C to stop."

setsid ros2 launch drone_bringup sim.launch.py \
    world:=optuna_course \
    headless:=0 \
    px4_terminal:=inline \
    parallel_worker_id:="$REPLAY_WORKER_ID" \
    agent_port:="$REPLAY_AGENT_PORT" \
    px4_work_dir:="$replay_work_dir" \
    use_rviz:="$REPLAY_USE_RVIZ" \
    follow:=false \
    >"$replay_log" 2>&1 &
sim_pid="$!"

echo "Camera follow will retry for $((CAMERA_RETRY_INTERVAL * CAMERA_RETRY_ATTEMPTS)) seconds."
camera_follow_loop &
camera_pid="$!"

echo "Waiting up to ${STARTUP_TIMEOUT}s for PX4 and 3D LiDAR topics..."
if ! wait_for_replay_topics; then
    echo "Replay did not become ready. Last simulation log lines:" >&2
    tail -n 40 "$replay_log" >&2 || true
    exit 1
fi
echo "PX4 and 3D LiDAR discovered. Starting the automated controller."
sleep "${APF_REPLAY_SETTLE_DELAY:-3}"

setsid ros2 launch drone_bringup automated_controller.launch.py \
    target_system:="$target_system" \
    mission_profile:="$mission_profile" \
    goal_x:="${APF_REPLAY_GOAL_X:-$stored_goal_x}" \
    goal_y:="${APF_REPLAY_GOAL_Y:-$stored_goal_y}" \
    cruise_altitude:="${APF_REPLAY_ALTITUDE:-$stored_goal_altitude}" \
    goal_2_x:="$stored_goal_2_x" \
    goal_2_y:="$stored_goal_2_y" \
    goal_2_altitude:="$stored_goal_2_altitude" \
    goal_3_x:="$stored_goal_3_x" \
    goal_3_y:="$stored_goal_3_y" \
    goal_3_altitude:="$stored_goal_3_altitude" \
    intermediate_goal_tolerance:="$stored_intermediate_tolerance" \
    transform_enabled:=false \
    visualization_enabled:="$REPLAY_USE_RVIZ" \
    "${controller_arguments[@]}" &
controller_pid="$!"

wait "$controller_pid"
