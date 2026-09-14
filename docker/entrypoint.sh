#!/usr/bin/env bash

set -euo pipefail

runtime_uid="${LOCAL_UID:-1000}"
runtime_gid="${LOCAL_GID:-1000}"

if [[ "${EUID}" -eq 0 ]]; then
    if [[ ! "${runtime_uid}" =~ ^[0-9]+$ || ! "${runtime_gid}" =~ ^[0-9]+$ ]]; then
        echo "LOCAL_UID and LOCAL_GID must be numeric." >&2
        exit 2
    fi

    current_gid="$(id -g drone)"
    current_uid="$(id -u drone)"
    if [[ "${current_gid}" != "${runtime_gid}" ]]; then
        groupmod --non-unique --gid "${runtime_gid}" drone
    fi
    if [[ "${current_uid}" != "${runtime_uid}" ]]; then
        usermod --non-unique --uid "${runtime_uid}" --gid "${runtime_gid}" drone
    fi

    install -d -o "${runtime_uid}" -g "${runtime_gid}" \
        /data/photos /data/recordings /data/settings \
        /workspace/build /workspace/install /workspace/log
    chown -R "${runtime_uid}:${runtime_gid}" /home/drone /workspace/build \
        /workspace/install /workspace/log
    exec gosu drone "$0" "$@"
fi

export HOME=/home/drone
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/px4_msgs_ws/install/setup.bash
set -u

if [[ -n "${DRONE_DEV_SOURCE:-}" ]]; then
    if [[ "${DRONE_DEV_BUILD:-1}" == "1" ]]; then
        drone-dev-build
    fi
    set +u
    source /workspace/install/setup.bash
    set -u
else
    set +u
    source /opt/drone_ws/install/setup.bash
    set -u
fi

if [[ "${1:-}" == "run" ]]; then
    exec ros2 launch drone_bringup swarm_sim.launch.py \
        "px4_dir:=${PX4_DIR}" \
        "agent:=${XRCE_AGENT}" \
        "drones:=${DRONE_COUNT:-3}" \
        "base_agent_port:=${DRONE_BASE_AGENT_PORT:-8888}" \
        "drone_spawn_spacing_m:=${DRONE_SPAWN_SPACING_M:-8.0}" \
        "world:=${DRONE_WORLD:-test}" \
        "headless:=${DRONE_HEADLESS:-true}" \
        "use_rviz:=${DRONE_USE_RVIZ:-false}" \
        "use_mapping:=${DRONE_USE_MAPPING:-true}" \
        "network_mode:=${DRONE_NETWORK_MODE:-local}" \
        "ros_domain_id:=${ROS_DOMAIN_ID:-0}" \
        "operator_terminal:=false" \
        "open_dashboard:=false" \
        "dashboard_host:=0.0.0.0" \
        "dashboard_port:=${DASHBOARD_PORT:-8765}" \
        "dashboard_rate_hz:=${DASHBOARD_RATE_HZ:-60.0}" \
        "preconfigure_media_storage:=true" \
        "photo_save_dir:=/data/photos" \
        "record_save_dir:=/data/recordings" \
        "settings_profile_path:=${SETTINGS_PROFILE_PATH:-/data/settings/runtime_profiles.json}"
fi

exec "$@"
