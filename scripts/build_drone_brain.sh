#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_root="$(cd -- "${repository_root}/../.." && pwd)"

# Do not inherit stale packages from a previously sourced workspace. The only
# non-system underlay needed by the brain is px4_msgs, sourced explicitly below.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH
source /opt/ros/jazzy/setup.bash

px4_msgs_prefix="${workspace_root}/install/px4_msgs"
if ! ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
  if [[ -f "${px4_msgs_prefix}/local_setup.bash" ]]; then
    source "${px4_msgs_prefix}/local_setup.bash"
  elif [[ -f "${px4_msgs_prefix}/share/px4_msgs/package.bash" ]]; then
    source "${px4_msgs_prefix}/share/px4_msgs/package.bash"
  fi
fi
set -u
if ! ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
  echo "px4_msgs is not available in the current ROS environment." >&2
  echo "Build or source px4_msgs before running this script." >&2
  exit 1
fi

export CMAKE_BUILD_PARALLEL_LEVEL=1
export MAKEFLAGS="-j1"

cd "${workspace_root}"
colcon --log-base log/onboard build \
  --base-paths "${repository_root}" \
  --build-base build/onboard \
  --install-base install/onboard \
  --executor sequential \
  --symlink-install \
  --allow-overriding \
    drone_interfaces \
    drone_description \
    drone_control \
    drone_navigation \
    drone_swarm \
    drone_bringup \
  --packages-select \
    drone_interfaces \
    drone_description \
    drone_control \
    drone_navigation \
    drone_swarm \
    drone_bringup \
  --cmake-args \
    -DDRONE_ONBOARD_BUILD=ON \
    -DBUILD_TESTING=OFF

echo
echo "Onboard build complete. Source it with:"
echo "  source ${workspace_root}/install/onboard/setup.bash"
