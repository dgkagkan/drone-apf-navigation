#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_dir}"

drone_count="${1:-${DRONE_COUNT:-3}}"
gpu_backend="${GPU_BACKEND:-auto}"
compose_files=(-f compose.yaml)

if [[ ! "${drone_count}" =~ ^[0-9]+$ ]]; then
  echo "DRONE_COUNT must be a non-negative integer." >&2
  exit 2
fi

case "${gpu_backend}" in
  auto)
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
      gpu_backend="nvidia"
    elif compgen -G "/dev/dri/renderD*" >/dev/null; then
      gpu_backend="intel"
    else
      gpu_backend="software"
    fi
    ;;
  nvidia|intel|software)
    ;;
  *)
    echo "GPU_BACKEND must be auto, nvidia, intel, or software." >&2
    exit 2
    ;;
esac

case "${gpu_backend}" in
  nvidia)
    compose_files+=(-f compose.nvidia.yaml)
    ;;
  intel)
    if [[ ! -e /dev/dri/renderD128 ]]; then
      echo "GPU_BACKEND=intel requested, but /dev/dri/renderD128 is unavailable." >&2
      exit 1
    fi
    if render_gid="$(stat -c '%g' /dev/dri/renderD128 2>/dev/null)"; then
      export RENDER_GID="${render_gid}"
    fi
    compose_files+=(-f compose.gpu.yaml)
    ;;
  software)
    export LIBGL_ALWAYS_SOFTWARE=1
    ;;
esac

if [[ "${DRONE_HEADLESS:-false}" != "true" ]] && command -v xhost >/dev/null 2>&1; then
  xhost "+si:localuser:$(id -un)" >/dev/null 2>&1 || true
fi

echo "Starting ${drone_count} drone(s) with ${gpu_backend} rendering."
echo "Compose files: ${compose_files[*]}"
DRONE_COUNT="${drone_count}" docker compose "${compose_files[@]}" up --build
