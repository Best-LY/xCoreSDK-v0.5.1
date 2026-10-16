#!/usr/bin/env bash
set -euo pipefail

usage() {
    printf '%s\n' \
        'Usage: return_to_grasp_start.sh --execute | --plan' \
        'Left arm only. Fixed saved grasp-start joints, NOT home.' \
        'No camera access. --execute moves the robot at 10% joint speed.' \
        'Keep people/obstacles clear and an operator at the emergency stop.' \
        'Both arms must be idle; the right arm is monitored but never commanded.'
}

if [[ $# -eq 0 || ${1:-} == --help ]]; then
    usage
    exit 0
fi
if [[ $# -ne 1 || ( $1 != --execute && $1 != --plan ) ]]; then
    usage >&2
    exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/../.." && pwd)"
target="$project_root/example/function_config/recorded_poses/20261006_143947_before_manual/left.yml"
binary="$project_root/build/bin/return_observation"
[[ -r "$target" ]] || { printf 'Saved grasp-start pose is missing: %s\n' "$target" >&2; exit 2; }
[[ -d "$project_root/build" ]] || { printf 'Configure the project build directory first.\n' >&2; exit 2; }

# Prevent concurrent invocations of this launcher. The executable handles SIGINT/SIGTERM.
command -v flock >/dev/null || { printf 'Required command not found: flock\n' >&2; exit 2; }
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/xcore-left-grasp-start-${UID}.lock"
flock -n 9 || { printf 'A grasp-start return launcher is already running.\n' >&2; exit 2; }

cmake --build "$project_root/build" --target return_observation -j2
export LD_LIBRARY_PATH="/home/arm/.local/xcore-vision/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
printf 'Fixed target: %s\n' "$target"
if [[ $1 == --execute ]]; then
    exec "$binary" "$target" --execute
else
    exec "$binary" "$target"
fi
