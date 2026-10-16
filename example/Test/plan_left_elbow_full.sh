#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 0 ]]; then
    printf '%s\n' 'Usage: plan_left_elbow_full.sh (read-only, one capture round, both motion stages)'
    [[ $# -eq 1 && $1 == --help ]] && exit 0
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$script_dir/../.." && pwd)"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/xcore-left-full-plan-${UID}.lock"
flock -n 9 || { printf 'Another full-plan capture is running.\n' >&2; exit 2; }
cmake --build "$root/build" --target mannequin_approach_plan mannequin_snapshot -j2
mkdir -p "$script_dir/reports/full_plan"
run_dir="$(mktemp -d "$script_dir/reports/full_plan/run_XXXXXXXX")"
export LD_LIBRARY_PATH="/home/arm/.local/xcore-vision/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="/home/arm/.local/share/xmc-pose-python${PYTHONPATH:+:$PYTHONPATH}"
python=/home/arm/论文/大语言模型/qwen_env/bin/python
printf 'Read-only full plan, artifacts: %s\n' "$run_dir"
"$python" "$script_dir/left_shoulder_micro_approach.py" --staged-plan --joint left_elbow \
    --output-prefix "$run_dir/left_elbow" 2>&1 | tee "$run_dir/planning.log"
shopt -s nullglob
plans=("$run_dir/"*.full_plan.json)
[[ ${#plans[@]} -eq 1 ]] || { printf 'No unique complete full-plan artifact.\n' >&2; exit 2; }
"$python" "$script_dir/validate_full_plan.py" "${plans[0]}" 2>&1 | tee "$run_dir/validation.log"
printf 'Full plan: %s\nNo robot motion was commanded.\n' "${plans[0]}"
