#!/usr/bin/env python3
"""Capture two RGB-D bursts, validate a knee, and preview its base point."""

import argparse
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path,
                        default=Path("/home/arm/.local/share/xmc-pose-models/yolo11m-pose.pt"))
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--output-prefix", type=Path,
                        default=Path("/tmp/xmc_right_knee_pipeline"))
    args = parser.parse_args()

    directory = Path(__file__).resolve().parent
    run_prefix = Path(str(args.output_prefix) + "_" + str(int(time.time() * 1000)))
    first = Path(str(run_prefix) + "_a")
    second = Path(str(run_prefix) + "_b")
    first_json = Path(str(first) + "_stability.json")
    second_json = Path(str(second) + "_stability.json")
    common = [sys.executable, str(directory / "mannequin_joint_stability.py"),
              "--weights", str(args.weights), "--frames", str(args.frames)]

    initial = subprocess.run(common + ["--output-prefix", str(first)],
                             check=False)
    if initial.returncode != 2 or not first_json.is_file():
        raise SystemExit(initial.returncode or 2)
    comparison = subprocess.run(common + ["--output-prefix", str(second),
                                          "--compare-with", str(first_json)],
                                check=False)
    if comparison.returncode != 0:
        print("[HOLD] knee localization did not meet the two-burst gate")
        return comparison.returncode
    preview = subprocess.run([sys.executable,
                              str(directory / "mannequin_joint_base_preview.py"),
                              "--stability-json", str(second_json)],
                             check=False)
    if preview.returncode != 0:
        print("[HOLD] robot coordinate preview did not pass its checks")
        return preview.returncode
    print("[SAFE] two-burst knee point preview complete; no robot motion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
