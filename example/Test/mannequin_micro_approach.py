#!/usr/bin/env python3
"""Validate a knee point JSON and hand its scalars to the guarded C++ probe."""

import argparse
import json
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = Path(__file__).resolve()
    project = source.parents[2]
    parser.add_argument("--point-json", type=Path)
    parser.add_argument("--capture", action="store_true")
    parser.add_argument("--output-prefix", type=Path,
                        default=Path("/tmp/xmc_right_joint_probe"))
    parser.add_argument("--weights", type=Path,
                        default=Path("/home/arm/.local/share/xmc-pose-models/yolo11m-pose.pt"))
    parser.add_argument("--step-mm", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if args.capture == bool(args.point_json):
        parser.error("select exactly one of --capture or --point-json")
    binary = project / "build/bin/mannequin_micro_approach"
    point_path = args.point_json
    if args.capture:
        snapshot = project / "build/bin/mannequin_snapshot"
        subprocess.run([str(snapshot), "--output-prefix", str(args.output_prefix)],
                       check=True)
        subprocess.run([
            sys.executable, str(source.with_name("mannequin_joint_preview.py")),
            "--snapshot", str(args.output_prefix) + ".yml",
            "--weights", str(args.weights),
            "--output-prefix", str(args.output_prefix),
        ], check=True)
        point_path = Path(str(args.output_prefix) + "_point.json")

    with point_path.open(encoding="ascii") as point_source:
        point = json.load(point_source)
    if point["robot_motion_permitted"] is not False:
        raise ValueError("point JSON must be a perception-only preview")
    if len(point["pixel"]) != 2:
        raise ValueError("point JSON needs two pixel coordinates")

    command = [
        str(binary),
        "--camera-serial", str(point["camera_serial"]),
        "--joint", str(point["joint"]),
        "--capture-ms", str(point["capture_unix_ms"]),
        "--pixel-x", str(point["pixel"][0]),
        "--pixel-y", str(point["pixel"][1]),
        "--depth-m", str(point["depth_m"]),
        "--person-conf", str(point["person_confidence"]),
        "--knee-conf", str(point["keypoint_confidence"]),
        "--spread-m", str(point["depth_p10_p90_spread_m"]),
        "--step-mm", str(args.step_mm),
    ]
    if args.execute:
        command.append("--execute")
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == "__main__":
    main()
