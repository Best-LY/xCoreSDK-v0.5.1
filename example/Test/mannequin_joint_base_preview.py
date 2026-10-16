#!/usr/bin/env python3
"""Pass a verified two-burst knee estimate to the read-only robot transform tool."""

import argparse
import json
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stability-json", type=Path, required=True)
    args = parser.parse_args()
    with args.stability_json.open(encoding="ascii") as source:
        point = json.load(source)
    if (point.get("stable_for_planning") is not True or
            point.get("robot_motion_permitted") is not False or
            len(point.get("two_burst_camera_xyz_m", [])) != 3):
        raise ValueError("need stable two-burst perception-only knee data")
    binary = Path(__file__).resolve().parents[2] / "build/bin/mannequin_joint_base_preview"
    xyz = point["two_burst_camera_xyz_m"]
    command = [
        str(binary),
        "--camera-serial", point["camera_serial"],
        "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
        "--capture-ms", str(point["capture_unix_ms"]),
        "--between-burst-m", str(point["between_burst_difference_m"]),
    ]
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == "__main__":
    main()
