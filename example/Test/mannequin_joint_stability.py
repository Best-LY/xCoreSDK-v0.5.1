#!/usr/bin/env python3
"""Measure temporal stability of the right mannequin knee in live RGB-D frames."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

from mannequin_joint_preview import choose_knee, knee_depth, load_snapshot


RIGHT_CAMERA_SERIAL = "236422070786"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path,
                        default=Path("/tmp/xmc_right_knee_stability"))
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--analyze-existing", action="store_true")
    parser.add_argument("--compare-with", type=Path)
    args = parser.parse_args()
    if not 10 <= args.frames <= 60:
        parser.error("--frames must be between 10 and 60")

    model = YOLO(str(args.weights))
    if not args.analyze_existing:
        snapshot = Path(__file__).resolve().parents[2] / "build/bin/mannequin_snapshot"
        subprocess.run([str(snapshot), "--camera", RIGHT_CAMERA_SERIAL,
                        "--output-prefix", str(args.output_prefix),
                        "--burst", str(args.frames)], check=True)
    observations = []
    last_image = None
    captured_ms = []
    for sample in range(args.frames):
        metadata = Path(f"{args.output_prefix}_{sample:03d}.yml")
        image, depth_image, intrinsics, scale, timestamp = load_snapshot(
            metadata, -1)
        captured_ms.append(timestamp)
        last_image = image.copy()
        result = model.predict(image, imgsz=960, conf=0.20,
                               device=0, verbose=False)[0]
        try:
            person, _, knee_conf, x, y, _ = choose_knee(result, image.shape)
            distance, spread, _ = knee_depth(depth_image, x, y, scale)
            xyz = rs.rs2_deproject_pixel_to_point(
                intrinsics, [x, y], distance)
            if not np.all(np.isfinite(xyz)):
                continue
            observations.append({
                "sample": sample,
                "pixel": [float(x), float(y)],
                "camera_xyz_m": [float(v) for v in xyz],
                "depth_m": distance,
                "depth_patch_spread_m": spread,
                "person_confidence": person,
                "keypoint_confidence": knee_conf,
            })
        except ValueError:
            continue

    count = len(observations)
    required = max(8, int(np.ceil(0.8 * args.frames)))
    payload = {
        "camera_serial": RIGHT_CAMERA_SERIAL,
        "joint": "right_knee",
        "capture_unix_ms": max(captured_ms),
        "analysis_unix_ms": int(time.time() * 1000),
        "frames_requested": args.frames,
        "frames_valid": count,
        "required_valid_frames": required,
        "robot_motion_permitted": False,
    }
    if count:
        points = np.asarray([item["camera_xyz_m"] for item in observations])
        pixels = np.asarray([item["pixel"] for item in observations])
        center = np.median(points, axis=0)
        median_pixel = np.median(pixels, axis=0)
        residuals = np.linalg.norm(points - center, axis=1)
        pixel_residuals = np.linalg.norm(pixels - median_pixel, axis=1)
        payload.update({
            "median_camera_xyz_m": center.tolist(),
            "median_pixel": median_pixel.tolist(),
            "position_p90_deviation_m": float(np.percentile(residuals, 90)),
            "position_max_deviation_m": float(np.max(residuals)),
            "pixel_p90_deviation_px": float(np.percentile(pixel_residuals, 90)),
            "observations": observations,
        })
    stable = False
    if args.compare_with:
        with args.compare_with.open(encoding="ascii") as source:
            previous = json.load(source)
        if (previous.get("camera_serial") != RIGHT_CAMERA_SERIAL or
                previous.get("joint") != "right_knee"):
            raise ValueError("comparison data is not the right-camera knee")
        if count == 0 or previous.get("frames_valid", 0) == 0:
            raise ValueError("comparison needs valid knee points in both bursts")
        age_s = (payload["capture_unix_ms"] - previous["capture_unix_ms"]) / 1000.0
        if not 0 <= age_s <= 60:
            raise ValueError(f"comparison data age is not within 60 s: {age_s:.1f}s")
        previous_center = np.asarray(previous["median_camera_xyz_m"])
        current_center = np.asarray(payload["median_camera_xyz_m"])
        difference = float(np.linalg.norm(current_center - previous_center))
        payload["comparison_file"] = str(args.compare_with)
        payload["between_burst_difference_m"] = difference
        payload["two_burst_camera_xyz_m"] = (
            0.5 * (current_center + previous_center)).tolist()
        stable = (
            count >= required and
            previous["frames_valid"] >= previous["required_valid_frames"] and
            payload["position_p90_deviation_m"] <= 0.07 and
            previous["position_p90_deviation_m"] <= 0.07 and
            difference <= 0.02
        )
    payload["stable_for_planning"] = bool(stable)

    result_path = Path(str(args.output_prefix) + "_stability.json")
    with result_path.open("w", encoding="ascii") as output:
        json.dump(payload, output, indent=2)
        output.write("\n")
    if last_image is not None and count:
        preview = last_image.copy()
        px, py = [int(round(v)) for v in payload["median_pixel"]]
        cv2.circle(preview, (px, py), 10, (0, 0, 255), 2)
        cv2.imwrite(str(args.output_prefix) + "_stability.png", preview)

    print(f"[STABILITY] valid={count}/{args.frames} required={required} "
          f"p90_3d={payload.get('position_p90_deviation_m', float('nan')):.3f}m "
          f"p90_pixel={payload.get('pixel_p90_deviation_px', float('nan')):.1f}px "
          f"stable={stable}")
    if args.compare_with:
        print(f"[COMPARE] median_difference="
              f"{payload['between_burst_difference_m']:.3f}m")
    if count:
        print(f"[POINT] median_camera_xyz_m={payload['median_camera_xyz_m']}")
    print(f"[SAFE] perception only; no robot connection or motion; data={result_path}")
    return 0 if stable else 2


if __name__ == "__main__":
    raise SystemExit(main())
