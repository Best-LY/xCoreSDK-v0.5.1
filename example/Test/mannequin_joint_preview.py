#!/usr/bin/env python3
"""Preview a mannequin knee in an aligned RealSense snapshot; never moves a robot."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


RIGHT_CAMERA_SERIAL = "236422070786"
RIGHT_KNEE_INDEX = 14


def scalar(storage, name):
    node = storage.getNode(name)
    if node.empty():
        raise ValueError(f"snapshot metadata missing {name}")
    return node.real()


def load_snapshot(path, max_age_s, expected_serial=RIGHT_CAMERA_SERIAL):
    storage = cv2.FileStorage(str(path), cv2.FileStorage_READ)
    if not storage.isOpened():
        raise ValueError(f"cannot open snapshot metadata: {path}")
    try:
        serial = storage.getNode("camera_serial").string()
        if serial != expected_serial:
            raise ValueError(f"expected camera {expected_serial}, got {serial}")
        color_path = storage.getNode("color_file").string()
        depth_path = storage.getNode("depth_file").string()
        stamp = storage.getNode("capture_unix_ms")
        captured_ms = int(stamp.string() if stamp.isString()
                          else scalar(storage, "capture_unix_ms"))
        age_s = time.time() - captured_ms / 1000.0
        if age_s < -2 or (max_age_s >= 0 and age_s > max_age_s):
            raise ValueError(f"snapshot is stale or clock is inconsistent: age={age_s:.1f}s")

        intrinsics = rs.intrinsics()
        intrinsics.width = int(scalar(storage, "width"))
        intrinsics.height = int(scalar(storage, "height"))
        intrinsics.fx = float(scalar(storage, "fx"))
        intrinsics.fy = float(scalar(storage, "fy"))
        intrinsics.ppx = float(scalar(storage, "ppx"))
        intrinsics.ppy = float(scalar(storage, "ppy"))
        intrinsics.model = rs.distortion(int(scalar(storage, "distortion_model")))
        coeffs = storage.getNode("distortion_coeffs").mat()
        if coeffs is None or coeffs.size != 5:
            raise ValueError("snapshot distortion coefficients are missing")
        intrinsics.coeffs = [float(value) for value in coeffs.reshape(-1)]
        depth_scale_m = float(scalar(storage, "depth_scale_m"))
    finally:
        storage.release()

    color = cv2.imread(color_path, cv2.IMREAD_COLOR)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if (color is None or depth is None or depth.dtype != np.uint16 or
            color.shape[:2] != depth.shape or
            color.shape[:2] != (intrinsics.height, intrinsics.width)):
        raise ValueError("snapshot color/depth are missing, mismatched, or not aligned")
    if not 0 < depth_scale_m < 0.01:
        raise ValueError(f"invalid depth scale: {depth_scale_m}")
    return color, depth, intrinsics, depth_scale_m, captured_ms


def choose_knee(result, image_shape):
    return choose_joint(result, image_shape, RIGHT_KNEE_INDEX)


def choose_joint(result, image_shape, joint_index):
    height, width = image_shape[:2]
    candidates = []
    for index, box in enumerate(result.boxes):
        box_conf = float(box.conf[0])
        left, top, right, bottom = [float(v) for v in box.xyxy[0]]
        area_fraction = ((right - left) * (bottom - top)) / (width * height)
        keypoint = result.keypoints.data[index, joint_index]
        x, y, knee_conf = [float(v) for v in keypoint]
        if (box_conf < 0.45 or area_fraction < 0.20 or knee_conf < 0.80 or
                not left <= x <= right or not top <= y <= bottom or
                not 8 <= x < width - 8 or not 8 <= y < height - 8):
            continue
        candidates.append((box_conf, area_fraction, knee_conf, x, y,
                           (left, top, right, bottom)))
    if len(candidates) != 1:
        raise ValueError(f"need one reliable mannequin, found {len(candidates)}")
    return candidates[0]


def knee_depth(depth, x, y, scale):
    col, row = int(round(x)), int(round(y))
    patch = depth[row - 5:row + 6, col - 5:col + 6]
    values = patch[patch > 0].astype(np.float64) * scale
    values = values[(values >= 0.35) & (values <= 3.0)]
    if len(values) < 80:
        raise ValueError(f"insufficient valid knee depth pixels: {len(values)}/121")
    p10, p90 = np.percentile(values, [10, 90])
    if p90 - p10 > 0.04:
        raise ValueError(f"knee depth patch crosses a surface edge: spread={p90-p10:.3f}m")
    return float(np.median(values)), float(p90 - p10), len(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--max-age-s", type=float, default=30.0)
    args = parser.parse_args()

    color, depth, intrinsics, scale, captured_ms = load_snapshot(
        args.snapshot, args.max_age_s)
    if not args.weights.is_file():
        raise ValueError(f"pose model not found: {args.weights}")
    model = YOLO(str(args.weights))
    result = model.predict(color, imgsz=960, conf=0.20, device=0,
                           verbose=False)[0]
    box_conf, area_fraction, knee_conf, x, y, box = choose_knee(
        result, color.shape)
    distance_m, spread_m, valid_pixels = knee_depth(depth, x, y, scale)
    xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], distance_m)
    if not np.all(np.isfinite(xyz)):
        raise ValueError("knee deprojection produced non-finite coordinates")

    preview = color.copy()
    left, top, right, bottom = [int(round(v)) for v in box]
    cv2.rectangle(preview, (left, top), (right, bottom), (0, 180, 0), 2)
    cv2.circle(preview, (int(round(x)), int(round(y))), 10, (0, 0, 255), 3)
    cv2.putText(preview, f"RIGHT KNEE  {distance_m:.3f} m",
                (int(round(x)) + 14, int(round(y)) - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    preview_path = str(args.output_prefix) + "_preview.png"
    result_path = str(args.output_prefix) + "_point.json"
    if not cv2.imwrite(preview_path, preview):
        raise ValueError(f"cannot write preview: {preview_path}")
    payload = {
        "camera_serial": RIGHT_CAMERA_SERIAL,
        "joint": "right_knee",
        "coco_index": RIGHT_KNEE_INDEX,
        "capture_unix_ms": captured_ms,
        "pixel": [x, y],
        "camera_xyz_m": [float(v) for v in xyz],
        "depth_m": distance_m,
        "depth_p10_p90_spread_m": spread_m,
        "valid_depth_pixels": valid_pixels,
        "person_confidence": box_conf,
        "person_box_area_fraction": area_fraction,
        "keypoint_confidence": knee_conf,
        "robot_motion_permitted": False,
    }
    with open(result_path, "w", encoding="ascii") as output:
        json.dump(payload, output, indent=2)
        output.write("\n")
    print(f"[POINT] right_knee pixel=({x:.1f}, {y:.1f}) "
          f"camera_xyz_m={payload['camera_xyz_m']} depth={distance_m:.3f}m "
          f"person_conf={box_conf:.3f} knee_conf={knee_conf:.3f}")
    print(f"[OK] preview={preview_path} point={result_path}")
    print("[SAFE] perception preview only; no robot connection or motion")


if __name__ == "__main__":
    main()
