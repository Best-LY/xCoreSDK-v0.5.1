#!/usr/bin/env python3
"""Locate the knee's ArUco marker in an aligned snapshot; never moves a robot."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from mannequin_joint_preview import load_snapshot


MARKER_ID = 23
MARKER_SIZE_M = 0.100


def find_marker(color):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(color)
    matches = [] if ids is None else [
        corners[i].reshape(4, 2).astype(np.float64)
        for i, marker_id in enumerate(ids.reshape(-1)) if marker_id == MARKER_ID
    ]
    if len(matches) != 1:
        raise ValueError(f"need exactly one ArUco ID {MARKER_ID}, found {len(matches)}")
    points = matches[0]
    edge_lengths = np.linalg.norm(points - np.roll(points, -1, axis=0), axis=1)
    if float(edge_lengths.min()) < 35.0:
        raise ValueError(f"marker is too small in the image: {edge_lengths.min():.1f}px")
    center = points.mean(axis=0)
    x, y = center
    if not (300 <= x <= 850 and 220 <= y <= 570):
        raise ValueError(f"marker is outside the expected knee region: ({x:.1f},{y:.1f})")
    return points, center


def read_marker_depth(depth, center, scale):
    col, row = np.rint(center).astype(int)
    patch = depth[row - 7:row + 8, col - 7:col + 8]
    values = patch[patch > 0].astype(np.float64) * scale
    values = values[(values >= 0.35) & (values <= 3.0)]
    if len(values) < 120:
        raise ValueError(f"insufficient marker depth pixels: {len(values)}/225")
    p10, p90 = np.percentile(values, [10, 90])
    if p90 - p10 > 0.035:
        raise ValueError(f"marker depth is inconsistent: spread={p90-p10:.3f}m")
    return float(np.median(values)), float(p90 - p10), len(values)


def marker_pnp_distance(points, intrinsics):
    half = MARKER_SIZE_M / 2
    square = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                              [0, intrinsics.fy, intrinsics.ppy],
                              [0, 0, 1]], dtype=np.float64)
    distortion = np.asarray(intrinsics.coeffs, dtype=np.float64)
    ok, _, translation = cv2.solvePnP(
        square, points, camera_matrix, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.isfinite(translation).all():
        raise ValueError("marker pose estimation failed")
    return float(translation[2, 0])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--max-age-s", type=float, default=30.0)
    args = parser.parse_args()

    color, depth, intrinsics, scale, captured_ms = load_snapshot(
        args.snapshot, args.max_age_s)
    corners, center = find_marker(color)
    distance_m, spread_m, valid_pixels = read_marker_depth(depth, center, scale)
    pnp_distance_m = marker_pnp_distance(corners, intrinsics)
    if abs(pnp_distance_m - distance_m) > 0.10:
        raise ValueError(f"depth/PnP disagree: {distance_m:.3f}m vs "
                         f"{pnp_distance_m:.3f}m; check printed marker size")
    xyz = rs.rs2_deproject_pixel_to_point(
        intrinsics, [float(v) for v in center], distance_m)
    if not np.isfinite(xyz).all():
        raise ValueError("marker deprojection produced non-finite coordinates")

    preview = color.copy()
    cv2.polylines(preview, [np.rint(corners).astype(np.int32)],
                  True, (0, 200, 0), 3)
    center_pixel = tuple(np.rint(center).astype(int))
    cv2.circle(preview, center_pixel, 8, (0, 0, 255), 3)
    cv2.putText(preview, f"KNEE MARKER  {distance_m:.3f} m",
                (center_pixel[0] + 15, center_pixel[1] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    preview_path = str(args.output_prefix) + "_preview.png"
    point_path = str(args.output_prefix) + "_point.json"
    if not cv2.imwrite(preview_path, preview):
        raise ValueError(f"cannot write preview: {preview_path}")
    payload = {
        "camera_serial": "236422070786",
        "joint": "marked_knee",
        "marker_dictionary": "DICT_4X4_50",
        "marker_id": MARKER_ID,
        "marker_size_m": MARKER_SIZE_M,
        "capture_unix_ms": captured_ms,
        "pixel": [float(v) for v in center],
        "camera_xyz_m": [float(v) for v in xyz],
        "depth_m": distance_m,
        "pnp_depth_m": pnp_distance_m,
        "depth_p10_p90_spread_m": spread_m,
        "valid_depth_pixels": valid_pixels,
        "robot_motion_permitted": False,
    }
    with open(point_path, "w", encoding="ascii") as output:
        json.dump(payload, output, indent=2)
        output.write("\n")
    print(f"[POINT] marker={MARKER_ID} pixel={payload['pixel']} "
          f"camera_xyz_m={payload['camera_xyz_m']} depth={distance_m:.3f}m "
          f"pnp_depth={pnp_distance_m:.3f}m")
    print(f"[OK] preview={preview_path} point={point_path}")
    print("[SAFE] marker preview only; no robot connection or motion")


if __name__ == "__main__":
    try:
        main()
    except ValueError as error:
        raise SystemExit(f"[HOLD] {error}") from None
