#!/usr/bin/env python3
"""Capture a left-camera mannequin joint and plan a guarded approach."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

from mannequin_joint_preview import choose_joint, knee_depth, load_snapshot
from mannequin_pregrasp_geometry import surface_normal


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--joint", choices=("left_shoulder", "left_elbow"),
                        default="left_shoulder")
    parser.add_argument("--plan-full", action="store_true",
                        help="check a complete non-contact approach without motion")
    parser.add_argument("--approach-full", action="store_true",
                        help="complete straight approach; requires --execute and onsite path clearance")
    parser.add_argument("--pregrasp", action="store_true",
                        help="orient provisional flange-Z tool toward the local surface; dry run unless --execute")
    parser.add_argument("--demonstrated-pregrasp", action="store_true",
                        help="use the saved successful manual orientation, with a fresh visual target")
    parser.add_argument("--grasp-plan", action="store_true",
                        help="read-only: align provisional grasp center with the fresh joint point, zero standoff")
    parser.add_argument("--staged-plan", action="store_true",
                        help="read-only: joint transit to 0.40 m pregrasp, then fixed-orientation linear approach")
    parser.add_argument("--reach-plan", action="store_true",
                        help="read-only: compare reachable routes to surface +50 mm; no fixed pregrasp")
    parser.add_argument("--grasp-align", action="store_true",
                        help="read-only zero-standoff alignment using --tool-length-mm 300; execution disabled after collision")
    parser.add_argument("--tool-length-mm", type=float,
                        help="explicit measured flange-to-grasp-center length for --grasp-align")
    parser.add_argument("--motion", choices=("joint", "linear"), default=None,
                        help="pregrasp motion; defaults to joint for demonstrated orientation, linear otherwise")
    parser.add_argument("--step-mm", type=float, choices=(0.25, 0.5, 1.0, 2.0), default=2.0)
    parser.add_argument("--output-prefix", type=Path,
                        default=Path("/tmp/xmc_left_shoulder_probe"))
    args = parser.parse_args(argv)
    if args.reach_plan:
        if (args.execute or args.staged_plan or args.grasp_plan or args.grasp_align or args.pregrasp or
                args.demonstrated_pregrasp or args.plan_full or args.approach_full or
                args.motion is not None or args.tool_length_mm is not None or args.joint != "left_elbow"):
            parser.error("--reach-plan requires --joint left_elbow, is read-only and excludes other modes")
        args.demonstrated_pregrasp = True
    if args.staged_plan:
        if (args.execute or args.grasp_plan or args.grasp_align or args.pregrasp or
                args.demonstrated_pregrasp or args.plan_full or args.approach_full or
                args.motion is not None or args.tool_length_mm is not None):
            parser.error("--staged-plan is read-only and selects both motion stages; no other mode/--execute allowed")
        args.demonstrated_pregrasp = True
    if args.grasp_align:
        if args.execute:
            parser.error("grasp alignment execution disabled after unintended contact; collision-aware path review required")
        if args.grasp_plan or args.pregrasp or args.demonstrated_pregrasp or args.plan_full or args.approach_full:
            parser.error("--grasp-align excludes other approach modes")
        if args.tool_length_mm != 300.0:
            parser.error("--grasp-align requires the measured --tool-length-mm 300")
        args.demonstrated_pregrasp = True
    elif args.tool_length_mm is not None:
        parser.error("--tool-length-mm requires --grasp-align")
    if args.grasp_plan:
        if args.execute or args.pregrasp or args.demonstrated_pregrasp or args.plan_full or args.approach_full:
            parser.error("--grasp-plan is read-only and excludes all other approach modes and --execute")
        args.demonstrated_pregrasp = True
    if args.plan_full and args.execute:
        parser.error("--plan-full is read-only; cannot combine with --execute")
    if args.approach_full and (args.plan_full or not args.execute):
        parser.error("--approach-full requires --execute and excludes --plan-full")
    if (args.pregrasp or args.demonstrated_pregrasp) and (args.plan_full or args.approach_full):
        parser.error("--pregrasp excludes the fixed-orientation approach modes")
    if args.pregrasp and args.demonstrated_pregrasp:
        parser.error("choose only one pregrasp orientation source")
    if args.motion is not None and not (args.pregrasp or args.demonstrated_pregrasp):
        parser.error("--motion requires a pregrasp mode")
    args.motion = args.motion or ("joint" if args.demonstrated_pregrasp else "linear")
    if args.joint != "left_shoulder" and not (args.pregrasp or args.demonstrated_pregrasp):
        parser.error("left_elbow requires a pregrasp mode; legacy micro-step mode is shoulder-only")
    return args


def main():
    args = parse_args()
    joint_index = {"left_shoulder": 5, "left_elbow": 7}[args.joint]
    project = Path(__file__).resolve().parents[2]
    model = YOLO("/home/arm/.local/share/xmc-pose-models/yolo11m-pose.pt")
    prefix = Path(str(args.output_prefix) + "_" + str(time.time_ns()))
    subprocess.run([str(project / "build/bin/mannequin_snapshot"),
                    "--robot-arm", "left", "--burst", "10",
                    "--output-prefix", str(prefix)], check=True)
    storage = cv2.FileStorage(str(prefix) + "_000.yml", cv2.FileStorage_READ)
    try:
        if storage.getNode("robot_arm").string() != "left":
            raise ValueError("snapshot is not paired with the left arm")
        joint_matrix = storage.getNode("joint_positions_rad").mat()
        if joint_matrix is None or joint_matrix.size != 6 or not np.isfinite(joint_matrix).all():
            raise ValueError("paired capture joints are missing or invalid")
        capture_joints = joint_matrix.ravel().tolist()
    finally:
        storage.release()
    samples = []
    normals = []
    fit_errors = []
    timestamps = []
    for index in range(10):
        image, depth, intrinsics, scale, captured_ms = load_snapshot(
            Path(f"{prefix}_{index:03d}.yml"), 15, "236422072262")
        timestamps.append(captured_ms)
        result = model.predict(image, imgsz=960, conf=0.2,
                               device=0, verbose=False)[0]
        try:
            person, _, confidence, x, y, _ = choose_joint(result, image.shape, joint_index)
            if person < 0.5 or confidence < 0.9:
                raise ValueError(f"{args.joint} confidence below commissioning limit")
            z, spread, _ = knee_depth(depth, x, y, scale)
            xyz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], z)
            if not np.all(np.isfinite(xyz)):
                raise ValueError(f"invalid {args.joint} 3D point")
            if args.pregrasp:
                normal, rms = surface_normal(depth, intrinsics, scale, x, y, xyz)
                normals.append(normal)
                fit_errors.append(rms)
            samples.append((x, y, z, person, confidence, spread, xyz))
        except ValueError as error:
            print(f"[HOLD] frame {index}: {error}", flush=True)
    if len(samples) < 8:
        raise ValueError(f"need at least 8/10 reliable {args.joint} frames; got {len(samples)}")
    points = np.asarray([sample[6] for sample in samples])
    median_xyz = np.median(points, axis=0)
    p90 = float(np.percentile(np.linalg.norm(points - median_xyz, axis=1), 90))
    if p90 > 0.02:
        raise ValueError(f"{args.joint} P90 variation {p90:.4f} m exceeds 0.020 m")
    x, y, z = np.median(np.asarray([sample[:3] for sample in samples]), axis=0)
    person = min(sample[3] for sample in samples)
    confidence = min(sample[4] for sample in samples)
    spread = max(sample[5] for sample in samples)
    payload = {
        "camera_serial": "236422072262", "joint": args.joint,
        "coco_index": joint_index,
        "capture_unix_ms": min(timestamps), "pixel": [float(x), float(y)],
        "depth_m": float(z), "camera_xyz_m": median_xyz.tolist(),
        "person_confidence": person, "keypoint_confidence": confidence,
        "depth_p10_p90_spread_m": spread, "valid_frames": len(samples),
        "position_p90_deviation_m": p90, "robot_motion_permitted": False,
        "capture_joints_rad": capture_joints,
    }
    point_path = Path(str(prefix) + "_point.json")
    point_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="ascii")
    print(f"[POINT] joint={args.joint} {point_path} depth={z:.4f} m p90={p90*1000:.2f} mm", flush=True)
    if args.pregrasp or args.demonstrated_pregrasp:
        if args.pregrasp:
            normal = np.median(np.asarray(normals), axis=0)
            normal /= np.linalg.norm(normal)
            angles = np.degrees(np.arccos(np.clip(np.asarray(normals) @ normal, -1, 1)))
            angle90 = float(np.percentile(angles, 90))
            if angle90 > 8.0:
                raise ValueError(f"surface-normal P90 variation {angle90:.2f} deg exceeds 8 deg")
        geometry_path = Path(str(prefix) + "_pregrasp.yml")
        fs = cv2.FileStorage(str(geometry_path), cv2.FileStorage_WRITE)
        if not fs.isOpened():
            raise ValueError("cannot write pregrasp geometry")
        try:
            fs.write("capture_file", str(prefix) + "_000.yml")
            fs.write("camera_serial", "236422072262")
            fs.write("joint", args.joint)
            fs.write("coco_index", joint_index)
            fs.write("point_C", median_xyz.reshape(3, 1))
            fs.write("orientation_source", "demonstrated" if args.demonstrated_pregrasp else "surface_normal")
            if args.grasp_align:
                fs.write("measured_tool_length_m", args.tool_length_mm / 1000.0)
            fs.write("point_p90_m", p90)
            fs.write("person_confidence", person)
            fs.write("keypoint_confidence", confidence)
            fs.write("depth_spread_m", spread)
            if args.pregrasp:
                fs.write("outward_normal_C", normal.reshape(3, 1))
                fs.write("normal_p90_deg", angle90)
                fs.write("fit_rms_max_m", max(fit_errors))
            fs.write("valid_frames", len(samples))
        finally:
            fs.release()
        if args.pregrasp:
            print(f"[NORMAL] outward_C={normal.tolist()} P90={angle90:.2f} deg", flush=True)
        else:
            print("[ORIENTATION] saved manual demonstration, not a fitted cloth normal", flush=True)
        if args.grasp_plan:
            print("[GRASP-PLAN] zero target standoff; provisional TCP; no contact/motion permission", flush=True)
        if args.grasp_align:
            print(f"[GRASP-ALIGN] measured flange-Z tool={args.tool_length_mm} mm; zero target standoff; NO gripper IO", flush=True)
        motion_label = ("reach-first candidate search, surface +50 mm" if args.reach_plan else
                        "joint + fixed-orientation linear" if args.staged_plan else args.motion)
        print(f"[MOTION] {motion_label}; {'execution requested' if args.execute else 'read-only planning'}", flush=True)
        if args.reach_plan:
            planner_mode = "--reach-plan"
        elif args.staged_plan:
            planner_mode = "--staged-plan"
        elif args.grasp_align:
            planner_mode = "--joint-grasp-align" if args.motion == "joint" else "--linear-grasp-align"
        elif args.grasp_plan:
            planner_mode = "--joint-grasp-plan" if args.motion == "joint" else "--linear-grasp-plan"
        else:
            planner_mode = "--joint-pregrasp" if args.motion == "joint" else "--pregrasp"
        command = [str(project / "build/bin/mannequin_approach_plan"),
                   str(prefix) + "_000.yml", planner_mode, str(geometry_path)]
        if args.execute:
            command.append("--execute")
        return subprocess.run(command, check=False).returncode
    if args.plan_full or args.approach_full:
        command = [str(project / "build/bin/mannequin_approach_plan"),
                   str(prefix) + "_000.yml"] + [str(v) for v in median_xyz]
        if args.approach_full:
            command.append("--execute")
        return subprocess.run(command, check=False).returncode
    command = [str(project / "build/bin/mannequin_micro_approach"),
               "--left-shoulder", "--step-mm", str(args.step_mm),
               "--camera-serial", "236422072262", "--joint", "left_shoulder",
               "--capture-ms", str(min(timestamps)),
               "--pixel-x", str(x), "--pixel-y", str(y), "--depth-m", str(z),
               "--person-conf", str(person), "--knee-conf", str(confidence),
               "--spread-m", str(spread)]
    command += ["--expected-joints"] + [str(q) for q in capture_joints]
    if args.execute:
        command.append("--execute")
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"[HOLD] {error}", flush=True)
        raise SystemExit(2)
