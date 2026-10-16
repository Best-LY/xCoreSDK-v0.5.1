"""Offline consistency checks for a two-stage plan. No robot/camera imports."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def require(condition, message):
    if not condition:
        raise ValueError(message)


def vector(value, size):
    result = np.asarray(value, dtype=float)
    require(result.shape == (size,) and np.isfinite(result).all(), "invalid plan vector")
    return result


def pose(value):
    result = vector(value, 16).reshape(4, 4)
    rotation = result[:3, :3]
    require(np.allclose(result[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            and np.linalg.norm(rotation.T @ rotation - np.eye(3)) < 1e-6
            and abs(np.linalg.det(rotation) - 1) < 1e-6, "invalid rigid pose")
    return result


def validate_manifest(plan):
    diagnostic = plan.get("diagnostic_replay", 0)
    require(diagnostic in (0, 1), "invalid diagnostic marker")
    require(plan["schema_version"] in (1, 2, 3)
            and plan["status"] == ("diagnostic_kinematic_complete" if diagnostic else "kinematic_plan_complete"),
            "incomplete/mislabeled plan")
    require(not diagnostic or plan["schema_version"] >= 2, "invalid diagnostic schema")
    require(plan["robot_motion_permitted"] == 0 and plan["collision_validation"] == "not_performed",
            "plan must not claim execution permission or collision clearance")
    require(plan["observation_rounds"] == (0 if diagnostic else 1) and plan["reidentify_between_stages"] == 0,
            "plan must use a single locked observation")
    require(plan["robot_arm"] == "left" and plan["gripper_command"] == "none", "unexpected actuator scope")
    local_count = plan["local_ik_sample_count"]
    require(isinstance(local_count, int) and local_count >= 0, "invalid local IK count")
    if plan["schema_version"] == 1:
        require(plan["controller_linear_ik_all_passed"] == int(local_count == 0),
                "controller/model IK status mismatch")
    else:
        require(plan["controller_linear_ik_all_passed"] == 0
                and plan["controller_linear_ik_queries"] == 0
                and plan["controller_linear_ik_status"] == "not_queried",
                "seeded model planning must not claim controller verification")
        require(plan["linear_ik_solver"] == "seeded_local_sdk"
                and plan["linear_ik_seed_policy"] == "stage1_endpoint_then_previous_sample"
                and plan["boundary_ik_round_trip_passed"] == 1, "invalid continuous IK seed contract")
    stages = plan["stages"]
    require(len(stages) == 2, "expected exactly two stages")
    first, second = stages
    if plan["schema_version"] >= 2:
        require(local_count == second["sample_count"] - 1, "not all linear samples used seeded IK")
    require(first["name"] == "joint_transit" and first["command"] == "MoveAbsJ"
            and second["name"] == "linear_approach" and second["command"] == "MoveL", "incorrect stage order")
    for stage in stages:
        vector(stage["start_joints_rad"], 6)
        vector(stage["end_joints_rad"], 6)
        pose(stage["start_flange_base"])
        pose(stage["end_flange_base"])
        require(stage["stop_at_endpoint"] == 1 and stage["sample_count"] >= 2, "missing stage boundary")
    require(np.max(np.abs(vector(first["end_joints_rad"], 6)
                          - vector(second["start_joints_rad"], 6))) < 1e-9, "joint discontinuity at handover")
    a = pose(first["end_flange_base"])
    b = pose(second["start_flange_base"])
    end = pose(second["end_flange_base"])
    require(np.max(np.abs(a - b)) < 1e-9, "pose discontinuity at handover")
    require(second["fixed_orientation"] == 1 and np.max(np.abs(b[:3, :3] - end[:3, :3])) < 1e-9,
            "linear approach changes orientation")
    tool = vector(plan["tool_offset_flange_m"], 3)
    point = vector(plan["target_point_base_m"], 3)
    insertion = 0.0
    grasp_target = point
    if plan["schema_version"] == 3:
        insertion = float(plan["surface_insertion_m"])
        require(np.isfinite(insertion) and 0 <= insertion <= .080, "invalid surface insertion")
        require(plan["target_point_semantics"] == "observed_surface"
                and plan["insertion_direction"] == "approach_positive_flange_z", "ambiguous surface/grasp frames")
        grasp_target = vector(plan["grasp_target_base_m"], 3)
        require(np.linalg.norm(grasp_target-point-insertion*end[:3, 2]) < 1e-8,
                "surface-to-grasp insertion mismatch")
        require(abs(second["approach_distance_m"]-(.4+insertion)) < 1e-9,
                "approach distance omits/doubles insertion")
    require(np.linalg.norm(tool - [0, 0, .3]) < 1e-9, "wrong measured tool offset")
    require(abs(first["end_gap_m"] - .4) < 1e-9 and abs(second["start_gap_m"] - .4) < 1e-9
            and abs(second["end_gap_m"]+insertion) < 1e-9, "incorrect approach gaps")
    require(np.linalg.norm(a[:3, 3] + a[:3, :3] @ tool - (point - .4 * end[:3, 2])) < 1e-8,
            "pregrasp does not reference the locked target")
    require(np.linalg.norm(end[:3, 3] + end[:3, :3] @ tool - grasp_target) < 1e-8,
            "final grasp center misses the locked target")
    pose(plan["flange_T_camera"])
    pose(plan["world_T_base"])
    return stages


def validate_report(path):
    plan = json.loads(Path(path).read_text())
    stages = validate_manifest(plan)
    lines = Path(plan["sample_csv"]).read_text().splitlines()
    require(bool(lines) and lines[-1].startswith("# COMPLETE"), "partial trajectory CSV")
    rows = list(csv.DictReader(line for line in lines if not line.startswith("#")))
    offset = 0
    for stage in stages:
        samples = rows[offset:offset + stage["sample_count"]]
        require(len(samples) == stage["sample_count"] and all(r["stage"] == stage["name"] for r in samples),
                "CSV stage count/order mismatch")
        joints = np.asarray([[float(r[f"q{i}"]) for i in range(1, 7)] for r in samples])
        require(np.isfinite(joints).all(), "nonfinite sampled joints")
        require(np.max(np.abs(joints[0] - stage["start_joints_rad"])) < 1e-8
                and np.max(np.abs(joints[-1] - stage["end_joints_rad"])) < 1e-8, "CSV joint endpoints mismatch")
        require(np.max(np.abs(np.diff(joints, axis=0))) <= .12, "sampled joint discontinuity")
        fractions = np.array([float(r["fraction"]) for r in samples])
        require(np.allclose(fractions, np.linspace(0, 1, len(samples)), atol=1e-9, rtol=0),
                "missing/reordered samples")
        flange = np.array([[float(r[f"flange_{axis}"]) for axis in "xyz"] for r in samples])
        grasp = np.array([[float(r[f"grasp_{axis}"]) for axis in "xyz"] for r in samples])
        axes = np.array([[float(r[f"axis_{axis}"]) for axis in "xyz"] for r in samples])
        require(np.isfinite(flange).all() and np.isfinite(grasp).all() and np.isfinite(axes).all(),
                "nonfinite Cartesian sample")
        require(np.max(np.linalg.norm(grasp - flange - .3 * axes, axis=1)) < 1e-8,
                "sampled tool offset mismatch")
        start_pose, end_pose = pose(stage["start_flange_base"]), pose(stage["end_flange_base"])
        require(np.linalg.norm(flange[0] - start_pose[:3, 3]) < .001
                and np.linalg.norm(flange[-1] - end_pose[:3, 3]) < .001, "CSV pose endpoint mismatch")
        if stage["name"] == "linear_approach":
            nominal = start_pose[:3, 3] + fractions[:, None] * (end_pose[:3, 3] - start_pose[:3, 3])
            require(np.max(np.linalg.norm(flange - nominal, axis=1)) < .001,
                    "linear approach departs from planned line")
            require(np.max(np.linalg.norm(axes - end_pose[:3, 2], axis=1)) < .005,
                    "linear approach tool axis changes")
        offset += len(samples)
    require(offset == len(rows), "unexpected extra CSV stage")
    return plan


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    args = parser.parse_args()
    try:
        result = validate_report(args.plan)
        if result.get("diagnostic_replay", 0):
            print("[OK] historical IK replay consistent; NOT a current scene plan")
        else:
            print("[OK] one observation, two connected stages, locked target and CSV consistent")
        print(f"[HOLD] collision validation={result['collision_validation']}; no execution permission")
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(2, f"[ERROR] {error}\n")
