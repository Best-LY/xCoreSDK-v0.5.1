"""Offline validation of reach-first candidates; never connects to hardware."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from validate_full_plan import pose, require, vector


def validate_manifest(p):
    require(p["schema_version"] == 1 and p["plan_kind"] == "reach_surface_offset"
            and p["status"] == "kinematic_plan_complete", "invalid reach plan")
    require(p["robot_motion_permitted"] == 0 and p["collision_validation"] == "not_performed"
            and p["controller_trajectory_acceptance"] == "not_tested", "false execution/clearance claim")
    require(p["robot_arm"] == "left" and p["joint"] == "left_elbow"
            and p["gripper_command"] == "none" and p["observation_rounds"] == 1, "wrong task scope")
    require(p["fixed_pregrasp_required"] == 0 and p["insertion_direction"] == "approach_positive_flange_z"
            and abs(p["surface_insertion_m"]-.05) < 1e-9, "wrong requested insertion")
    stages = p["stages"]
    require(1 <= len(stages) <= 2, "unexpected leg count")
    for i, stage in enumerate(stages):
        require(stage["command"] in ("MoveAbsJ", "MoveL") and stage["stop_at_endpoint"] == 1,
                "invalid leg command/boundary")
        require(type(stage["sample_count"]) is int and stage["sample_count"] >= 3, "invalid sample count")
        a, b = vector(stage["start_joints_rad"], 6), vector(stage["end_joints_rad"], 6)
        start, end = pose(stage["start_flange_base"]), pose(stage["end_flange_base"])
        speed = stage["joint_speed_fraction"] if stage["command"] == "MoveAbsJ" else stage["linear_speed_mm_s"]
        require(np.isfinite(speed) and 0 < speed <= (.01 if stage["command"] == "MoveAbsJ" else 5), "invalid speed")
        if i:
            require(np.max(np.abs(a-vector(stages[i-1]["end_joints_rad"], 6))) < 1e-9
                    and np.max(np.abs(start-pose(stages[i-1]["end_flange_base"]))) < 1e-8,
                    "discontinuous handover")
    surface = vector(p["surface_point_base_m"], 3)
    center = vector(p["grasp_target_base_m"], 3)
    tool = vector(p["tool_offset_flange_m"], 3)
    require(np.linalg.norm(tool-[0, 0, .3]) < 1e-9, "tool offset changed")
    require(np.linalg.norm(center-surface-.05*end[:3, 2]) < .001, "grasp center is not surface +50 mm")
    require(np.linalg.norm(end[:3, 3]+end[:3, :3]@tool-center) < .0025, "flange/grasp target mismatch")
    feasible = [c for c in p["candidates"] if c["passed"] == 1]
    require(feasible and len({c["name"] for c in p["candidates"]}) == len(p["candidates"]), "missing/duplicate candidates")
    for c in feasible:
        require(np.isfinite([c["grasp_path_length_m"], c["joint_travel_rad"]]).all()
                and c["grasp_path_length_m"] >= 0 and c["joint_travel_rad"] >= 0, "invalid route cost")
    selected = [c for c in feasible if c["name"] == p["selected_candidate"]]
    require(len(selected) == 1, "selected route is not feasible")
    selected = selected[0]
    require(all(selected["grasp_path_length_m"] <= c["grasp_path_length_m"]+1e-9 for c in feasible),
            "selected path is not shortest among feasible candidates")
    require(abs(p["sampled_grasp_path_length_m"]-selected["grasp_path_length_m"]) < 1e-9
            and abs(p["sampled_joint_travel_rad"]-selected["joint_travel_rad"]) < 1e-9, "selected score mismatch")
    return stages


def validate(path):
    p = json.loads(Path(path).read_text())
    stages = validate_manifest(p)
    lines = Path(p["sample_csv"]).read_text().splitlines()
    require(lines and lines[-1].startswith("# COMPLETE"), "partial trajectory")
    rows = list(csv.DictReader(x for x in lines if not x.startswith("#")))
    offset, length, travel = 0, 0., 0.
    tool = vector(p["tool_offset_flange_m"], 3)
    for i, stage in enumerate(stages):
        samples = rows[offset:offset+stage["sample_count"]]
        require(len(samples) == stage["sample_count"] and all(int(r["leg"]) == i for r in samples), "sample order/count")
        q = np.array([[float(r[f"q{j}"]) for j in range(1, 7)] for r in samples])
        grasp = np.array([[float(r[f"grasp_{c}"]) for c in "xyz"] for r in samples])
        matrices = np.array([pose([float(r[f"flange_{j}"]) for j in range(16)]) for r in samples])
        require(np.isfinite(q).all() and np.isfinite(grasp).all(), "nonfinite route")
        t = np.array([float(r["fraction"]) for r in samples])
        require(np.allclose(t, np.linspace(0, 1, len(samples)), atol=1e-9, rtol=0), "sample fraction mismatch")
        require(np.max(np.abs(q[0]-stage["start_joints_rad"])) < 1e-8
                and np.max(np.abs(q[-1]-stage["end_joints_rad"])) < 1e-8, "joint endpoints differ")
        require(np.max(np.abs(matrices[0]-pose(stage["start_flange_base"]))) < 1e-8
                and np.max(np.abs(matrices[-1]-pose(stage["end_flange_base"]))) < 1e-8, "pose endpoints differ")
        require(np.max(np.abs(np.diff(q, axis=0))) <= .12, "joint branch discontinuity")
        expected_grasp = matrices[:, :3, 3]+np.einsum("nij,j->ni", matrices[:, :3, :3], tool)
        require(np.max(np.linalg.norm(grasp-expected_grasp, axis=1)) < 1e-8, "sampled grasp offset mismatch")
        if stage["command"] == "MoveAbsJ":
            require(np.max(np.abs(q-(q[0]+t[:, None]*(q[-1]-q[0])))) < 1e-8, "joint path interpolation mismatch")
        else:
            nominal = matrices[0, :3, 3]+t[:, None]*(matrices[-1, :3, 3]-matrices[0, :3, 3])
            require(np.max(np.linalg.norm(matrices[:, :3, 3]-nominal, axis=1)) < .001, "Cartesian line deviation")
        length += np.linalg.norm(np.diff(grasp, axis=0), axis=1).sum()
        travel += np.abs(np.diff(q, axis=0)).sum()
        offset += len(samples)
    require(offset == len(rows), "extra samples")
    require(abs(length-p["sampled_grasp_path_length_m"]) < 1e-8
            and abs(travel-p["sampled_joint_travel_rad"]) < 1e-8, "sample/manifest route cost mismatch")
    return p


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    args = parser.parse_args()
    try:
        result = validate(args.plan)
        print(f"[OK] {result['selected_candidate']}: grasp surface +50 mm, {len(result['stages'])} stage(s), "
              f"sampled grasp path {result['sampled_grasp_path_length_m']:.4f} m")
        print("[HOLD] kinematic candidate only; no collision certificate or hardware execution permission")
    except (ValueError, KeyError, TypeError, OSError, IndexError) as error:
        parser.exit(2, f"[ERROR] {error}\n")
