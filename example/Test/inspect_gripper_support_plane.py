"""Offline local support-plane diagnostic; never authorizes robot motion."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from validate_full_plan import pose, require, validate_report


def fit_plane(points, viewpoint):
    points = np.asarray(points, dtype=float)
    require(points.ndim == 2 and points.shape[1] == 3 and len(points) >= 500
            and np.isfinite(points).all(), "insufficient/invalid depth points")
    rng = np.random.default_rng(0)
    sample = points[rng.choice(len(points), min(len(points), 5000), replace=False)]
    best = None
    best_count = 0
    for _ in range(128):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b-a, c-a)
        if np.linalg.norm(n) < 1e-6:
            continue
        n /= np.linalg.norm(n)
        mask = np.abs(np.einsum("ij,j->i", sample-a, n)) < .008
        if np.count_nonzero(mask) > best_count:
            best_count = np.count_nonzero(mask)
            best = sample[mask]
    require(best is not None and best_count >= .6*len(sample), "no dominant support plane")
    for _ in range(3):
        center = best.mean(axis=0)
        _, singular, vectors = np.linalg.svd(best-center, full_matrices=False)
        require(singular[1]/np.sqrt(len(best)) > .015, "plane patch lacks two-dimensional extent")
        normal = vectors[-1]
        residuals = np.einsum("ij,j->i", points-center, normal)
        best = points[np.abs(residuals) < .008]
        require(len(best) >= .6*len(points), "insufficient plane inliers")
    center = best.mean(axis=0)
    _, _, vectors = np.linalg.svd(best-center, full_matrices=False)
    normal = vectors[-1]
    if normal @ (np.asarray(viewpoint)-center) < 0:
        normal = -normal
    residual = (best-center) @ normal
    return normal, center, {
        "valid_depth_points": len(points), "inlier_points": len(best),
        "inlier_fraction": len(best)/len(points),
        "plane_rms_m": float(np.sqrt(np.mean(residual**2))),
        "plane_inlier_abs_residual_p95_m": float(np.percentile(np.abs(residual), 95)),
    }


def tip_clearance(final_pose, target, geometry, normal, center):
    final_pose = pose(np.asarray(final_pose).ravel())
    target, normal, center = [np.asarray(x, dtype=float) for x in (target, normal, center)]
    require(all(x.shape == (3,) and np.isfinite(x).all() for x in (target, normal, center))
            and abs(np.linalg.norm(normal)-1) < 1e-6, "invalid plane/target")
    width, thickness, extension = [float(geometry[k]) for k in (
        "maximum_open_outer_width_m", "maximum_thickness_m",
        "finger_tip_extension_beyond_grasp_center_m")]
    require(all(np.isfinite(v) and 0 < v <= .5 for v in (width, thickness, extension)),
            "invalid reported gripper dimensions")
    axis = final_pose[:3, 2]
    tip_axis_point = target + extension*axis
    # Use a circumscribed disk because the finger closing-axis mounting angle
    # is not fully measured. This bounds transverse offsets, not finger solids.
    radius = np.hypot(width/2, thickness/2)
    variation = radius*np.linalg.norm(final_pose[:3, :2].T @ normal)
    height = float((tip_axis_point-center) @ normal)
    return {
        "grasp_center_signed_height_m": float((target-center) @ normal),
        "finger_tip_axis_signed_height_m": height,
        "finger_tip_height_range_m": [height-float(variation), height+float(variation)],
        "approach_axis_normal_dot": float(axis @ normal),
        "transverse_bound_radius_m": float(radius),
    }


def inspect(plan_path, geometry_path, roi):
    plan = validate_report(plan_path)
    geometry = json.loads(Path(geometry_path).read_text())
    require(geometry["arm"] == "left" and geometry["grasp_center_on_flange_positive_z"] is True
            and abs(geometry["flange_to_grasp_center_m"]-.3) < 1e-9, "tool frame does not match plan")
    capture = cv2.FileStorage(plan["capture_file"], cv2.FILE_STORAGE_READ)
    require(capture.isOpened(), "capture missing")
    try:
        require(capture.getNode("camera_serial").string() == plan["camera_serial"], "camera mismatch")
        coefficients = capture.getNode("distortion_coeffs").mat()
        require(coefficients is not None and np.isfinite(coefficients).all()
                and np.max(np.abs(coefficients)) < 1e-12, "nonzero distortion needs calibrated deprojection")
        scale = capture.getNode("depth_scale_m").real()
        require(np.isfinite(scale) and 0 < scale < .1, "invalid depth scale")
        raw = cv2.imread(capture.getNode("depth_file").string(), cv2.IMREAD_UNCHANGED)
        require(raw is not None and raw.ndim == 2 and raw.dtype == np.uint16, "depth image missing/invalid")
        fx, fy, cx, cy = [capture.getNode(k).real() for k in ("fx", "fy", "ppx", "ppy")]
        require(np.isfinite([fx, fy, cx, cy]).all() and fx > 0 and fy > 0, "invalid intrinsics")
        captured_pose = pose(capture.getNode("base_T_flange").mat().ravel())
    finally:
        capture.release()
    require(np.max(np.abs(captured_pose-pose(plan["stages"][0]["start_flange_base"]))) < .005,
            "capture/plan observation pose differs")
    x0, y0, x1, y1 = roi
    require(0 <= x0 < x1 <= raw.shape[1] and 0 <= y0 < y1 <= raw.shape[0], "invalid ROI")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    z = raw[yy, xx]*scale
    points = np.column_stack(((xx-cx).ravel()*z.ravel()/fx,
                              (yy-cy).ravel()*z.ravel()/fy, z.ravel()))
    points = points[(points[:, 2] >= .35) & (points[:, 2] <= 3.)]
    base_camera = captured_pose @ pose(plan["flange_T_camera"])
    points = points @ base_camera[:3, :3].T + base_camera[:3, 3]
    normal, center, fit = fit_plane(points, base_camera[:3, 3])
    clearance = tip_clearance(plan["stages"][1]["end_flange_base"],
                              plan.get("grasp_target_base_m", plan["target_point_base_m"]), geometry, normal, center)
    return {
        "schema_version": 1, "plan_file": str(Path(plan_path).resolve()),
        "capture_unix_ms": plan["capture_unix_ms"], "roi_xyxy": list(roi),
        "surface_insertion_m": plan.get("surface_insertion_m", 0.),
        "normal_base": normal.tolist(), "plane_center_base_m": center.tolist(),
        **fit, **clearance,
        "status": ("potential_support_plane_intersection" if clearance["finger_tip_height_range_m"][0] < 0
                   else "no_intersection_with_this_fitted_plane"),
        "local_plane_extrapolated_to_tool": True,
        "exact_finger_solids_known": False, "collision_clearance_established": False,
        "robot_motion_permitted": False,
        "limitations": "Selected visible plane only; no occluded surfaces, exact finger cavity, full-arm or other-arm model. Not motion authorization.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("--geometry", type=Path, default=Path(__file__).resolve().parents[1]/"function_config/left_gripper_geometry.json")
    parser.add_argument("--roi", nargs=4, type=int, required=True, metavar=("X0", "Y0", "X1", "Y1"))
    args = parser.parse_args()
    try:
        print(json.dumps(inspect(args.plan, args.geometry, args.roi), indent=2, allow_nan=False))
    except (ValueError, TypeError, KeyError, OSError, cv2.error) as error:
        parser.exit(2, f"[HOLD] {error}\n")
