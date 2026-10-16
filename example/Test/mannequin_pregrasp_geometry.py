"""Estimate an outward local surface normal, rejecting unreliable depth patches."""

import numpy as np
import pyrealsense2 as rs


def surface_normal(depth, intrinsics, scale, x, y, center):
    radius = 24
    u, v = int(round(x)), int(round(y))
    h, w = depth.shape
    if min(u, v, w - 1 - u, h - 1 - v) < radius:
        raise ValueError("surface-normal patch touches image boundary")
    points = []
    for row in range(v - radius, v + radius + 1, 2):
        for col in range(u - radius, u + radius + 1, 2):
            z = float(depth[row, col]) * scale
            if 0.35 < z < 3.0 and abs(z - center[2]) < 0.05:
                points.append(rs.rs2_deproject_pixel_to_point(intrinsics, [col, row], z))
    points = np.asarray(points)
    if len(points) < 400:
        raise ValueError("not enough continuous surface around the joint")
    retained = points
    for _ in range(4):
        mean = retained.mean(axis=0)
        _, s, vt = np.linalg.svd(retained - mean, full_matrices=False)
        normal = vt[-1]
        residual = (points - mean) @ normal
        sigma = 1.4826 * np.median(np.abs(residual - np.median(residual)))
        retained = points[np.abs(residual) < max(0.004, min(0.010, 3 * sigma))]
        if len(retained) < 400:
            raise ValueError("surface is discontinuous or insufficiently planar")
    mean = retained.mean(axis=0)
    _, s, vt = np.linalg.svd(retained - mean, full_matrices=False)
    normal = vt[-1]
    rms = float(np.sqrt(np.mean(((retained - mean) @ normal) ** 2)))
    if rms > 0.005 or s[1] / np.sqrt(len(retained)) < 0.012 or s[2] / s[1] > 0.25:
        raise ValueError("surface-normal fit is poorly conditioned")
    if abs(float((np.asarray(center) - mean) @ normal)) > 0.012:
        raise ValueError("joint depth does not lie on fitted surface")
    if np.dot(normal, center) > 0:
        normal = -normal
    incidence = float(np.dot(normal, -np.asarray(center) / np.linalg.norm(center)))
    if incidence < 0.5:
        raise ValueError("surface normal is too oblique to observe reliably")
    return normal, rms
