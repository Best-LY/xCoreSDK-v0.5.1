# Left Hand-Eye Three-Pose Static Validation

Camera: 236422072262. Board: 8 x 6 inner corners, 0.0286 m squares.
Calibration snapshot: calibration_used.yml. No motion commands issued.

pose_a_000.yml through pose_a_009.yml contain paired RGB-D paths, intrinsics,
base_T_flange, joint positions and capture timestamps. Capture checks idle
state and joint changes no greater than 0.0005 rad throughout the burst.
This is stationary bracketing, not hardware-synchronized acquisition.

All 10 images passed complete-board SB detection and the static image/depth
checks. Reprojection RMS is approximately 0.36 pixels. Median measured-minus-
PnP corner depth difference is approximately -0.014 m.

Using the stored calibration, the median board center in base coordinates is
[0.7219958647, -0.5830973337, 1.7755869921] m. Within-burst P90 scatter is
0.0001064 m. This measures static repeatability, NOT calibration accuracy.

The preceding /tmp/xmc_left_board_clear_20260930 images have no paired robot
poses and MUST NOT be used as a cross-pose hand-eye validation baseline.

## Second Pose

pose_b_000.yml through pose_b_009.yml contain the second stationary burst.
All 10 frames passed the same static image/depth checks. Median reprojection
RMS: 0.5634 pixels. P90 within-burst center scatter: 0.1537 mm.
Median measured-minus-PnP corner depth difference: approximately -0.011 m.

Board center in base coordinates: [0.7251130562, -0.5881110446, 1.7715288124] m.
Difference from pose_a: [3.1172, -5.0137, -4.0582] mm; norm: 7.1640 mm.
Flange translation changed 457.7584 mm and rotation changed 24.8089 degrees.
These figures assume the board and camera mounting remained physically fixed.

An independent depth-based check (mean of 48 deprojected corners) gives:
- pose_a: [0.7136993519, -0.5786019156, 1.7651037425] m.
- pose_b: [0.7166911132, -0.5832514580, 1.7665881268] m.

## Third Pose And Comparison

pose_c_000.yml through pose_c_009.yml contain the third stationary burst.
All 10 frames passed the same checks. Median reprojection RMS: 0.3853 pixels.
P90 within-burst center scatter: 0.2949 mm. Corner depth residual: about -15 mm.
Board center in base coordinates: [0.7280307950, -0.5867751581, 1.7755075711] m.
Depth-based center: [0.7169279029, -0.5808335741, 1.7671740824] m.

| Pair | PnP center difference (mm) | Depth center difference (mm) | Flange translation (mm) | Flange rotation (deg) |
| --- | --- | --- | --- | --- |
| a-b | 7.1640 | 5.7247 | 457.7584 | 24.8089 |
| a-c | 7.0677 | 4.4374 | 595.2521 | 38.9762 |
| b-c | 5.1116 | 2.4991 | 375.8995 | 49.5485 |

The a-to-b and a-to-c spatial rotation axes have an acute separation of
79.7615 degrees. Stacking (R_b - R_a) and (R_c - R_a) gives singular values
[0.793575, 0.674384, 0.418292], condition 1.89718: translation observability
is not degenerate for this set. Estimated board normal differences are
0.4585, 0.2494, 0.3540 degrees for a-b, a-c, b-c respectively.

Conclusion: good static cross-pose consistency over these tested poses,
conditional on a fixed board and rigid camera mount. These measurements do
not establish absolute accuracy, gripper-tip clearance, collision-free paths,
or safety of automatic grasping. The depth and PnP calculations share RGB
corner detections and are not fully independent measurements.

The original calibration is unchanged. No axis-verified file was created.
Next stage is guarded, low-speed, non-contact directional validation after
current-state and physical-clearance checks, not further identical static
captures. No robot mode, power or motion was commanded during these tests.
