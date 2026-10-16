# Before Manual Positioning

Recorded on 2026-10-06 (Asia/Shanghai) at the user's request, before manual motion.
This is a reference checkpoint, not a new home pose or a calibration result.

- `left.yml`: left-arm joints and flange pose, with paired RGB/depth images.
- `right.yml`: right-arm joints and flange pose, with paired RGB/depth images.
- Joint angles are in radians; transform translations are in meters.
- `base_T_flange` maps flange coordinates into that robot's base coordinates.
- Each capture has its own timestamp. The two arms were captured sequentially.
- Both arms were idle and stationary during their respective captures.
- Capture was read-only: no power, mode, drag, gripper, or motion commands.

The left arm had returned to the start of the preceding straight approach.
The subsequent surface-normal pregrasp was NOT executed: controller endpoint
IK returned error -32.

Provisional left grasp-center offset, confirmed by the user:
`[0, 0, 0.35]` meters in the flange frame, along flange +Z, coaxial.
This is a rough measurement, NOT a calibrated TCP, and is NOT the camera offset.

Do not replay these poses automatically. Recheck current robot state, base and
tool mounting, scene clearance, and the full path before any return motion.
