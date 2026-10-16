# Manual Elbow Pose Analysis

2026-10-06. The user manually positioned the left gripper approximately at the
mannequin's left elbow. Captures and IK comparisons were READ-ONLY.

`left.yml` stores this pose, joint angles in radians and flange-in-base in meters,
with companion RGB/depth images. `ik_comparison.csv` contains hypothetical
endpoint IK queries, not motion commands, path validation, or motion approval.

The failed automated target was the LEFT SHOULDER, not the elbow. Under the
provisional grasp-center offset [0, 0, 0.35] m along flange +Z:

- Manual grasp center was about 0.335 m from the old shoulder observation.
- Manual approach axis differed by about 42 degrees from the old planned axis.
- Manual flange position differed by about 0.788 m from the failed flange target.
- Current manual pose: controller IK and local FK succeeded.
- Old shoulder / fitted surface-normal approach: IK -32 for hypothetical gaps
  0.40, 0.30, 0.20, 0.10, and 0.00 m.
- Same old shoulder / manual orientation: endpoint IK and local FK succeeded
  for those gaps. No paths were validated or executed for these alternatives.
- Changing roll around the old normal in 45-degree increments did not solve
  the old 0.40 m standoff endpoint (all returned -32).

The 0.40 m standoff included 0.20 m requested clearance plus two 0.10 m reserves.
Combined with the 0.35 m tool offset, the failed flange target was 0.75 m outward
from the shoulder along the fitted normal. This combination, rather than the
joint point alone, caused the requested pose to be rejected by controller IK.

The normal was fitted to a local clothed surface. Temporal stability and a small
plane residual do not establish that it is the intended anatomical grasp direction.
Do not assume the new manual pose calibrates the TCP or validates hand-eye accuracy.
All results use the existing hand-eye calibration and a user-estimated tool length.
The old target was historical; mannequin/base immobility during this comparison
was assumed, not independently measured.

Next planning should explicitly select the desired joint, confirm the intended
approach direction using the user's manual pose, and validate the entire path.
Do not replay either saved pose automatically.
