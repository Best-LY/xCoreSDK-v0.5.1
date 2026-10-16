# Offline Support-Plane Review

This run used one fresh left-camera observation on 2026-10-09. The complete
two-stage kinematic plan and supervisor replay passed. No motion was issued.

The operator reported 300 mm flange-to-grasp-center length, 150 mm open outer
width, 20 mm maximum thickness and 100 mm extension beyond the grasp center.
These dimensions are not an exact finger/camera/adapter collision mesh.

An initial unfiltered plane fit in ROI [720,220,900,420] estimated the target
67 mm above a nearby surface and the finger-tip axis 32 mm below its extrapolated
plane. That fit had 13 mm RMS error. The subsequent robust fit FAILED the
dominant-plane requirement; do not cite the initial result as proven collision.

Three other manually selected visible floor patches passed the local robust
fit (about 4 mm inlier RMS, 7.3 mm inlier P95), but extrapolation to the elbow
was inconsistent:

| ROI | Estimated grasp height | Estimated tip-height interval |
| --- | --- | --- |
| [700,420,830,590] | 111 mm | +9.5 to +12.5 mm |
| [660,350,780,480] | 96 mm | -11.1 to +4.2 mm |
| [720,500,860,640] | 66 mm | -41.9 to -23.8 mm |

These are extrapolations of selected visible depth patches, not direct
measurements of the occluded surface underneath the elbow. Positive numbers
alone do not establish clearance. Small inlier residuals do not bound plane
extrapolation, hand-eye calibration, recognition or tool measurement errors.
The exact finger cavity and whole-arm collision geometry remain unmodeled.

Conclusion: final finger/support clearance is unresolved. Do not select only
the favorable patch, relax gates or automatically move to the zero-gap target.
Onsite elbow-to-support clearance or a side-view measurement was requested.

The operator then reported approximately 5 cm of target-to-support clearance.
Given the reported 10 cm finger extension and current nearly downward approach,
the planned grasp-center endpoint is expected to put the tips roughly 5 cm
past the available support clearance. This supports holding the current zero-gap
approach independently of selecting a favorable depth patch. See
`onsite_clearance.json`. Changing only velocity cannot resolve the geometry.
The scene must provide open space under the elbow, or the target/approach/tool
geometry must be deliberately revised and replanned. Do not silently substitute
a stopped-short endpoint and call it successful grasp alignment.

Reproduction: `inspect_gripper_support_plane.py PLAN --roi X0 Y0 X1 Y1`.
Reports roi2 through roi4 are preserved next to this file. The failed roi1
produced no JSON and its empty output file was removed. No tool parameters,
controller settings, speeds or execution holds were changed for this diagnostic.
