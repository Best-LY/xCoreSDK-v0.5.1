import copy
import unittest

import numpy as np

from validate_reach_plan import validate_manifest


def fixture():
    start, end = np.eye(4), np.eye(4)
    start[:3, 3], end[:3, 3] = [.2, 0, .2], [.5, 0, .45]
    stage = dict(command="MoveAbsJ", start_joints_rad=[0]*6, end_joints_rad=[.2]*6,
                 start_flange_base=start.ravel().tolist(), end_flange_base=end.ravel().tolist(),
                 sample_count=41, stop_at_endpoint=1, joint_speed_fraction=.01)
    return dict(schema_version=1, plan_kind="reach_surface_offset", status="kinematic_plan_complete",
                robot_motion_permitted=0, collision_validation="not_performed",
                controller_trajectory_acceptance="not_tested", robot_arm="left", joint="left_elbow",
                gripper_command="none", observation_rounds=1, fixed_pregrasp_required=0,
                insertion_direction="approach_positive_flange_z", surface_insertion_m=.05,
                surface_point_base_m=[.5, 0, .7], grasp_target_base_m=[.5, 0, .75],
                tool_offset_flange_m=[0, 0, .3], stages=[stage], selected_candidate="direct_joint",
                sampled_grasp_path_length_m=.5, sampled_joint_travel_rad=1.2,
                candidates=[dict(name="direct_joint", passed=1, grasp_path_length_m=.5, joint_travel_rad=1.2),
                            dict(name="short_but_failed", passed=0)])


class ReachValidationTest(unittest.TestCase):
    def test_direct_route_has_no_mandatory_waypoint(self):
        self.assertEqual(len(validate_manifest(fixture())), 1)

    def test_bad_goal_or_execution_claim_rejected(self):
        for k, v in [("surface_insertion_m", .08), ("robot_motion_permitted", 1),
                     ("grasp_target_base_m", [.5, 0, .7]), ("fixed_pregrasp_required", 1),
                     ("controller_trajectory_acceptance", "passed")]:
            with self.subTest(key=k):
                p = fixture(); p[k] = v
                with self.assertRaises(ValueError):
                    validate_manifest(p)

    def test_inferior_or_unreachable_selection_rejected(self):
        p = fixture()
        p["selected_candidate"] = "short_but_failed"
        with self.assertRaises(ValueError):
            validate_manifest(p)
        p = fixture()
        p["candidates"].append(dict(name="shorter", passed=1, grasp_path_length_m=.4, joint_travel_rad=2.))
        with self.assertRaisesRegex(ValueError, "shortest"):
            validate_manifest(p)

    def test_discontinuous_multi_leg_route_rejected(self):
        p = fixture(); leg = copy.deepcopy(p["stages"][0]); p["stages"].append(leg)
        with self.assertRaisesRegex(ValueError, "handover"):
            validate_manifest(p)

    def test_negative_or_nonfinite_speed_rejected(self):
        for speed in (-1, float("nan"), .5):
            p = fixture(); p["stages"][0]["joint_speed_fraction"] = speed
            with self.assertRaisesRegex(ValueError, "speed"):
                validate_manifest(p)


if __name__ == "__main__":
    unittest.main()
