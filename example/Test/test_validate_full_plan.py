"""Synthetic geometry tests only; no robot or camera access."""
import copy
import unittest

import numpy as np

from validate_full_plan import validate_manifest


def matrix(x, y, z):
    value = np.eye(4)
    value[:3, 3] = [x, y, z]
    return value.ravel().tolist()


def fixture():
    q0, q1, q2 = [0.] * 6, [.1] * 6, [.2] * 6
    first = dict(name="joint_transit", command="MoveAbsJ", start_joints_rad=q0,
                 end_joints_rad=q1, start_flange_base=matrix(.1, .2, .2),
                 end_flange_base=matrix(.6, .2, .2), end_gap_m=.4,
                 stop_at_endpoint=1, sample_count=101)
    second = dict(name="linear_approach", command="MoveL", start_joints_rad=q1.copy(),
                  end_joints_rad=q2, start_flange_base=matrix(.6, .2, .2),
                  end_flange_base=matrix(.6, .2, .6), start_gap_m=.4, end_gap_m=0.,
                  fixed_orientation=1, stop_at_endpoint=1, sample_count=101)
    return dict(schema_version=1, status="kinematic_plan_complete", robot_motion_permitted=0,
                collision_validation="not_performed", observation_rounds=1,
                reidentify_between_stages=0, robot_arm="left", gripper_command="none",
                stages=[first, second], tool_offset_flange_m=[0, 0, .3],
                target_point_base_m=[.6, .2, .9], flange_T_camera=matrix(0, 0, 0),
                world_T_base=matrix(0, 0, 0), local_ik_sample_count=0,
                controller_linear_ik_all_passed=1)


class FullPlanTest(unittest.TestCase):
    def seeded_fixture(self):
        plan = fixture()
        plan.update(schema_version=2, local_ik_sample_count=100, controller_linear_ik_all_passed=0,
                    controller_linear_ik_queries=0, controller_linear_ik_status="not_queried",
                    linear_ik_solver="seeded_local_sdk",
                    linear_ik_seed_policy="stage1_endpoint_then_previous_sample",
                    boundary_ik_round_trip_passed=1)
        return plan

    def test_continuous_seeded_plan(self):
        validate_manifest(self.seeded_fixture())

    def insertion_fixture(self):
        plan = self.seeded_fixture()
        plan.update(schema_version=3, target_point_semantics="observed_surface", surface_insertion_m=.08,
                    insertion_direction="approach_positive_flange_z", grasp_target_base_m=[.6, .2, .98])
        second = plan["stages"][1]
        second["end_flange_base"][11] += .08
        second.update(end_gap_m=-.08, approach_distance_m=.48)
        return plan

    def test_surface_insertion_changes_final_center_not_pregrasp(self):
        plan = self.insertion_fixture()
        validate_manifest(plan)
        self.assertEqual(plan["stages"][0], self.seeded_fixture()["stages"][0])

    def test_insertion_mismatch_rejected(self):
        for field, value in [("surface_insertion_m", -.08), ("surface_insertion_m", .081),
                             ("grasp_target_base_m", [.6, .2, .9]),
                             ("insertion_direction", "base_z")]:
            with self.subTest(field=field):
                plan = self.insertion_fixture()
                plan[field] = value
                with self.assertRaises(ValueError):
                    validate_manifest(plan)

    def test_stale_40cm_second_stage_rejected(self):
        plan = self.insertion_fixture()
        plan["stages"][1]["approach_distance_m"] = .4
        with self.assertRaisesRegex(ValueError, "approach distance"):
            validate_manifest(plan)

    def test_historical_replay_is_labeled_and_not_new_observation(self):
        plan = self.seeded_fixture()
        plan.update(diagnostic_replay=1, status="diagnostic_kinematic_complete", observation_rounds=0)
        validate_manifest(plan)
        plan["status"] = "kinematic_plan_complete"
        with self.assertRaisesRegex(ValueError, "mislabeled"):
            validate_manifest(plan)

    def test_seeded_plan_rejects_false_controller_verification(self):
        plan = self.seeded_fixture()
        plan["controller_linear_ik_all_passed"] = 1
        with self.assertRaisesRegex(ValueError, "controller verification"):
            validate_manifest(plan)

    def test_seeded_plan_rejects_live_pose_reference(self):
        plan = self.seeded_fixture()
        plan["linear_ik_seed_policy"] = "live_robot_pose"
        with self.assertRaisesRegex(ValueError, "seed contract"):
            validate_manifest(plan)

    def test_seeded_plan_requires_every_step_and_boundary_check(self):
        for key, value in [("local_ik_sample_count", 1), ("boundary_ik_round_trip_passed", 0)]:
            with self.subTest(key=key):
                plan = self.seeded_fixture()
                plan[key] = value
                with self.assertRaises(ValueError):
                    validate_manifest(plan)

    def test_valid_single_observation_two_stage_plan(self):
        self.assertEqual(len(validate_manifest(fixture())), 2)

    def test_model_fallback_is_explicit_not_controller_success(self):
        plan = fixture()
        plan.update(local_ik_sample_count=1, controller_linear_ik_all_passed=0)
        validate_manifest(plan)
        plan["controller_linear_ik_all_passed"] = 1
        with self.assertRaisesRegex(ValueError, "IK status"):
            validate_manifest(plan)

    def test_discontinuous_joint_handover_rejected(self):
        plan = fixture()
        plan["stages"][1]["start_joints_rad"][0] += .01
        with self.assertRaisesRegex(ValueError, "joint discontinuity"):
            validate_manifest(plan)

    def test_discontinuous_pose_handover_rejected(self):
        plan = fixture()
        plan["stages"][1]["start_flange_base"][3] += .01
        with self.assertRaisesRegex(ValueError, "pose discontinuity"):
            validate_manifest(plan)

    def test_wrong_target_rejected(self):
        plan = fixture()
        plan["target_point_base_m"][0] += .01
        with self.assertRaises(ValueError):
            validate_manifest(plan)

    def test_wrong_tool_length_rejected(self):
        plan = fixture()
        plan["tool_offset_flange_m"][2] = .35
        with self.assertRaisesRegex(ValueError, "tool offset"):
            validate_manifest(plan)

    def test_changed_approach_orientation_rejected(self):
        plan = fixture()
        final = np.array(plan["stages"][1]["end_flange_base"]).reshape(4, 4)
        final[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        plan["stages"][1]["end_flange_base"] = final.ravel().tolist()
        with self.assertRaisesRegex(ValueError, "changes orientation"):
            validate_manifest(plan)

    def test_no_reidentification_or_unverified_execution_permission(self):
        for key, value in [("observation_rounds", 2), ("reidentify_between_stages", 1),
                           ("robot_motion_permitted", 1), ("gripper_command", "close")]:
            with self.subTest(key=key):
                plan = copy.deepcopy(fixture())
                plan[key] = value
                with self.assertRaises(ValueError):
                    validate_manifest(plan)

    def test_nonfinite_geometry_rejected(self):
        plan = fixture()
        plan["target_point_base_m"][0] = float("nan")
        with self.assertRaises(ValueError):
            validate_manifest(plan)


if __name__ == "__main__":
    unittest.main()
