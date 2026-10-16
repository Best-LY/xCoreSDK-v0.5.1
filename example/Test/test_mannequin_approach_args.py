"""Argument regressions only; never capture images or connect to a robot."""
import contextlib
import io
import unittest

from left_shoulder_micro_approach import parse_args


class ApproachArgsTest(unittest.TestCase):
    def test_reach_plan_uses_fresh_elbow_demonstrated_orientation(self):
        args = parse_args(["--reach-plan", "--joint", "left_elbow"])
        self.assertTrue(args.reach_plan and args.demonstrated_pregrasp)
        self.assertFalse(args.execute or args.staged_plan)

    def test_reach_plan_rejects_execution_and_old_modes(self):
        for extra in (["--execute"], ["--staged-plan"], ["--grasp-plan"], ["--pregrasp"],
                      ["--motion", "joint"], ["--tool-length-mm", "300"], ["--grasp-align"]):
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(["--reach-plan", "--joint", "left_elbow"]+extra)

    def test_demonstrated_elbow_defaults_to_read_only_joint_motion(self):
        args = parse_args(["--demonstrated-pregrasp", "--joint", "left_elbow"])
        self.assertEqual(args.motion, "joint")
        self.assertFalse(args.execute)

    def test_linear_override_is_preserved(self):
        args = parse_args(["--demonstrated-pregrasp", "--motion", "linear"])
        self.assertEqual(args.motion, "linear")
        self.assertFalse(args.execute)

    def test_explicit_execution(self):
        args = parse_args(["--demonstrated-pregrasp", "--motion", "joint", "--execute"])
        self.assertEqual(args.motion, "joint")
        self.assertTrue(args.execute)

    def test_surface_normal_mode_keeps_linear_default(self):
        self.assertEqual(parse_args(["--pregrasp"]).motion, "linear")

    def test_surface_normal_accepts_explicit_joint(self):
        self.assertEqual(parse_args(["--pregrasp", "--motion", "joint"]).motion, "joint")

    def test_grasp_plan_is_read_only_joint_motion(self):
        args = parse_args(["--grasp-plan", "--joint", "left_elbow"])
        self.assertTrue(args.grasp_plan)
        self.assertTrue(args.demonstrated_pregrasp)
        self.assertEqual(args.motion, "joint")
        self.assertFalse(args.execute)

    def test_staged_plan_is_read_only_and_selects_demonstrated_orientation(self):
        args = parse_args(["--staged-plan", "--joint", "left_elbow"])
        self.assertTrue(args.staged_plan)
        self.assertTrue(args.demonstrated_pregrasp)
        self.assertFalse(args.execute)

    def test_staged_plan_rejects_execution_and_conflicting_modes(self):
        cases = [
            ["--execute"], ["--grasp-align"], ["--grasp-plan"],
            ["--pregrasp"], ["--demonstrated-pregrasp"], ["--plan-full"],
            ["--approach-full"], ["--motion", "joint"], ["--motion", "linear"],
            ["--tool-length-mm", "300"],
        ]
        for extra in cases:
            with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    parse_args(["--staged-plan"] + extra)
                self.assertEqual(error.exception.code, 2)

    def test_grasp_plan_allows_explicit_linear_planning(self):
        args = parse_args(["--grasp-plan", "--motion", "linear"])
        self.assertEqual(args.motion, "linear")
        self.assertFalse(args.execute)

    def test_grasp_align_requires_explicit_measured_length_and_defaults_read_only(self):
        args = parse_args(["--grasp-align", "--tool-length-mm", "300", "--joint", "left_elbow"])
        self.assertTrue(args.grasp_align)
        self.assertEqual(args.tool_length_mm, 300.0)
        self.assertEqual(args.motion, "joint")
        self.assertFalse(args.execute)

    def test_grasp_align_execution_is_blocked_after_contact(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                parse_args(["--grasp-align", "--tool-length-mm", "300", "--execute"])
        self.assertEqual(error.exception.code, 2)

    def test_legacy_mode_is_unchanged_and_read_only(self):
        args = parse_args([])
        self.assertEqual(args.joint, "left_shoulder")
        self.assertFalse(args.execute)

    def test_invalid_combinations_fail_before_capture(self):
        cases = [
            ["--motion", "joint"],
            ["--joint", "left_elbow"],
            ["--plan-full", "--execute"],
            ["--demonstrated-pregrasp", "--pregrasp"],
            ["--demonstrated-pregrasp", "--plan-full"],
            ["--approach-full"],
            ["--demonstrated-pregrasp", "--motion", "invalid"],
            ["--grasp-plan", "--execute"],
            ["--grasp-plan", "--pregrasp"],
            ["--grasp-plan", "--demonstrated-pregrasp"],
            ["--grasp-plan", "--plan-full"],
            ["--grasp-plan", "--approach-full"],
            ["--grasp-align", "--execute"],
            ["--grasp-align", "--tool-length-mm", "350"],
            ["--grasp-align", "--tool-length-mm", "nan"],
            ["--grasp-align", "--tool-length-mm", "300", "--grasp-plan"],
            ["--grasp-align", "--tool-length-mm", "300", "--pregrasp"],
            ["--grasp-align", "--tool-length-mm", "300", "--demonstrated-pregrasp"],
            ["--tool-length-mm", "300"],
        ]
        for argv in cases:
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    parse_args(argv)
                self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
