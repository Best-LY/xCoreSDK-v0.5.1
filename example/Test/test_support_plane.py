import unittest
import numpy as np

from inspect_gripper_support_plane import fit_plane, tip_clearance


class SupportPlaneTest(unittest.TestCase):
    def test_plane_fit_with_depth_outliers(self):
        rng = np.random.default_rng(8)
        points = np.column_stack((rng.uniform(-.2, .2, 1500), rng.uniform(-.2, .2, 1500),
                                  rng.normal(0, .001, 1500)))
        points[:200, 2] += .1
        n, center, result = fit_plane(points, [0, 0, 1])
        self.assertGreater(n[2], .999)
        self.assertLess(abs(center[2]), .001)
        self.assertGreater(result["inlier_fraction"], .85)

    def test_line_and_missing_depth_are_rejected(self):
        for data in (np.zeros((30, 3)), np.column_stack((np.linspace(0, 1, 1000), np.zeros((1000, 2))))):
            with self.assertRaises(ValueError):
                fit_plane(data, [0, 0, 1])

    def test_tips_extend_beyond_center_toward_support(self):
        final = np.eye(4)
        final[:3, :3] = np.diag([1, -1, -1])
        g = dict(maximum_open_outer_width_m=.15, maximum_thickness_m=.02,
                 finger_tip_extension_beyond_grasp_center_m=.1)
        result = tip_clearance(final, [0, 0, .067], g, [0, 0, 1], [0, 0, 0])
        self.assertAlmostEqual(result["finger_tip_axis_signed_height_m"], -.033)
        self.assertAlmostEqual(result["finger_tip_height_range_m"][1], -.033)

    def test_transverse_bound_includes_unknown_mounting_angle(self):
        final = np.eye(4)
        angle = .2
        final[:3, :3] = [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]]
        g = dict(maximum_open_outer_width_m=.15, maximum_thickness_m=.02,
                 finger_tip_extension_beyond_grasp_center_m=.1)
        result = tip_clearance(final, [0, 0, .2], g, [0, 0, 1], [0, 0, 0])
        lo, hi = result["finger_tip_height_range_m"]
        for theta in np.linspace(0, 2*np.pi, 100):
            transverse = result["transverse_bound_radius_m"]*np.array([np.cos(theta), np.sin(theta), 0])
            tip = np.array([0, 0, .2]) + final[:3, :3] @ (transverse+[0, 0, .1])
            self.assertGreaterEqual(tip[2], lo-1e-9)
            self.assertLessEqual(tip[2], hi+1e-9)


if __name__ == "__main__":
    unittest.main()
