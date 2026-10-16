import unittest

import numpy as np
import pyrealsense2 as rs

from mannequin_pregrasp_geometry import surface_normal


class SurfaceNormalTest(unittest.TestCase):
    def setUp(self):
        self.intr = rs.intrinsics()
        self.intr.width = self.intr.height = 200
        self.intr.fx = self.intr.fy = 900
        self.intr.ppx = self.intr.ppy = 100
        self.intr.model = rs.distortion.none
        self.intr.coeffs = [0.0] * 5

    def test_tilted_plane_points_toward_camera(self):
        y, x = np.indices((200, 200))
        z = 1.4 / (1 + 0.2 * (x - 100) / 900 - 0.1 * (y - 100) / 900)
        depth = np.rint(z * 1000).astype(np.uint16)
        normal, rms = surface_normal(depth, self.intr, 0.001, 100, 100, [0, 0, 1.4])
        expected = -np.array([0.2, -0.1, 1.0])
        expected /= np.linalg.norm(expected)
        self.assertGreater(float(normal @ expected), 0.999)
        self.assertLess(rms, 0.001)

    def test_missing_depth_rejected(self):
        with self.assertRaises(ValueError):
            surface_normal(np.zeros((200, 200), np.uint16), self.intr,
                           0.001, 100, 100, [0, 0, 1.4])

    def test_joint_off_surface_rejected(self):
        with self.assertRaises(ValueError):
            surface_normal(np.full((200, 200), 1430, np.uint16), self.intr,
                           0.001, 100, 100, [0, 0, 1.4])


if __name__ == "__main__":
    unittest.main()
