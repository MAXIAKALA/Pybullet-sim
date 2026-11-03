import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from wireconnect.utils import (  # noqa: E402
    camera_intrinsics, look_at, mat_from_quat, quat_from_mat,
)


class TestGeometry(unittest.TestCase):
    def test_quat_roundtrip(self):
        for _ in range(20):
            q = np.random.default_rng(0).normal(size=4)
            q /= np.linalg.norm(q)
            R = mat_from_quat(q)
            q2 = quat_from_mat(R)
            R2 = mat_from_quat(q2)
            np.testing.assert_allclose(R, R2, atol=1e-8)

    def test_look_at_orthonormal(self):
        R, t = look_at((0.5, -0.5, 0.9), (0.6, 0.0, 0.55), (0, 0, 1))
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-8)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=8)
        np.testing.assert_allclose(t, [0.5, -0.5, 0.9])

    def test_intrinsics(self):
        K = camera_intrinsics(640, 480, 60)
        self.assertAlmostEqual(K[0, 0], K[1, 1])
        self.assertAlmostEqual(K[0, 2], 320)
        self.assertAlmostEqual(K[1, 2], 240)


if __name__ == "__main__":
    unittest.main()
