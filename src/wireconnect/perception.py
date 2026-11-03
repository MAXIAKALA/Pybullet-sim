"""Vision-based fiducial detection.

The simulation uses four high-contrast coloured spheres per marker instead of a
printed ArUco card (PyBullet's TinyRenderer does not render textures).  The
geometry is the same PnP problem, so the real-robot detector can be swapped for
``cv2.aruco`` without touching the task layer.
"""
from __future__ import annotations

import numpy as np
import cv2


HUE_RANGES = {
    "red": [(0, 8), (172, 179)],
    "orange": [(9, 22)],
    "yellow": [(23, 38)],
    "green": [(40, 85)],
    "cyan": [(86, 100)],
    "blue": [(101, 130)],
    "purple": [(131, 150)],
    "magenta": [(151, 171)],
}

# marker frame: x right, y up, z out of the plate
LAYOUT = [(-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]  # TL, TR, BR, BL


class ColorFiducialDetector:
    def __init__(self, K: np.ndarray, half_size: float = 0.025, colors=("red", "green", "blue", "yellow")):
        self.K = np.asarray(K, dtype=np.float64)
        self.half = float(half_size)
        self.colors = tuple(colors)

    def detect(self, rgb: np.ndarray):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        h = hsv[:, :, 0].astype(np.int16)
        s = hsv[:, :, 1].astype(np.int16)
        v = hsv[:, :, 2].astype(np.int16)
        centroids = {}
        for color in self.colors:
            mask = np.zeros(h.shape, dtype=bool)
            for lo, hi in HUE_RANGES[color]:
                mask |= (h >= lo) & (h <= hi)
            mask &= (s > 90) & (v > 55)
            ys, xs = np.nonzero(mask)
            if len(xs) < 25:
                return None
            centroids[color] = np.array([xs.mean(), ys.mean()], dtype=np.float64)

        img_pts = np.array([centroids[c] for c in self.colors], dtype=np.float64)
        obj_pts = np.array(
            [[LAYOUT[i][0] * self.half, LAYOUT[i][1] * self.half, 0.0] for i in range(4)],
            dtype=np.float64,
        )
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R_cm, _ = cv2.Rodrigues(rvec)
        return {
            "R_cm": R_cm,
            "t_cm": tvec.ravel(),
            "centroids": centroids,
            "img_pts": img_pts,
        }

    @staticmethod
    def to_world(R_wc: np.ndarray, t_wc: np.ndarray, det: dict):
        R_wm = R_wc @ det["R_cm"]
        t_wm = t_wc + R_wc @ det["t_cm"]
        return R_wm, t_wm
