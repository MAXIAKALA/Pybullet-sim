"""Small geometry / camera helpers shared by simulation, perception and task code."""
from __future__ import annotations

import numpy as np
import pybullet as p


def normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalize zero vector")
    return v / n


def mat_from_quat(q) -> np.ndarray:
    return np.array(p.getMatrixFromQuaternion(q), dtype=float).reshape(3, 3)


def quat_from_mat(R: np.ndarray):
    """Robust 3x3 rotation matrix -> (x,y,z,w) quaternion."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def look_at(eye, target, up=(0, 0, 1)):
    """OpenCV-convention camera basis: x right, y down, z forward.

    Returns (R_wc, t_wc) with columns [right, down, forward].
    """
    eye = np.asarray(eye, dtype=float)
    target = np.asarray(target, dtype=float)
    up = np.asarray(up, dtype=float)
    f = normalize(target - eye)
    r = normalize(np.cross(f, up))
    d = np.cross(f, r)  # down in the image
    R = np.column_stack([r, d, f])
    return R, eye


def invert(R: np.ndarray, t: np.ndarray):
    return R.T, -R.T @ np.asarray(t, dtype=float)


def camera_intrinsics(width: int, height: int, vfov_deg: float = 60.0) -> np.ndarray:
    f = height / (2.0 * np.tan(np.deg2rad(vfov_deg) / 2.0))
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]], dtype=float)
