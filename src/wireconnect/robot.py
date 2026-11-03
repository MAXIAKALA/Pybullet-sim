"""Thin Franka Panda wrapper: IK, joint moves, gripper."""
from __future__ import annotations

import numpy as np
import pybullet as p

from .utils import quat_from_mat


class PandaRobot:
    ARM_JOINTS = list(range(7))
    FINGER_JOINTS = [9, 10]
    TCP_LINK = 11
    HAND_LINK = 8

    def __init__(self, body: int, on_step=None):
        self.body = body
        self.on_step = on_step
        self.home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    def _step(self):
        p.stepSimulation()
        if self.on_step is not None:
            self.on_step()

    # ------------------------------------------------------------------ basic
    def reset(self):
        for i, q in enumerate(self.home):
            p.resetJointState(self.body, i, q)
        for j in self.FINGER_JOINTS:
            p.resetJointState(self.body, j, 0.04)

    def tcp_pose(self):
        ls = p.getLinkState(self.body, self.TCP_LINK, computeForwardKinematics=True)
        return np.array(ls[4]), np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)

    def ik(self, pos, R):
        q = p.calculateInverseKinematics(
            self.body, self.TCP_LINK, list(map(float, pos)), quat_from_mat(R),
            maxNumIterations=200, residualThreshold=1e-5,
        )
        return np.array(q[:7], dtype=float)

    def set_arm(self, q, force=500.0, max_velocity=1.0):
        for i, qi in enumerate(q):
            p.setJointMotorControl2(
                self.body, i, p.POSITION_CONTROL, targetPosition=float(qi),
                force=force, maxVelocity=max_velocity,
            )

    def move_joints(self, q, steps=240, tol=0.01):
        self.set_arm(q)
        for _ in range(steps):
            self._step()
            cur = np.array([p.getJointState(self.body, i)[0] for i in self.ARM_JOINTS])
            if np.max(np.abs(cur - q)) < tol:
                break

    def move_to(self, pos, R, max_steps=600, tol=0.004, max_velocity=1.0):
        pos = np.asarray(pos, dtype=float)
        q = self.ik(pos, R)
        self.set_arm(q, max_velocity=max_velocity)
        best = 1e9
        for step in range(max_steps):
            self._step()
            if step % 60 == 59:
                tcp, _ = self.tcp_pose()
                err = float(np.linalg.norm(tcp - pos))
                best = min(best, err)
                if err > tol * 3:
                    q = self.ik(pos, R)
                    self.set_arm(q, max_velocity=max_velocity)
        tcp, _ = self.tcp_pose()
        return float(np.linalg.norm(tcp - pos))

    # ------------------------------------------------------------------ gripper
    def gripper(self, width: float, force: float = 120.0, steps: int = 90):
        for j in self.FINGER_JOINTS:
            p.setJointMotorControl2(
                self.body, j, p.POSITION_CONTROL, targetPosition=float(width),
                force=force, maxVelocity=0.3,
            )
        for _ in range(steps):
            self._step()
