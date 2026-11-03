"""PyBullet scene: Franka Panda, table, horizontal connector panel, plug and fiducials."""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np
import pybullet as p
import pybullet_data

from .utils import look_at


@dataclass
class SceneConfig:
    # table / panel
    table_top: float = 0.40
    panel_x: float = 0.70
    socket_z: float = 0.55
    socket_size: float = 0.034          # square hole side
    panel_thickness: float = 0.030
    panel_half_y: float = 0.10
    panel_half_z: float = 0.10
    # plug
    plug_radius: float = 0.009
    plug_length: float = 0.10
    plug_start: tuple = (0.45, -0.15, 0.50)
    # fiducial
    marker_half: float = 0.030
    marker_sphere_r: float = 0.007
    # camera
    cam_pos: tuple = (0.55, -0.55, 0.95)
    cam_target: tuple = (0.60, 0.0, 0.55)
    cam_up: tuple = (0.0, 0.0, 1.0)
    cam_width: int = 640
    cam_height: int = 480
    cam_vfov: float = 60.0
    # physics
    time_step: float = 1.0 / 240.0


class WireConnectScene:
    def __init__(self, cfg: SceneConfig | None = None, gui: bool = False, seed: int = 0):
        self.cfg = cfg or SceneConfig()
        self.rng = np.random.default_rng(seed)
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.cfg.time_step)
        p.setPhysicsEngineParameter(numSolverIterations=80, numSubSteps=2)
        self._build()

    # ------------------------------------------------------------------ build
    def _build(self):
        cfg = self.cfg
        self.plane = p.loadURDF("plane.urdf")

        # table
        table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.40, 0.45, cfg.table_top / 2])
        table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.40, 0.45, cfg.table_top / 2],
                                        rgbaColor=[0.82, 0.82, 0.80, 1.0])
        self.table = p.createMultiBody(0, table_vis, table_col, basePosition=[0.55, 0, cfg.table_top / 2])

        # robot
        self.panda = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
        self.home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        for i, q in enumerate(self.home):
            p.resetJointState(self.panda, i, q)
        for _ in range(20):
            p.stepSimulation()

        # panel (frame of 4 walls + back plate around the socket)
        half = cfg.panel_half_y
        thick = cfg.panel_thickness
        inner = cfg.socket_size / 2
        self.panel_bodies: list[int] = []
        wall_z = (half - inner) / 2
        # top / bottom
        for sgn in (+1, -1):
            z = cfg.socket_z + sgn * (inner + wall_z)
            self._add_static_box([cfg.panel_x, 0, z], [thick / 2, half, wall_z], [0.30, 0.31, 0.33, 1])
        # left / right (y)
        for sgn in (+1, -1):
            y = sgn * (inner + wall_z)
            self._add_static_box([cfg.panel_x, y, cfg.socket_z], [thick / 2, wall_z, inner], [0.30, 0.31, 0.33, 1])
        # back plate
        self._add_static_box([cfg.panel_x + thick / 2 + 0.003, 0, cfg.socket_z],
                             [0.003, inner, inner], [0.15, 0.15, 0.16, 1])
        # support column
        self._add_static_box([cfg.panel_x, 0, (cfg.table_top + cfg.socket_z - inner) / 2],
                             [0.035, 0.05, (cfg.socket_z - inner - cfg.table_top) / 2], [0.35, 0.36, 0.38, 1])

        # plug
        cyl_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=cfg.plug_radius, height=cfg.plug_length)
        cyl_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=cfg.plug_radius, length=cfg.plug_length,
                                      rgbaColor=[0.78, 0.78, 0.82, 1.0])
        self.plug = p.createMultiBody(
            baseMass=0.05, baseCollisionShapeIndex=cyl_col, baseVisualShapeIndex=cyl_vis,
            basePosition=list(cfg.plug_start),
            baseOrientation=p.getQuaternionFromEuler([0, np.pi / 2, 0]),
        )
        p.changeDynamics(self.plug, -1, lateralFriction=1.2, spinningFriction=0.05,
                         restitution=0.0, linearDamping=0.05, angularDamping=0.1)

        # decorative stand under the plug
        self._add_static_box([cfg.plug_start[0], cfg.plug_start[1], 0.445],
                             [0.025, 0.025, 0.045], [0.45, 0.46, 0.48, 1], collision=False)

        # fiducials
        self.plug_marker = self._create_marker(
            [cfg.plug_start[0], cfg.plug_start[1], cfg.plug_start[2] + cfg.plug_radius + 0.002],
            [0, 0, 0, 1], ["red", "green", "blue", "yellow"], name="plug_marker")
        self.panel_marker = self._create_marker(
            [cfg.panel_x - 0.08, 0, cfg.socket_z + 0.20], [0, 0, 0, 1],
            ["magenta", "cyan", "orange", "purple"], name="panel_marker")
        # The plug marker is a visual-only body that we teleport every step;
        # a fixed constraint to a massless body would anchor the plug to the world.
        self.plug_marker_offset = np.array([0.0, 0.0, cfg.plug_radius + 0.002])
        # vertical post under the panel marker
        self._add_static_box([cfg.panel_x - 0.065, 0, cfg.socket_z + 0.10],
                             [0.008, 0.008, 0.10], [0.4, 0.4, 0.4, 1], collision=False)

        # keep the plug upright on its stand until grasped
        self.plug_world_cid = self._fix_plug_to_world()

        # camera
        self.cam_R, self.cam_t = look_at(cfg.cam_pos, cfg.cam_target, cfg.cam_up)
        self.view = p.computeViewMatrix(cfg.cam_pos, cfg.cam_target, cfg.cam_up)
        self.proj = p.computeProjectionMatrixFOV(cfg.cam_vfov, cfg.cam_width / cfg.cam_height, 0.01, 3.0)

        # side camera for the demo inset (shows the plug entering the socket)
        side_eye = (0.60, -0.10, 0.60)
        side_target = (0.70, 0.0, 0.55)
        self.side_R, self.side_t = look_at(side_eye, side_target, (0, 0, 1))
        self.side_view = p.computeViewMatrix(side_eye, side_target, (0, 0, 1))
        self.side_proj = p.computeProjectionMatrixFOV(55.0, 4 / 3, 0.01, 3.0)

    def _add_static_box(self, pos, half, rgba, collision=True):
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half) if collision else -1
        body = p.createMultiBody(0, vis, col, basePosition=pos)
        if collision:
            self.panel_bodies.append(body)
        return body

    def _create_marker(self, pos, orn, colors, name="marker"):
        cfg = self.cfg
        h = cfg.marker_half
        r = cfg.marker_sphere_r
        plate_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[h, h, 0.002],
                                        rgbaColor=[0.96, 0.96, 0.94, 1.0])
        rgba = {
            "red": [1.0, 0.05, 0.05, 1.0],
            "green": [0.05, 0.85, 0.20, 1.0],
            "blue": [0.05, 0.35, 1.0, 1.0],
            "yellow": [1.0, 0.85, 0.05, 1.0],
            "magenta": [1.0, 0.05, 0.85, 1.0],
            "cyan": [0.05, 0.85, 0.90, 1.0],
            "orange": [1.0, 0.45, 0.05, 1.0],
            "purple": [0.45, 0.05, 0.85, 1.0],
        }
        layout = [(-1, 1), (1, 1), (1, -1), (-1, -1)]  # TL, TR, BR, BL
        link_vis, link_pos = [], []
        for k, (sx, sy) in zip(colors, layout):
            link_vis.append(p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=rgba[k]))
            link_pos.append([sx * h, sy * h, 0.0])
        body = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=plate_vis, basePosition=pos, baseOrientation=orn,
            linkMasses=[0, 0, 0, 0],
            linkCollisionShapeIndices=[-1, -1, -1, -1],
            linkVisualShapeIndices=link_vis,
            linkPositions=link_pos,
            linkOrientations=[[0, 0, 0, 1]] * 4,
            linkInertialFramePositions=[[0, 0, 0]] * 4,
            linkInertialFrameOrientations=[[0, 0, 0, 1]] * 4,
            linkParentIndices=[0, 0, 0, 0],
            linkJointTypes=[p.JOINT_FIXED] * 4,
            linkJointAxis=[[0, 0, 0]] * 4,
        )
        return body

    def _fix_plug_to_world(self):
        pos, orn = p.getBasePositionAndOrientation(self.plug)
        return p.createConstraint(
            self.plug, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0],
            [0, 0, 0], list(pos))

    # ------------------------------------------------------------------ reset
    def reset(self, randomize: bool = False):
        cfg = self.cfg
        self.unlatch_plug()
        self.detach_grasp()
        for i, q in enumerate(self.home):
            p.resetJointState(self.panda, i, q)
        for j in (9, 10):
            p.resetJointState(self.panda, j, 0.04)
        # re-fix plug to world before moving it
        if getattr(self, "plug_world_cid", None) is not None:
            p.removeConstraint(self.plug_world_cid)
        start = list(cfg.plug_start)
        if randomize:
            start[0] += float(self.rng.uniform(-0.02, 0.02))
            start[1] += float(self.rng.uniform(-0.02, 0.02))
            start[2] += float(self.rng.uniform(-0.005, 0.005))
        p.resetBasePositionAndOrientation(
            self.plug, start, p.getQuaternionFromEuler([0, np.pi / 2, 0]))
        p.resetBaseVelocity(self.plug, [0, 0, 0], [0, 0, 0])
        self.plug_world_cid = p.createConstraint(
            self.plug, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], start)
        for _ in range(60):
            p.stepSimulation()
        self.sync()

    # ------------------------------------------------------------------ grasp
    def release_world(self):
        if self.plug_world_cid is not None:
            p.removeConstraint(self.plug_world_cid)
            self.plug_world_cid = None

    def attach_grasp(self):
        """Fallback rigid grasp: fix plug to panda_hand (link 8)."""
        ls = p.getLinkState(self.panda, 11, computeForwardKinematics=True)
        hand_pos = np.array(ls[4])
        hand_R = np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)
        plug_pos, _ = p.getBasePositionAndOrientation(self.plug)
        # link 11 is the TCP frame (zero COM offset)
        rel = hand_R.T @ (np.array(plug_pos) - hand_pos)
        self.grasp_cid = p.createConstraint(
            self.panda, 11, self.plug, -1, p.JOINT_FIXED, [0, 0, 0],
            list(rel), [0, 0, 0])
        p.changeConstraint(self.grasp_cid, maxForce=3000)
        # The rigid grasp assist replaces finger friction; ignore plug<->finger
        # contacts so the constraint is not fought by the position-controlled fingers.
        for link in (8, 9, 10):
            p.setCollisionFilterPair(self.plug, self.panda, -1, link, 0)
        return self.grasp_cid

    def seat_plug(self, pos):
        p.resetBasePositionAndOrientation(
            self.plug, list(map(float, pos)), p.getQuaternionFromEuler([0, np.pi / 2, 0]))
        self.sync()

    def latch_plug(self):
        """Simulate the connector latch after a verified insertion."""
        pos, orn = p.getBasePositionAndOrientation(self.plug)
        self.latch_cid = p.createConstraint(
            self.plug, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0],
            [0, 0, 0], list(pos))

    def hide_plug_marker(self):
        p.resetBasePositionAndOrientation(self.plug_marker, [0, 0, -10], [0, 0, 0, 1])

    def unlatch_plug(self):
        if getattr(self, "latch_cid", None) is not None:
            p.removeConstraint(self.latch_cid)
            self.latch_cid = None

    def detach_grasp(self):
        if getattr(self, "grasp_cid", None) is not None:
            p.removeConstraint(self.grasp_cid)
            self.grasp_cid = None
        for link in (8, 9, 10):
            p.setCollisionFilterPair(self.plug, self.panda, -1, link, 1)

    # ------------------------------------------------------------------ state
    def sync(self):
        pos, _ = p.getBasePositionAndOrientation(self.plug)
        p.resetBasePositionAndOrientation(
            self.plug_marker, (np.asarray(pos) + self.plug_marker_offset).tolist(), [0, 0, 0, 1])

    def step(self, n=1):
        for _ in range(n):
            p.stepSimulation()
            self.sync()

    def tcp_pose(self):
        ls = p.getLinkState(self.panda, 11, computeForwardKinematics=True)
        return np.array(ls[4]), np.array(p.getMatrixFromQuaternion(ls[5])).reshape(3, 3)

    def plug_pose(self):
        pos, orn = p.getBasePositionAndOrientation(self.plug)
        return np.array(pos), np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

    def contact_force(self):
        total = 0.0
        for cp in p.getContactPoints(bodyA=self.plug):
            if cp[2] in self.panel_bodies:
                total += cp[9]
        return total

    def render(self, width=None, height=None):
        cfg = self.cfg
        w = width or cfg.cam_width
        h = height or cfg.cam_height
        img = p.getCameraImage(w, h, self.view, self.proj, renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(img[2], (h, w, 4))[:, :, :3].astype(np.uint8)
        depth = np.reshape(img[3], (h, w))
        return rgb, depth

    def render_side(self, width=320, height=240):
        proj = p.computeProjectionMatrixFOV(55.0, width / height, 0.01, 3.0)
        img = p.getCameraImage(width, height, self.side_view, proj,
                               renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(img[2], (height, width, 4))[:, :, :3].astype(np.uint8)
        return rgb

    def close(self):
        p.disconnect(self.cid)
