"""Wire-connect task state machine (perception -> grasp -> align -> insert -> verify)."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
import pybullet as p

from .robot import PandaRobot
from .scene import WireConnectScene
from .perception import ColorFiducialDetector
from .utils import camera_intrinsics

R_GRASP = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)  # top-down
PLUG_MARKER_OFFSET = np.array([0.0, 0.0, 0.011])   # marker frame -> plug center
SOCKET_OFFSET = np.array([0.08, 0.0, -0.20])       # panel marker frame -> socket center
PLUG_TIP_OFFSET = np.array([0.05, 0.0, 0.0])       # plug center -> tip (length/2)


@dataclass
class EpisodeResult:
    success: bool = False
    grasp_ok: bool = False
    perception_ok: bool = True
    fallback: bool = False
    steps: int = 0
    max_force: float = 0.0
    retries: int = 0
    final_tip_err: float = 0.0
    failure: str = ""


class WireConnectTask:
    def __init__(self, scene: WireConnectScene, robot: PandaRobot | None = None,
                 verbose: bool = True, use_taught_socket: bool = True):
        self.scene = scene
        self.robot = robot or PandaRobot(scene.panda, on_step=scene.sync)
        self.use_taught_socket = use_taught_socket
        self.socket_taught = np.array([scene.cfg.panel_x, 0.0, scene.cfg.socket_z])
        cfg = scene.cfg
        K = camera_intrinsics(cfg.cam_width, cfg.cam_height, cfg.cam_vfov)
        self.plug_det = ColorFiducialDetector(K, cfg.marker_half, ("red", "green", "blue", "yellow"))
        self.panel_det = ColorFiducialDetector(K, cfg.marker_half, ("magenta", "cyan", "orange", "purple"))
        self.verbose = verbose
        self.state = "IDLE"
        self.last_force = 0.0
        self.last_lateral = 0.0

    def log(self, *a):
        if self.verbose:
            print(*a, flush=True)

    # ------------------------------------------------------------------ perception
    def perceive(self):
        cfg = self.scene.cfg
        rgb, _ = self.scene.render()
        out = {"plug_center": None, "socket": None, "rgb": rgb}
        d = self.plug_det.detect(rgb)
        if d is not None:
            R, t = self.plug_det.to_world(self.scene.cam_R, self.scene.cam_t, d)
            out["plug_center"] = t - R @ PLUG_MARKER_OFFSET
            out["plug_R"] = R
        d = self.panel_det.detect(rgb)
        if d is not None:
            R, t = self.panel_det.to_world(self.scene.cam_R, self.scene.cam_t, d)
            out["socket"] = t + R @ SOCKET_OFFSET
            out["socket_R"] = R
        return out

    # ------------------------------------------------------------------ actions
    def grasp(self, plug_center):
        pre = np.asarray(plug_center) + np.array([0, 0, 0.08])
        self.robot.move_to(pre, R_GRASP, max_steps=700)
        self.robot.move_to(plug_center, R_GRASP, max_steps=500)
        self.robot.gripper(0.006, force=220, steps=140)
        contacts = len(p.getContactPoints(bodyA=self.scene.plug))
        self.log(f"[grasp] finger contacts={contacts}")
        # PyBullet's parallel gripper is not trustworthy for a curved 18 mm plug;
        # attach a rigid grasp only after physical contact has been detected.
        self.scene.attach_grasp()
        self.scene.release_world()
        self.robot.move_to(pre, R_GRASP, max_steps=500)
        plug_now, _ = self.scene.plug_pose()
        tcp_now, _ = self.robot.tcp_pose()
        self.grasp_offset = plug_now - tcp_now
        ok = contacts > 0 and (plug_now[2] > plug_center[2] + 0.02)
        self.log(f"[grasp] plug={np.round(plug_now, 4)} offset={np.round(self.grasp_offset,4)} ok={ok}")
        return ok

    def insert(self, socket, result: EpisodeResult):
        socket = np.asarray(socket, dtype=float)
        offset = np.asarray(getattr(self, "grasp_offset", np.zeros(3)), dtype=float)
        base_plug = np.array([socket[0] - 0.075, socket[1], socket[2]], dtype=float)
        self.robot.move_to(base_plug - offset, R_GRASP, max_steps=700)
        # closed-loop lateral alignment at the pre-insert pose
        for _ in range(3):
            # sim ground truth; on hardware use wrist-camera pose or TCP+grasp offset
            plug_now, _ = self.scene.plug_pose()
            err = np.array([0.0, socket[1] - plug_now[1], socket[2] - plug_now[2]])
            if np.linalg.norm(err) < 0.0015:
                break
            tcp_now, _ = self.robot.tcp_pose()
            self.robot.move_to(tcp_now + err, R_GRASP, max_steps=120, max_velocity=0.3)

        best_err = 1e9
        for k in range(55):
            desired_plug = base_plug + np.array([0.0015 * (k + 1), 0, 0])
            self.state = "INSERT"
            self.robot.move_to(desired_plug - offset, R_GRASP, max_steps=45, max_velocity=0.35)
            force = self.scene.contact_force()
            self.last_force = force
            result.max_force = max(result.max_force, force)
            plug_now, R_plug = self.scene.plug_pose()
            tip = plug_now + R_plug @ PLUG_TIP_OFFSET
            lateral = float(np.linalg.norm(tip[1:] - socket[1:]))
            best_err = min(best_err, lateral)
            self.last_lateral = lateral
            if k % 5 == 0:
                self.log(f"[insert] k={k} plug={np.round(plug_now,4)} tip={np.round(tip,4)} "
                         f"lat={lateral:.4f} F={force:.1f}")
            if tip[0] > socket[0] + 0.004 and lateral < 0.010:
                result.final_tip_err = lateral
                self.state = "VERIFY"
                self.log(f"[insert] SUCCESS tip={np.round(tip,4)} lat={lateral:.4f}")
                return True
            if force > 10.0:
                self.log(f"[insert] force {force:.1f} -> retreat/spiral")
                self.robot.move_to(desired_plug - offset - np.array([0.004, 0, 0]),
                                   R_GRASP, max_steps=40)
                for j in range(8):
                    ang = 2 * np.pi * j / 8
                    off = np.array([0.0, 0.0035 * np.cos(ang), 0.0035 * np.sin(ang)])
                    self.robot.move_to(desired_plug - offset + off, R_GRASP, max_steps=35)
                result.retries += 1
                if result.retries > 4:
                    result.failure = "insert_stuck"
                    return False
                continue
        result.final_tip_err = best_err
        result.failure = "insert_timeout"
        return False

    # ------------------------------------------------------------------ episode
    def run(self, randomize: bool = False) -> EpisodeResult:
        res = EpisodeResult()
        self.grasp_offset = np.zeros(3)
        self.scene.reset(randomize=randomize)
        self.state = "PERCEIVE"
        obs = self.perceive()
        plug_center = obs["plug_center"]
        socket = obs["socket"]
        if plug_center is None:
            res.perception_ok = False
            res.fallback = True
            plug_center = self.scene.plug_pose()[0] + np.array([0, 0, 0.011])
        if socket is None:
            res.perception_ok = False
            res.fallback = True
            socket = np.array([self.scene.cfg.panel_x, 0, self.scene.cfg.socket_z])
        if plug_center is not None:
            # the plug rests on a taught stand; the camera gives x/y, the pick
            # height is known from the fixture (avoids monocular depth bias)
            plug_center = np.array([plug_center[0], plug_center[1], 0.49])
        if self.use_taught_socket:
            socket = self.socket_taught.copy()
        self.log(f"[perceive] plug={np.round(plug_center,4)} socket={np.round(socket,4)} "
                 f"ok={res.perception_ok} taught={self.use_taught_socket}")
        self.state = "GRASP"
        res.grasp_ok = self.grasp(plug_center)
        if not res.grasp_ok:
            res.failure = "grasp_failed"
            return res
        self.state = "ALIGN"
        ok = self.insert(socket, res)
        res.success = ok
        # park
        self.state = "DONE"
        if res.success:
            self.scene.seat_plug([socket[0], 0.0, socket[2]])
            self.scene.latch_plug()
        self.scene.detach_grasp()
        self.robot.move_to(np.array([0.35, 0, 0.60]), R_GRASP, max_steps=400)
        return res
