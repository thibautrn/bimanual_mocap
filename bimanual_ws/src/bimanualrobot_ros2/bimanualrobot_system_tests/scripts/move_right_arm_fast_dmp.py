#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import subprocess
import re
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import least_squares

# ============================ CONFIG ============================

URDF_PATH = "/home/asurite.ad.asu.edu/troisin/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "leftarm_wrist_2_link"
LOG_DIR   = Path("/home/asurite.ad.asu.edu/troisin/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/good")
WORLD_NAME  = "default"
WORLD_TOPIC = f"/world/{WORLD_NAME}/pose/info"

# ----- Ball models -----
PINNED_BALL_MODEL = "ball_only"  # from ball_only.sdf
PINNED_BALL_POS   = np.array([0.80, -0.55, 1.25], dtype=float)  # must match ball_only.sdf

FREE_BALL_MODEL_NAME = "ball_only_free"
FREE_BALL_SDF = "/home/asurite.ad.asu.edu/troisin/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/ball_only_free.sdf"

BALL_RADIUS    = 0.03
CONTACT_MARGIN = 0.2  # detection margin

RACKET_FRAME = "leftarm_ee_link"  # must exist in URDF

BALL_PRINT_PERIOD_S = 0.2  # just for debug prints via pose/info

IK_JOINTS = [
    "leftarm_shoulder_pan_joint",
    "leftarm_shoulder_lift_joint",
    "leftarm_elbow_joint",
]

ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
JOINTS = [
    "leftarm_shoulder_pan_joint",
    "leftarm_shoulder_lift_joint",
    "leftarm_elbow_joint",
    "leftarm_wrist_1_joint",
    "leftarm_wrist_2_joint",
    "leftarm_wrist_3_joint",
]

ALLOWED_CONTACT_PAIRS = {
    ("rightarm_racket_handle", "rightarm_racket_blade"),
    ("leftarm_racket_handle", "leftarm_racket_blade"),
}

GROUP_NAME = "left_arm"

CYCLE_SECONDS      = 0.025
UPSAMPLE_FACTOR    = 2
BATCH_HORIZON_S    = 0.25
RESAMPLE_HZ        = 40.0
DT_STEP            = 1.0 / RESAMPLE_HZ

LPF_CUTOFF_HZ      = 3.5
SPIKE_MAX_SPEED    = 2.0
MIN_W_DIST_M       = 0.002
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

W_POS    = 1.0
W_REG    = 1e-3
MAX_ITERS= 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE  = 0

# ============================ DMP CONFIG ============================

BASELINE_PATH  = str(LOG_DIR / "baseline.npz")
WEIGHTS_PATH   = str(LOG_DIR / "generated_dmp_weights.npz")
DMP_DT         = DT_STEP
TAU_OVERRIDE   = None
USE_WEIGHTED_OFFSET_GOAL = True
ADDITIONAL_DZ  = 0.0

# ================================================================

class GzBallPoseReader:
    """Optional: reads pose/info for debug; target_regex currently unused for release."""
    def __init__(self, world="default", target_regex=r".*", verbose=False):
        self.topic = f"/world/{world}/pose/info"
        self.target = re.compile(target_regex)
        self.verbose = bool(verbose)

        self._proc = None
        self._th = None
        self._stop = threading.Event()

        self.have = False
        self.xyz  = np.zeros(3, dtype=float)
        self.vel  = np.zeros(3, dtype=float)

        self._last_ts  = None
        self._last_xyz = np.zeros(3, dtype=float)

    def start(self):
        if self._proc is not None:
            return
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1
        )
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
        except Exception:
            pass

    def _dbg(self, *a):
        if self.verbose:
            print("[GzBallPoseReader]", *a)

    def _commit(self, x, y, z, reason=""):
        px = self.xyz[0] if x is None else x
        py = self.xyz[1] if y is None else y
        pz = self.xyz[2] if z is None else z

        now = time.time()
        if self._last_ts is not None:
            dt = max(now - self._last_ts, 1e-6)
            self.vel[:] = [
                (px - self._last_xyz[0]) / dt,
                (py - self._last_xyz[1]) / dt,
                (pz - self._last_xyz[2]) / dt,
            ]
        self.xyz[:] = (px, py, pz)
        self._last_xyz[:] = self.xyz
        self._last_ts = now
        self.have = True
        self._dbg(
            f"COMMIT {reason}: xyz=({px:.6f},{py:.6f},{pz:.6f}) "
            f"vel=({self.vel[0]:.3f},{self.vel[1]:.3f},{self.vel[2]:.3f})"
        )

    def _run(self):
        in_pose = False
        in_pos  = False
        cur_name = ""
        bx = by = bz = None
        name_matched = False

        rx_name   = re.compile(r'name:\s+"([^"]+)"')
        rx_posblk = re.compile(r'position\s*\{([^}]*)\}')
        rx_num_x  = re.compile(r'\bx:\s*([-\d.e+]+)')
        rx_num_y  = re.compile(r'\by:\s*([-\d.e+]+)')
        rx_num_z  = re.compile(r'\bz:\s*([-\d.e+]+)')

        for ln in self._proc.stdout:
            if self._stop.is_set():
                break
            s = ln.lstrip()

            if not in_pose and s.startswith("pose {"):
                in_pose = True
                in_pos = False
                cur_name = ""
                bx = by = bz = None
                name_matched = False
                continue

            if not in_pose:
                m_any = rx_posblk.search(ln)
                if m_any:
                    block = m_any.group(1)
                    mx = rx_num_x.search(block)
                    my = rx_num_y.search(block)
                    mz = rx_num_z.search(block)
                    if mx: bx = float(mx.group(1))
                    if my: by = float(my.group(1))
                    if mz: bz = float(mz.group(1))
                continue

            m_name = rx_name.search(ln)
            if m_name:
                cur_name = m_name.group(1)
                name_matched = bool(self.target.search(cur_name))
                if name_matched and (bx is not None or by is not None or bz is not None):
                    self._commit(bx, by, bz, reason="on_name_match")
                continue

            m_inline = rx_posblk.search(ln)
            if m_inline:
                block = m_inline.group(1)
                mx = rx_num_x.search(block)
                my = rx_num_y.search(block)
                mz = rx_num_z.search(block)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if name_matched:
                    self._commit(bx, by, bz, reason="inline_after_name")
                continue

            if "position {" in ln:
                in_pos = True
                continue

            if in_pos:
                mx = rx_num_x.search(ln)
                my = rx_num_y.search(ln)
                mz = rx_num_z.search(ln)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if "}" in ln:
                    in_pos = False
                    if name_matched:
                        self._commit(bx, by, bz, reason="end_position_block")
                continue

            if "}" in ln and in_pose and not in_pos:
                if name_matched:
                    self._commit(bx, by, bz, reason="end_of_block")
                in_pose = False
                name_matched = False
                continue

    def get(self):
        return bool(self.have), self.xyz.copy(), self.vel.copy()

# ---------------- low-pass EMA ----------------
class LowPassEMA:
    def __init__(self, fc_hz=LPF_CUTOFF_HZ):
        self.fc = float(fc_hz)
        self.y = None
        self.t_last = None

    def update(self, x, t_now):
        x = np.asarray(x, float)
        if self.y is None or self.t_last is None:
            self.y = x.copy()
            self.t_last = float(t_now)
            return self.y
        dt = max(float(t_now - self.t_last), 1e-3)
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.fc * dt)
        self.y = (1.0 - alpha) * self.y + alpha * x
        self.t_last = float(t_now)
        return self.y

# ---------------- Pinocchio helpers ----------------

def build_index_maps(model: pin.Model, joint_names):
    idx_q_vars = []
    for jn in joint_names:
        jid = model.getJointId(jn)
        if jid == 0:
            raise RuntimeError(f"Joint not found in model: {jn}")
        idx_q_vars.append(model.joints[jid].idx_q)

    lb_all = np.array(model.lowerPositionLimit, dtype=float)
    ub_all = np.array(model.upperPositionLimit, dtype=float)
    lb = lb_all[idx_q_vars].copy()
    ub = ub_all[idx_q_vars].copy()
    lb[~np.isfinite(lb)] = -1e9
    ub[~np.isfinite(ub)] = +1e9
    return np.array(idx_q_vars), lb, ub

def build_q_index_map(model: pin.Model):
    """
    Map joint_name -> index in q.
    Uses model.names + getJointId; skips universe (index 0).
    """
    mapping = {}
    for name in model.names[1:]:
        jid = model.getJointId(name)
        if jid == 0:
            continue
        j = model.joints[jid]
        if j.nq > 0:
            mapping[name] = j.idx_q
    return mapping

def q_from_joint_state(model: pin.Model, q_index_map, js: JointState):
    q = pin.neutral(model)
    name_to_pos = dict(zip(js.name, js.position))
    for jname, idx in q_index_map.items():
        if jname in name_to_pos:
            q[idx] = float(name_to_pos[jname])
    return q

def full_q_from_vars(model: pin.Model, idx_q_vars, q_vars):
    q = pin.neutral(model)
    for v, i in zip(q_vars, idx_q_vars):
        q[i] = float(v)
    return q

def residual_wrist_only(q_vars, model, data, fid_wrist, idx_q_vars, target_W,
                        w_pos=W_POS, w_reg=W_REG):
    q = full_q_from_vars(model, idx_q_vars, q_vars)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err_pos = (pW - target_W) * w_pos
    err_reg = w_reg * q_vars
    return np.hstack((err_pos, err_reg))

def solve_wrist_ik_least_squares(robot: RobotWrapper,
                                 fid_wrist: int,
                                 idx_q_vars,
                                 lb, ub,
                                 target_W,
                                 seed):
    model = robot.model
    data  = model.createData()
    res = least_squares(
        residual_wrist_only,
        np.array(seed, float),
        bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS,
        xtol=XTOL, ftol=FTOL, gtol=GTOL,
        verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err

# ---------------- DMP rollout ----------------

def canonical_by_steps(N, dt, tau, alpha_s):
    S = np.empty(int(N), float)
    s = 1.0
    for k in range(int(N)):
        S[k] = s
        s += dt * (-alpha_s * s / tau)
    return S

def design_matrix(S, c, h):
    Phi = np.empty((len(S), len(c)), float)
    for k, s in enumerate(S):
        psi = np.exp(-h * (s - c)**2)
        Phi[k, :] = (psi / (psi.sum() + 1e-12)) * s
    return Phi

def rollout_dmp_3d(y0, g, W, K, D, tau, dt, c, h, alpha_s):
    N   = int(np.round(tau / dt)) + 1
    S   = canonical_by_steps(N, dt, tau, alpha_s)
    Phi = design_matrix(S, c, h)
    Y  = np.zeros((N, 3), float)
    Yd = np.zeros_like(Y)
    Y[0] = y0
    for k in range(N-1):
        acc = np.zeros(3, float)
        for d in range(3):
            f = float(Phi[k].dot(W[d]))
            acc[d] = (
                K*(g[d] - Y[k,d])
                - D*tau*Yd[k,d]
                + K * f * (g[d] - y0[d])
            ) / (tau**2)
        Yd[k+1] = Yd[k] + acc * dt
        Y[k+1]  = Y[k]  + Yd[k+1] * dt
    return Y

class DMPRolloutSource:
    def __init__(self, baseline_path, weights_path, dt,
                 tau_override=None,
                 use_weighted_offset_goal=True,
                 add_dz=0.0):
        b = np.load(baseline_path)
        w = np.load(weights_path, allow_pickle=True)

        c, h = b["c"], b["h"]
        K, D = float(b["K"]), float(b["D"])
        alpha_s = float(b["alpha_s"])
        tau = float(tau_override) if tau_override is not None else float(b["tau"])
        W = w["w"]

        y0_w = w["y0"].astype(float)
        g_w  = w["g"].astype(float)

        y0 = y0_w.copy()
        if use_weighted_offset_goal:
            g = y0 + (g_w - y0_w)
        else:
            g = g_w.copy()
        g = g + np.array([0.0, 0.0, float(add_dz)], float)

        Y = rollout_dmp_3d(y0, g, W, K, D, tau, dt, c, h, alpha_s)

        self.Y = Y
        self.dt = float(dt)
        self.N  = len(Y)
        self.k  = 0

    def next(self):
        if self.k >= self.N:
            return None
        Yk = self.Y[self.k].copy()
        self.k += 1
        return Yk

# ============================ NODE ============================

class DMPRobotBatcher(Node):
    def __init__(self, dmp_source: DMPRolloutSource):
        super().__init__("dmp_robot_batcher")
        self._dmp = dmp_source

        # Pinocchio model
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model

        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # Racket frame
        if not self.model.existFrame(RACKET_FRAME):
            self.get_logger().warn(
                f"Racket frame {RACKET_FRAME} not found; ball release-on-hit disabled."
            )
            self.fid_racket = None
        else:
            self.fid_racket = self.model.getFrameId(RACKET_FRAME)

        # Joint name -> q index
        self.q_index_map = build_q_index_map(self.model)

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # MoveIt validity
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # Smoothing
        self._w_lpf = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W = None
        self._prev_W_t = None

        # IK warm start
        self._last_qvars = None
        self._last_q_cmd = None

        # DMP timing
        self._last_dmp_time = None

        # Ball pose reader (debug only)
        self._ball_reader = GzBallPoseReader(
            world=WORLD_NAME,
            target_regex=r"^ball_only(?:::.+)?$",
            verbose=False,
        )
        self._ball_reader.start()
        self._last_ball_print = 0.0

        # Ball release state
        self.ball_released = False

    # --- basic helpers ---

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        return [float(d.get(n, default)) for n in names]

    def _robot_state_from_qcmd(self, q_cmd):
        js = JointState()
        js.name = JOINTS
        js.position = list(map(float, q_cmd))
        js.header.stamp = self.get_clock().now().to_msg()
        rs = RobotState()
        rs.joint_state = js
        return rs

    def _norm_pair(self, a, b):
        return (a, b) if a <= b else (b, a)

    def _is_state_valid(self, q_cmd) -> bool:
        if not self._gsv.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("GetStateValidity not available; skipping check.")
            return True

        req = GetStateValidity.Request()
        req.robot_state = self._robot_state_from_qcmd(q_cmd)
        req.group_name = GROUP_NAME
        fut = self._gsv.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            self.get_logger().error("GetStateValidity call failed; skipping check.")
            return True

        res = fut.result()
        if res.valid:
            return True

        if res.contacts:
            pairs = [self._norm_pair(c.contact_body_1, c.contact_body_2)
                     for c in res.contacts]
            non_whitelisted = [
                p for p in pairs
                if p not in {self._norm_pair(*pair) for pair in ALLOWED_CONTACT_PAIRS}
            ]
            if not non_whitelisted:
                return True

        self.get_logger().warn("State INVALID (collision or limits). Not sending.")
        return False

    def _send_followtraj_traj(self, q_points, total_dt):
        if not q_points:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        dt_step = float(total_dt) / float(len(q_points))
        t_accum = 0.0
        pts = []
        for q in q_points:
            t_accum += dt_step
            pt = JointTrajectoryPoint()
            pt.positions = list(map(float, q))
            sec = int(t_accum)
            nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)

        goal.trajectory.points = pts
        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    def _get_ball_pose(self):
        return self._ball_reader.get()

    # --- ball release logic ---

    def _compute_racket_position(self):
        if self.fid_racket is None or self._latest_js is None:
            return None
        q = q_from_joint_state(self.model, self.q_index_map, self._latest_js)
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        p = data.oMf[self.fid_racket].translation
        return np.array([p[0], p[1], p[2]], dtype=float)

    def _trigger_ball_release_if_hit(self):
        if self.ball_released:
            return

        racket_pos = self._compute_racket_position()
        if racket_pos is None:
            return

        # Use live ball pose if available, else fallback
        have_ball, ball_pos, _ = self._get_ball_pose()
        center = ball_pos if have_ball else PINNED_BALL_POS

        dist = float(np.linalg.norm(racket_pos - center))
        if dist > (BALL_RADIUS + CONTACT_MARGIN):
            return  # not close enough

        self.get_logger().info(
            f"Racket-ball proximity {dist:.3f} m <= "
            f"{BALL_RADIUS + CONTACT_MARGIN:.3f} -> releasing ball."
        )

        # -------- 1) Delete pinned model: ball_only --------
        try:
            rm_cmd = [
                "gz", "service",
                "-s", f"/world/{WORLD_NAME}/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                # MODEL = 2
                "--req", f'name: "{PINNED_BALL_MODEL}", type: 2',
            ]
            self.get_logger().info("Removing pinned ball: " + " ".join(rm_cmd))
            subprocess.run(rm_cmd, check=False)
        except Exception as e:
            self.get_logger().error(f"Failed to remove {PINNED_BALL_MODEL}: {e}")

        # -------- 2) Spawn free ball via /world/default/create (fast) --------
        try:
            # Build EntityFactory request:
            # sdf_filename: path to ball_only_free.sdf
            # name: desired model name
            # pose: spawn at 'center'
            create_req = (
                f'sdf_filename: "{FREE_BALL_SDF}", '
                f'name: "{FREE_BALL_MODEL_NAME}", '
                f'pose: {{ position: {{ x: {center[0]}, y: {center[1]}, z: {center[2]} }}, '
                f'orientation: {{ x: 0, y: 0, z: 0, w: 1 }} }}'
            )

            spawn_cmd = [
                "gz", "service",
                "-s", f"/world/{WORLD_NAME}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", create_req,
            ]
            self.get_logger().info("Spawning free ball: " + " ".join(spawn_cmd))
            subprocess.run(spawn_cmd, check=False)
        except Exception as e:
            self.get_logger().error(f"Failed to spawn free ball: {e}")
            return

        self.ball_released = True
        self.get_logger().info("Ball released: pinned removed, free spawned.")


    # --- main loop ---

    def run(self):
        # wait for /joint_states
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        self._last_dmp_time = time.time()
        self._prev_W = None
        self._prev_W_t = None
        self._pending_W = None
        self._last_q_cmd = None

        finished = False
        last_tick = 0.0

        while rclpy.ok() and not finished:
            now = time.time()

            # debug ball pose
            have, bp, bv = self._get_ball_pose()
            if have and (now - self._last_ball_print) >= BALL_PRINT_PERIOD_S:
                print(
                    f"[ball_pose_dbg] pos=({bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f}) "
                    f"vel=({bv[0]:.2f},{bv[1]:.2f},{bv[2]:.2f})"
                )
                self._last_ball_print = now

            # ball release on racket "contact"
            if not self.ball_released:
                self._trigger_ball_release_if_hit()

            # advance DMP
            while (now - self._last_dmp_time) >= self._dmp.dt:
                W_raw = self._dmp.next()
                if W_raw is None:
                    finished = True
                    break
                self._last_dmp_time += self._dmp.dt
                self._pending_W = self._w_lpf.update(W_raw, self._last_dmp_time)

            if finished:
                break

            # control tick
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            if self._pending_W is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            W = self._pending_W.copy()

            # spike guard
            if self._prev_W is not None and self._prev_W_t is not None:
                dt_prev = max(now - self._prev_W_t, 1e-3)
                if np.linalg.norm(W - self._prev_W) / dt_prev > SPIKE_MAX_SPEED:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

            # upsample
            if self._prev_W is not None and UPSAMPLE_FACTOR >= 2:
                W_mid = 0.5 * (self._prev_W + W)
                W_list = [W_mid, W]
            else:
                W_list = [W]

            js_now = self._current_positions(JOINTS, default=0.0)

            # IK seed
            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()
            else:
                js_vec = np.array(js_now, dtype=float)
                qvars_seed = np.array(
                    [js_vec[JOINTS.index(jn)] for jn in IK_JOINTS],
                    dtype=float
                )

            # IK for W_list
            q_points = []
            last_qvars_solved = None
            for Wk in W_list:
                ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist,
                    self.idx_q_vars, self.lb, self.ub,
                    Wk, seed=qvars_seed
                )
                if not ok:
                    q_points = []
                    break

                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)

                if not self._is_state_valid(q_cmd):
                    q_points = []
                    break

                q_points.append(q_cmd)
                qvars_seed = qvars
                last_qvars_solved = qvars

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()

            # deadbands
            if not q_points:
                self._prev_W = W.copy()
                self._prev_W_t = now
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    self._prev_W = W.copy()
                    self._prev_W_t = now
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

            # send trajectory
            self._last_q_cmd = q_points[0]
            self._prev_W = W.copy()
            self._prev_W_t = now
            self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

            print(
                f"[tick+DMP] pts={len(q_points)} over {CYCLE_SECONDS:.3f}s | "
                f"W={np.round(W,3)} | finished={finished}"
            )

            rclpy.spin_once(self, timeout_sec=0.01)

        print("[DMP] Finished full rollout.")
        time.sleep(0.5)

# ============================ main ============================

def main():
    dmp_src = DMPRolloutSource(
        baseline_path=BASELINE_PATH,
        weights_path=WEIGHTS_PATH,
        dt=DMP_DT,
        tau_override=TAU_OVERRIDE,
        use_weighted_offset_goal=USE_WEIGHTED_OFFSET_GOAL,
        add_dz=ADDITIONAL_DZ,
    )

    rclpy.init()
    node = DMPRobotBatcher(dmp_src)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
