#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, threading
from collections import deque
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
# from ros_gz_interfaces.msg import Pose_V

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import least_squares

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/asurite.ad.asu.edu/troisin/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"     # IK target frame
LOG_DIR   = Path("/home/asurite.ad.asu.edu/troisin/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs")

# --------- BALL POSE INPUT (from Gazebo via ros_gz_bridge) ---------
WORLD_NAME   = "default"                                  # your world
WORLD_TOPIC  = f"/world/{WORLD_NAME}/pose/info"           # bridged topic
BALL_NAMES   = {"end_ball", "rope_square_rig::end_ball"}  # accept either name
BALL_PRINT_PERIOD_S = 0.2                                 # print at most 5 Hz


# IK DOFs
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    # "rightarm_wrist_1_joint",  # not used in IK (as before)
]

# Controller & joint order (must match the controller)
ACTION_NAME = "/right_arm_controller/follow_joint_trajectory"
JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    "rightarm_wrist_1_joint",
    "rightarm_wrist_2_joint",
    "rightarm_wrist_3_joint",
]

ALLOWED_CONTACT_PAIRS = {
        ("leftarm_racket_handle", "leftarm_racket_blade"),
        ("rightarm_racket_handle", "rightarm_racket_blade"),
    }

GROUP_NAME = "right_arm"

# Loop & timing
CYCLE_SECONDS      = 0.05      # send a new short trajectory ~20 Hz
BATCH_HORIZON_S    = 0.25      # each goal spans the recent 250 ms path
RESAMPLE_HZ        = 40.0
DT_STEP            = 1.0 / RESAMPLE_HZ  # 0.025 s

# Smoothing / guards
LPF_CUTOFF_HZ      = 3.5       # light smoothing of wrist positions
SPIKE_MAX_SPEED    = 2.0       # m/s (reject absurd jumps)
MIN_W_DIST_M       = 0.002     # tiny distance deadband
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

# IK solver params
W_POS = 1.0
W_REG = 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE = 0

# ============================ DMP CONFIG ============================
# Reuses your preview_dmp settings, but runs online here.
BASELINE_PATH  = str(LOG_DIR / "baseline.npz")
# WEIGHTS_PATH   = str(LOG_DIR / "wrist_20251103_004433_weights.npz")  # change to your chosen weights file
# WEIGHTS_PATH   = str(LOG_DIR / "wrist_20251103_004509_weights.npz")  # change to your chosen weights file
WEIGHTS_PATH   = str(LOG_DIR / "wrist_20251103_004546_weights.npz")  # change to your chosen weights file
DMP_DT         = DT_STEP           # keep rollout at controller-friendly rate
TAU_OVERRIDE   = None              # None -> use baseline['tau']; or set e.g. 3.0
USE_WEIGHTED_OFFSET_GOAL = True    # True: g = y0 + (g_w - y0_w); False: g = g_w
ADDITIONAL_DZ  = 0.0               # e.g., 0.10 to lift path by 10 cm

# ================================================================

import subprocess, re


import subprocess, re, threading, time
import numpy as np

class GzBallPoseReader:
    """
    Robust reader for Gazebo's aggregate pose stream.
    Mirrors the working awk: commit at end of each `pose { ... }` block
    for entities whose name matches the target (end_ball / ::end_ball).
    - Handles inline or multi-line `position{}`.
    - Handles delta updates (only x/y/z lines).
    - Fills missing components with last-known values.
    """

    def __init__(self, world="default", target_regex=r"(?:^end_ball$|::end_ball$)", verbose=False):
        self.topic = f"/world/{world}/pose/info"
        self.target = re.compile(target_regex)
        self.verbose = bool(verbose)

        self._proc = None
        self._th = None
        self._stop = threading.Event()

        # public state
        self.have = False
        self.xyz  = np.zeros(3, dtype=float)
        self.vel  = np.zeros(3, dtype=float)

        # internals
        self._last_ts  = None
        self._last_xyz = np.zeros(3, dtype=float)

    # ---------- lifecycle ----------
    def start(self):
        if self._proc is not None:
            return
        if self.verbose:
            print("[GzBallPoseReader] spawn:", "gz topic -e -t", self.topic)
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

    # ---------- helpers ----------
    def _dbg(self, *a):
        if self.verbose:
            print("[GzBallPoseReader]", *a)

    def _commit(self, x, y, z, reason=""):
        # fill missing from last-known
        px = self.xyz[0] if x is None else x
        py = self.xyz[1] if y is None else y
        pz = self.xyz[2] if z is None else z

        now = time.time()
        if self._last_ts is not None:
            dt = max(now - self._last_ts, 1e-6)
            self.vel[:] = [(px - self._last_xyz[0]) / dt,
                           (py - self._last_xyz[1]) / dt,
                           (pz - self._last_xyz[2]) / dt]
        self.xyz[:] = (px, py, pz)
        self._last_xyz[:] = self.xyz
        self._last_ts = now
        self.have = True
        self._dbg(f"COMMIT {reason}: xyz=({px:.6f},{py:.6f},{pz:.6f}) vel=({self.vel[0]:.3f},{self.vel[1]:.3f},{self.vel[2]:.3f})")

    # ---------- core parsing loop ----------
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

            # start pose
            if not in_pose and s.startswith("pose {"):
                in_pose = True; in_pos = False
                cur_name = ""; bx = by = bz = None
                name_matched = False
                self._dbg("POSE{")
                continue

            # outside pose: sometimes position appears anyway
            if not in_pose:
                m_any = rx_posblk.search(ln)
                if m_any:
                    block = m_any.group(1)
                    mx = rx_num_x.search(block); my = rx_num_y.search(block); mz = rx_num_z.search(block)
                    if mx: bx = float(mx.group(1))
                    if my: by = float(my.group(1))
                    if mz: bz = float(mz.group(1))
                    self._dbg(f"pos-outside: x={bx} y={by} z={bz}")
                continue

            # inside pose: name
            m_name = rx_name.search(ln)
            if m_name:
                cur_name = m_name.group(1)
                name_matched = bool(self.target.search(cur_name))
                self._dbg("NAME:", cur_name, "match=", name_matched)
                if name_matched and (bx is not None or by is not None or bz is not None):
                    self._commit(bx, by, bz, reason="on_name_match")
                continue

            # inline position
            m_inline = rx_posblk.search(ln)
            if m_inline:
                block = m_inline.group(1)
                mx = rx_num_x.search(block); my = rx_num_y.search(block); mz = rx_num_z.search(block)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                self._dbg(f"pos-inline: x={bx} y={by} z={bz}")
                if name_matched:
                    self._commit(bx, by, bz, reason="inline_after_name")
                continue

            # multiline position
            if "position {" in ln:
                in_pos = True; self._dbg("  POSITION{"); continue
            if in_pos:
                mx = rx_num_x.search(ln); my = rx_num_y.search(ln); mz = rx_num_z.search(ln)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if "}" in ln:
                    in_pos = False
                    self._dbg(f"  }}POSITION  (x={bx} y={by} z={bz})")
                    if name_matched:
                        self._commit(bx, by, bz, reason="end_position_block")
                continue

            # end pose
            if "}" in ln and in_pose and not in_pos:
                self._dbg("}POSE")
                if name_matched:
                    self._commit(bx, by, bz, reason="end_of_block")
                in_pose = False; name_matched = False
                continue


    # ---------- public getter ----------
    def get(self):
        """Returns (have_pose: bool, xyz: np.array(3), vel: np.array(3))"""
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
            self.y = x.copy(); self.t_last = float(t_now)
            return self.y
        dt = max(float(t_now - self.t_last), 1e-3)
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.fc * dt)
        self.y = (1.0 - alpha) * self.y + alpha * x
        self.t_last = float(t_now)
        return self.y

# ---------------- Pinocchio IK ----------------
def build_index_maps(model: pin.Model, joint_names):
    idx_q_vars = []
    for jn in joint_names:
        jid = model.getJointId(jn)
        if jid == 0:
            raise RuntimeError(f"Joint not found in model: {jn}")
        idx_q_vars.append(model.joints[jid].idx_q)
    lb_all = np.array(model.lowerPositionLimit, dtype=float)
    ub_all = np.array(model.upperPositionLimit, dtype=float)
    lb = lb_all[idx_q_vars].copy(); ub = ub_all[idx_q_vars].copy()
    lb[~np.isfinite(lb)] = -1e9; ub[~np.isfinite(ub)] = +1e9
    return np.array(idx_q_vars), lb, ub

def full_q_from_vars(model: pin.Model, idx_q_vars, q_vars):
    q = pin.neutral(model)
    for v, i in zip(q_vars, idx_q_vars):
        q[i] = float(v)
    return q

def residual_wrist_only(q_vars, model, data, fid_wrist, idx_q_vars, target_W, w_pos=W_POS, w_reg=W_REG):
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
    model = robot.model; data  = model.createData()
    res = least_squares(
        residual_wrist_only, np.array(seed, float),
        bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full); pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err

# ---------------- DMP rollout (discrete, 3D) ----------------
def canonical_by_steps(N, dt, tau, alpha_s):
    S = np.empty(int(N), float); s = 1.0
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
            acc[d] = (K*(g[d] - Y[k,d]) - D*tau*Yd[k,d] + K * f * (g[d] - y0[d])) / (tau**2)
        Yd[k+1] = Yd[k] + acc * dt
        Y[k+1]  = Y[k]  + Yd[k+1] * dt
    return Y

class DMPRolloutSource:
    """Precompute the DMP wrist path Y (Nx3) and yield one sample per call."""
    def __init__(self, baseline_path, weights_path, dt, tau_override=None,
                 use_weighted_offset_goal=True, add_dz=0.0):
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
        g  = (y0 + (g_w - y0_w)) if use_weighted_offset_goal else g_w.copy()
        g  = g + np.array([0.0, 0.0, float(add_dz)], float)
        Y = rollout_dmp_3d(y0, g, W, K, D, tau, dt, c, h, alpha_s)
        self.Y = Y
        self.dt = float(dt)
        self.N  = len(Y)
        self.k  = 0

    def next(self):
        if self.k >= self.N:
            return None  # finished
        Yk = self.Y[self.k].copy()
        self.k += 1
        return Yk

# ============================ NODE ============================

class DMPRobotBatcher(Node):
    def __init__(self, dmp_source: DMPRolloutSource):
        super().__init__("dmp_robot_batcher")
        self._dmp = dmp_source

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # MoveIt validity
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # Wrist history (smoothed)
        self._w_lpf = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._w_hist = deque()  # (t, W_smoothed)
        self._hist_keep_s = 0.6

        # IK warm-start
        self._last_qvars = None
        self._last_q_cmd = None

        # DMP timing bookkeeping
        self._last_dmp_time = None  # when we last consumed a DMP sample

                # --- Ball pose state (WORLD frame) ---

        self._last_ball_print = 0.0  # for throttled prints

        # Subscribe to Gazebo world pose stream (via ros_gz_bridge parameter_bridge)
        self._ball_reader = GzBallPoseReader(world="default", verbose=False)  # set True to debug
        self._ball_reader.start()


    # --- helpers ---
    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        return [float(d.get(n, default)) for n in names]

    def _append_wrist(self, W, t_now):
        if self._w_hist:
            t_prev, W_prev = self._w_hist[-1]
            dt = max(t_now - t_prev, 1e-3)
            if np.linalg.norm(W - W_prev) / dt > SPIKE_MAX_SPEED:
                return
            if np.linalg.norm(W - W_prev) < MIN_W_DIST_M:
                return
        self._w_hist.append((t_now, W.copy()))
        while self._w_hist and (t_now - self._w_hist[0][0]) > self._hist_keep_s:
            self._w_hist.popleft()

    def _resample_recent_path(self, t_now, horizon_s=BATCH_HORIZON_S, dt=DT_STEP):
        if not self._w_hist:
            return []
        t_start = t_now - horizon_s
        ts = np.arange(t_start + dt, t_now + 1e-9, dt)
        times = np.array([t for (t, _) in self._w_hist], float)
        Ws    = np.array([w for (_, w) in self._w_hist], float)
        if len(times) < 2:
            return []
        W_list = []
        for tk in ts:
            tk = float(np.clip(tk, times[0], times[-1]))
            j = int(np.searchsorted(times, tk, side="right") - 1)
            j = max(0, min(j, len(times)-2))
            t0, t1 = times[j], times[j+1]
            a = 0.0 if (t1 <= t0) else (tk - t0) / (t1 - t0)
            Wk = (1.0 - a) * Ws[j] + a * Ws[j+1]
            W_list.append(Wk)
        return W_list

    def _robot_state_from_qcmd(self, q_cmd):
        js = JointState()
        js.name = JOINTS
        js.position = list(map(float, q_cmd))
        js.header.stamp = self.get_clock().now().to_msg()
        rs = RobotState(); rs.joint_state = js
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

        # If invalid, inspect contacts
        if res.contacts:
            pairs = [(self._norm_pair(c.contact_body_1, c.contact_body_2)) for c in res.contacts]
            # self.get_logger().warn(f"Contacts (first): {pairs[:3]}")
            # keep only contacts that are NOT whitelisted
            non_whitelisted = [
                p for p in pairs
                if p not in { self._norm_pair(*pair) for pair in ALLOWED_CONTACT_PAIRS }
            ]
            if not non_whitelisted:
                # self.get_logger().warn("Only whitelisted tool self-contacts; overriding to VALID.")
                return True

        self.get_logger().warn("State INVALID (collision or limits). Not sending.")
        return False

    def _send_followtraj_traj(self, q_points, dt_step):
        if not q_points:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        t_accum = 0.0
        pts = []
        for q in q_points:
            t_accum += dt_step
            pt = JointTrajectoryPoint()
            pt.positions = list(map(float, q))
            sec = int(t_accum); nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)
        goal.trajectory.points = pts
        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    # def _on_pose_v(self, msg: Pose_V):
    #     # Find the ball entry by name (some Gazebo builds use "end_ball", others "model::end_ball")
    #     for e in msg.pose:
    #         name = e.name
    #         if (name in BALL_NAMES) or name.endswith("::end_ball"):
    #             p = e.pose.position
    #             # timestamp (prefer msg header; fallback to node clock)
    #             if (msg.header.stamp.sec or msg.header.stamp.nanosec):
    #                 t_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
    #             else:
    #                 t_ns = int(self.get_clock().now().nanoseconds())

    #             # velocity (finite difference)
    #             if self._ball_last_t is not None:
    #                 dt = max((t_ns - self._ball_last_t) * 1e-9, 1e-6)
    #                 self._ball_vel[:] = (
    #                     (p.x - self._ball_last_p[0]) / dt,
    #                     (p.y - self._ball_last_p[1]) / dt,
    #                     (p.z - self._ball_last_p[2]) / dt
    #                 )

    #             self._ball_pos[:] = (p.x, p.y, p.z)
    #             self._ball_last_t  = t_ns
    #             self._ball_last_p  = (p.x, p.y, p.z)
    #             self._ball_have    = True
    #             return  # found the ball; done

    def _get_ball_pose(self):
        return (bool(self._ball_reader.have),
                self._ball_reader.xyz.copy(),
                self._ball_reader.vel.copy())



    # --- main loop ---
    def run(self):
        # wait a moment for /joint_states
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

    # use p for IK/DMP targetting
        last_tick = 0.0
        self._last_dmp_time = time.time()

        finished = False
        while rclpy.ok() and not finished:
            now = time.time()
            now_s = time.time()
            have, bp, bv = self._ball_reader.get()
            if have and (now_s - self._last_ball_print) >= BALL_PRINT_PERIOD_S:
                print(f"[ball] pos=({bp[0]:.3f}, {bp[1]:.3f}, {bp[2]:.3f})  "
                    f"vel=({bv[0]:.2f}, {bv[1]:.2f}, {bv[2]:.2f}) m/s")
                self._last_ball_print = now_s


            # Consume DMP samples at DMP_DT to maintain 40 Hz stream
            while (now - self._last_dmp_time) >= self._dmp.dt:
                W_raw = self._dmp.next()
                if W_raw is None:
                    finished = True
                    break
                self._last_dmp_time += self._dmp.dt
                W_smooth = self._w_lpf.update(W_raw, self._last_dmp_time)
                self._append_wrist(W_smooth, self._last_dmp_time)

            # Batch and send at CYCLE_SECONDS
            if now - last_tick >= CYCLE_SECONDS:
                last_tick = now

                W_list = self._resample_recent_path(now, horizon_s=BATCH_HORIZON_S, dt=DT_STEP)
                if not W_list:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

                js_now = self._current_positions(JOINTS, default=0.0)
                if self._last_qvars is not None:
                    qvars_seed = self._last_qvars.copy()
                else:
                    js_vec = np.array(js_now, dtype=float)
                    qvars_seed = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)

                q_points = []
                last_qvars_solved = None
                for Wk in W_list:
                    ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                        self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, Wk, seed=qvars_seed
                    )
                    if not ok:
                        break
                    q_cmd = list(js_now)
                    for jn, val in zip(IK_JOINTS, qvars):
                        if jn in JOINTS:
                            q_cmd[JOINTS.index(jn)] = float(val)
                    if not self._is_state_valid(q_cmd):
                        break
                    q_points.append(q_cmd)
                    qvars_seed = qvars
                    last_qvars_solved = qvars

                if last_qvars_solved is not None:
                    self._last_qvars = last_qvars_solved.copy()
                if not q_points:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

                if self._last_q_cmd is not None:
                    dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                    if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                        rclpy.spin_once(self, timeout_sec=0.01)
                        continue

                self._last_q_cmd = q_points[0]
                self._send_followtraj_traj(q_points, dt_step=DT_STEP)

                print(f"[batch] pts={len(q_points)} over {BATCH_HORIZON_S:.3f}s | "
                      f"W_last={np.round(self._w_hist[-1][1],3)} | finished={finished}")

            rclpy.spin_once(self, timeout_sec=0.01)

        print("[DMP] Finished full rollout.")
        # Optional: hold last pose briefly
        time.sleep(0.5)

# ============================ main ============================

def main():
    # Build DMP source (precompute full path once)
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
