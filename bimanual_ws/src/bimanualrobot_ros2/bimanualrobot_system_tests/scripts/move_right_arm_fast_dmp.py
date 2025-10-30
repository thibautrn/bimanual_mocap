#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, math
from collections import deque

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

from std_srvs.srv import Trigger
from pathlib import Path

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"     # IK target frame
LOG_DIR   = Path("/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs")

# ---- DMP model (baseline + weights) ----
DMP_BASELINE_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/baseline.npz"
DMP_WEIGHTS_PATH  = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/wrist_20251020_031909_weights.npz"  # pick one
DMP_TAU_SECONDS   = 3  # duration of one DMP stroke; tune (e.g., 0.4–0.8 s)

# IK DOFs
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    # "rightarm_wrist_1_joint",
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

GROUP_NAME = "right_arm"

# Wearable → robot mapping (for optional teleop start pose)
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow
L2 = 0.23293990641364998   # elbow→wrist_2

MIN_W_DIST_M       = 0.002 

# UDP (wearable)
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats

# Loop & timing
CYCLE_SECONDS      = 0.05      # send a new short trajectory ~20 Hz
RESAMPLE_HZ        = 40.0      # JTC-friendly uniform spacing
DT_STEP            = 1.0 / RESAMPLE_HZ  # 0.025 s

# Smoothing / guards
LPF_CUTOFF_HZ      = 3.5       # light smoothing of wrist positions
SPIKE_MAX_SPEED    = 2.0       # m/s (reject absurd jumps)
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)  # tiny deadband on first point

# IK solver params
W_POS = 1.0
W_REG = 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE = 0

# ================================================================

# Shared UDP state
_udp_lock = threading.Lock()
_hand_pos = None
_larm_pos = None
_uarm_pos = None

def udp_listener(port=UDP_PORT):
    """Receive wearable packet and keep the latest sample (thread)."""
    global _hand_pos, _larm_pos, _uarm_pos
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[UDP] Listening on udp://0.0.0.0:{port}")
    unpack = struct.Struct(PACK_FMT).unpack_from
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) < 100:
                continue
            (hw, hx, hy, hz,
             hpx, hpy, hpz,
             lw, lx, ly, lz,
             lpx, lpy, lpz,
             uw, ux, uy, uz,
             upx, upy, upz,
             qw, qx, qy, qz) = unpack(data)
            with _udp_lock:
                _hand_pos = (hpx, hpy, hpz)
                _larm_pos = (lpx, lpy, lpz)
                _uarm_pos = (upx, upy, upz)
        except Exception as e:
            print(f"[UDP ERROR] {e}")

# ---------------- mapping helpers ----------------
def remap_watch_to_base(p):
    """(x,y,z)_watch -> (z, -x, y)_base"""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_right_robot(uarm_watch, larm_watch, hand_watch):
    """Map watch 3 pts (upper, lower, hand) to robot-base wrist (W)."""
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)

    # Right-arm flip (kept)
    Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

    u1 = unit(El - Sh)  # shoulder->elbow dir
    u2 = unit(Wr - El)  # elbow->wrist  dir

    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot        + L2 * u2
    return E_robot, W_robot

# ---------------- DMP helpers ----------------
def load_dmp_model(baseline_path, weights_path):
    b = np.load(baseline_path)
    wfile = np.load(weights_path, allow_pickle=True)
    baseline = {
        "c": b["c"], "h": b["h"],
        "K": float(b["K"]), "D": float(b["D"]),
        "tau": float(b["tau"]),
        "alpha_s": float(b["alpha_s"]),
    }
    weights = {
        "w": wfile["w"],        # (3, M)
        "y0": wfile["y0"],      # (3,)
        "g":  wfile["g"],       # (3,)
    }
    return baseline, weights

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

def rollout_dmp_batch(y0, g, W, K, D, tau, dt, c, h, alpha_s):
    """
    y0,g: (3,) start/goal in base frame for this *stroke*
    W:    (3,M) weights
    Return: Y (N,3) positions
    """
    N = int(np.round(tau / dt)) + 1
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
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=FTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full); pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err

# ============================ NODE ============================

class WatchPathBatcher(Node):
    def __init__(self):
        super().__init__("watch_path_batcher")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

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

        # ---- DMP model (load once) ----
        try:
            self._dmp_baseline, self._dmp_weights = load_dmp_model(DMP_BASELINE_PATH, DMP_WEIGHTS_PATH)
            self.get_logger().info(
                f"DMP loaded: M={len(self._dmp_baseline['c'])}, K={self._dmp_baseline['K']}, "
                f"D={self._dmp_baseline['D']}, alpha_s={self._dmp_baseline['alpha_s']}"
            )
        except Exception as e:
            raise RuntimeError(f"Failed to load DMP files: {e}")

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
                return  # drop spike
            if np.linalg.norm(W - W_prev) < MIN_W_DIST_M:
                return  # too close to last accepted point
        self._w_hist.append((t_now, W.copy()))
        while self._w_hist and (t_now - self._w_hist[0][0]) > self._hist_keep_s:
            self._w_hist.popleft()

    def _current_wrist_fk(self):
        """FK of right wrist in base frame from /joint_states."""
        if self._latest_js is None:
            return None

        # Start from neutral config
        q = pin.neutral(self.model)

        # Map incoming joint states into the Pinocchio config vector
        js_map = dict(zip(self._latest_js.name, self._latest_js.position))
        for jname, qval in js_map.items():
            try:
                jid = self.model.getJointId(jname)
            except Exception:
                continue  # name not in model
            if jid <= 0:
                continue
            # Most of your joints are 1-DoF; write into the first config index for that joint
            idx_q = self.model.joints[jid].idx_q
            nq_j  = self.model.joints[jid].nq  # number of cfg dofs for this joint
            # If nq_j > 0, assign the first dof (typical revolute); extend if you later add multi-dof joints
            if nq_j >= 1:
                q[idx_q] = float(qval)

        # Forward kinematics → wrist pose
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        pW = data.oMf[self.fid_wrist].translation
        return np.array(pW, dtype=float)


    def _robot_state_from_qcmd(self, q_cmd):
        js = JointState()
        js.name = JOINTS
        js.position = list(map(float, q_cmd))
        now = self.get_clock().now().to_msg()
        js.header.stamp = now
        rs = RobotState()
        rs.joint_state = js
        return rs
    
    def _is_state_valid(self, q_cmd) -> bool:
        if not self._gsv.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("GetStateValidity service not available; skipping check.")
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
        if not res.valid:
            self.get_logger().warn("State INVALID (collision or limits). Not sending.")
            if res.contacts:
                pairs = [(c.contact_body_1, c.contact_body_2) for c in res.contacts[:3]]
                self.get_logger().warn(f"Contacts (first): {pairs}")
        return res.valid

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

    # --- main loop ---
    def run(self):
        # wait a moment for /joint_states
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        last_tick = 0.0
        while rclpy.ok():
            now = time.time()
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            # latest wearable sample -> optional smoothing; else FK fallback
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos

            if hp is not None and lp is not None and up is not None:
                _, W_raw = scale_watch_to_right_robot(up, lp, hp)
                W_smooth = self._w_lpf.update(W_raw, now)
                self._append_wrist(W_smooth, now)
                y0 = W_smooth.copy()
            else:
                W_fk = self._current_wrist_fk()
                if W_fk is None:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
                y0 = W_fk

            # ---- DMP rollout: generate a short stroke to a goal ----
            baseline = self._dmp_baseline
            weights  = self._dmp_weights

            # Default goal: use the learned offset from the weights file (goal-invariant retargeting)
            g_offset = (weights["g"] - weights["y0"])    # 3-vector from training
            g = (y0 + g_offset).astype(float)
            # If you have a live target, replace the line above with: g = live_goal_vec

            tau = float(DMP_TAU_SECONDS)

            Y = rollout_dmp_batch(
                y0=y0, g=g, W=weights["w"],
                K=baseline["K"], D=baseline["D"],
                tau=tau, dt=DT_STEP,
                c=baseline["c"], h=baseline["h"],
                alpha_s=baseline["alpha_s"],
            )

            # Skip the first sample (it's y0), otherwise first IK has zero move (deadband)
            if len(Y) <= 1:
                self.get_logger().warn("DMP rollout too short; skipping.")
                continue
            W_list = [Y[k].copy() for k in range(1, len(Y))]

            # current measured joints (for non-IK joints)
            js_now = self._current_positions(JOINTS, default=0.0)

            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()
            else:
                js_vec = np.array(js_now, dtype=float)
                qvars_seed = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)

            # IK each waypoint (warm-start)
            q_points = []
            last_qvars_solved = None
            for i, Wk in enumerate(W_list):
                ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, Wk, seed=qvars_seed
                )
                if not ok:
                    self.get_logger().warn(f"IK failed at waypoint {i}; err={err:.4f}")
                    break

                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)

                if not self._is_state_valid(q_cmd):
                    self.get_logger().warn(f"State invalid at waypoint {i}; dropping stroke.")
                    break

                q_points.append(q_cmd)
                qvars_seed = qvars  # warm-start next point
                last_qvars_solved = qvars 

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()
            if not q_points:
                self.get_logger().warn("No IK points produced; skipping send.")
                continue

            # tiny deadband on the first point to avoid spam
            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    # still send the rest if there is significant movement later
                    # (but generally the first point defines the span)
                    self.get_logger().warn("First point within joint deadband; skipping this stroke.")
                    continue

            # remember seeds
            self._last_qvars = qvars_seed
            self._last_q_cmd = q_points[0]

            self._send_followtraj_traj(q_points, dt_step=DT_STEP)
            print(f"[dmp] sent pts={len(q_points)} over {DMP_TAU_SECONDS:.3f}s | y0={np.round(y0,3)} -> g={np.round(g,3)}")

# ============================ main ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = WatchPathBatcher()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
