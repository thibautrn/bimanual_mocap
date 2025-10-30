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
LOG_DIR = Path("/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs")  # change if you want

# IK DOFs
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    "rightarm_wrist_1_joint",
    "rightarm_wrist_2_joint",
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

# Wearable → robot mapping
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow
L2 = 0.23293990641364998   # elbow→wrist_2

MIN_W_DIST_M       = 0.002 

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats

# Loop & timing
CYCLE_SECONDS      = 0.05      # send a new short trajectory ~20 Hz
BATCH_HORIZON_S    = 0.25      # each goal spans the *recent* 250 ms path
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
_hand_quat = None  

def udp_listener(port=UDP_PORT):
    """Receive wearable packet and keep the latest sample (thread)."""
    global _hand_pos, _larm_pos, _uarm_pos, _hand_quat
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
                _hand_quat = (hw, hx, hy, hz)
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

    # Right-arm flip kept from your previous mapping
    Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

    u1 = unit(El - Sh)  # shoulder->elbow dir
    u2 = unit(Wr - El)  # elbow->wrist  dir

    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot        + L2 * u2
    return E_robot, W_robot

def quat_to_rot(wxyz: tuple|list|np.ndarray) -> np.ndarray:
    """Quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    w, x, y, z = map(float, wxyz)
    n = w*w + x*x + y*y + z*z
    if n < 1e-12: return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    xx, xy, xz = x*x*s, x*y*s, x*z*s
    yy, yz     = y*y*s, y*z*s
    zz         = z*z*s
    return np.array([
        [1-(yy+zz), xy-wz,     xz+wy],
        [xy+wz,     1-(xx+zz), yz-wx],
        [xz-wy,     yz+wx,     1-(xx+yy)]
    ], float)

def lh_to_rh_R(R_lh: np.ndarray) -> np.ndarray:
    """Unity-LH (X=Right,Y=Up,Z=Fwd) -> ROS-RH (X=Fwd,Y=Left,Z=Up): R' = P R P^T."""
    P = np.array([[ 0, 0, 1],
                  [-1, 0, 0],
                  [ 0, 1, 0]], float)
    return P @ R_lh @ P.T

def wrist_left_right_from_quat_lh(q_lh):
    """
    Map Unity-LH quat -> ROS-RH basis (same P R P^T you already use),
    apply your axis remap, and compute a single left↔right tilt (β).
    Positive β = tip to robot's right.
    """
    # quat -> R (LH)
    R_lh = quat_to_rot(q_lh)

    # LH (X=Right,Y=Up,Z=Fwd) -> RH (X=Fwd,Y=Left,Z=Up)
    P = np.array([[ 0, 0, 1],
                  [-1, 0, 0],
                  [ 0, 1, 0]], float)
    R_ros = P @ R_lh @ P.T

    # your axis remap: forward, left, up from R_ros
    ex = R_ros @ np.array([0.0, 1.0, 0.0])   # forward
    ey = R_ros @ np.array([-1.0, 0.0, 0.0])  # left
    ez = R_ros @ np.array([0.0, 0.0, 1.0])   # up
    ez = ez / (np.linalg.norm(ez) + 1e-12)

    X = np.array([1,0,0.], float)  # fwd
    Y = np.array([0,1,0.], float)  # left
    Z = np.array([0,0,1.], float)  # up

    # left↔right tilt only (use 'up' axis vs Z)
    beta = float(np.arctan2(ez @ (-Y), ez @ Z))  # + => tip right
    return beta


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
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
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


    # --- helpers ---
    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        return [float(d.get(n, default)) for n in names]

    def _append_wrist(self, W, t_now):
        # reject absurd spikes (simple speed guard relative to last)
        if self._w_hist:
            t_prev, W_prev = self._w_hist[-1]
            dt = max(t_now - t_prev, 1e-3)
            # speed spike guard
            if np.linalg.norm(W - W_prev) / dt > SPIKE_MAX_SPEED:
                return  # drop spike
            # NEW: distance deadband (skip tiny moves)
            if np.linalg.norm(W - W_prev) < MIN_W_DIST_M:
                return  # too close to last accepted point

        self._w_hist.append((t_now, W.copy()))
        # prune old
        while self._w_hist and (t_now - self._w_hist[0][0]) > self._hist_keep_s:
            self._w_hist.popleft()

    def _resample_recent_path(self, t_now, horizon_s=BATCH_HORIZON_S, dt=DT_STEP):
        """Return uniformly sampled wrist waypoints over the last horizon_s."""
        if not self._w_hist:
            return []
        t_start = t_now - horizon_s
        ts = np.arange(t_start + dt, t_now + 1e-9, dt)  # strictly increasing for time_from_start
        # gather arrays
        times = np.array([t for (t, _) in self._w_hist], float)
        Ws    = np.array([w for (_, w) in self._w_hist], float)  # shape (M,3)
        if len(times) < 2:
            return []
        # interpolate component-wise (linear)
        W_list = []
        for tk in ts:
            tk = float(np.clip(tk, times[0], times[-1]))
            # indices for interpolation
            j = int(np.searchsorted(times, tk, side="right") - 1)
            j = max(0, min(j, len(times)-2))
            t0, t1 = times[j], times[j+1]
            a = 0.0 if (t1 <= t0) else (tk - t0) / (t1 - t0)
            Wk = (1.0 - a) * Ws[j] + a * Ws[j+1]
            W_list.append(Wk)
        return W_list  # list of np.array(3,)

    def _robot_state_from_qcmd(self, q_cmd):
        js = JointState()
        js.name = JOINTS
        js.position = list(map(float, q_cmd))
        # optional stamp
        now = self.get_clock().now().to_msg()
        js.header.stamp = now

        rs = RobotState()
        rs.joint_state = js
        return rs
    
    def _is_state_valid(self, q_cmd) -> bool:
        if not self._gsv.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("GetStateValidity service not available; skipping check.")
            return True  # fallback: allow

        req = GetStateValidity.Request()
        req.robot_state = self._robot_state_from_qcmd(q_cmd)
        req.group_name = GROUP_NAME

        fut = self._gsv.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            self.get_logger().error("GetStateValidity call failed; skipping check.")
            return True  # conservative fallback

        res = fut.result()
        if not res.valid:
            self.get_logger().warn("State INVALID (collision or limits). Not sending.")
            # (Optional) print first few contacts for debugging
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

            # latest wearable sample → wrist target (smoothed & buffered)
            with _udp_lock:
                hp, lp, up, hq = _hand_pos, _larm_pos, _uarm_pos, _hand_quat
            if hp is None or lp is None or up is None or hq is None:
                continue
            _, W_raw = scale_watch_to_right_robot(up, lp, hp)
            W_smooth = self._w_lpf.update(W_raw, now)

            self._append_wrist(W_smooth, now)

            # resample last horizon_s path at uniform rate
            W_list = self._resample_recent_path(now, horizon_s=BATCH_HORIZON_S, dt=DT_STEP)
            if not W_list:
                continue

            # current measured joints (for non-IK joints)
            js_now = self._current_positions(JOINTS, default=0.0)

            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()   # warm start from last good IK
            else:
                # Project the *measured* joint state onto the IK DOFs (better than zeros)
                js_vec = np.array(js_now, dtype=float)
                qvars_seed = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)


            # IK each waypoint (warm-start)
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
                
                alpha= wrist_left_right_from_quat_lh(hq)

                # clamp to limits
                i_w1_full = JOINTS.index("rightarm_wrist_1_joint")

                j_w1 = self.model.getJointId("rightarm_wrist_1_joint")
                self._w1_lo = float(self.model.lowerPositionLimit[self.model.joints[j_w1].idx_q])
                self._w1_hi = float(self.model.upperPositionLimit[self.model.joints[j_w1].idx_q])


                q_cmd[i_w1_full] = float(np.clip(alpha, self._w1_lo, self._w1_hi))
                if not self._is_state_valid(q_cmd):
                    break

                q_points.append(q_cmd)
                qvars_seed = qvars  # warm-start next point
                last_qvars_solved = qvars 

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()
            if not q_points:
                continue

            # tiny deadband on the first point to avoid spam
            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    continue

            # remember seeds
            self._last_qvars = qvars_seed
            self._last_q_cmd = q_points[0]


            self._send_followtraj_traj(q_points, dt_step=DT_STEP)

            # light debug
            print(f"[batch] pts={len(q_points)} over {BATCH_HORIZON_S:.3f}s | W_now={np.round(W_smooth,3)}")

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
