#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from scipy.optimize import least_squares

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"     # IK target frame

# Joint group used by IK (the DOFs you want the solver to move)
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    # "rightarm_wrist_1_joint",  # enable if you want 4DOF IK
]

# Controller & joint order (must match your controller)
ACTION_NAME = "/right_arm_controller/follow_joint_trajectory"
JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    "rightarm_wrist_1_joint",
    "rightarm_wrist_2_joint",
    "rightarm_wrist_3_joint",
]

# Wearable → robot mapping
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow (m, joint-to-joint)
L2 = 0.23293990641364998   # elbow→wrist_2 (m)

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats = 100 bytes

# Loop & timing
CYCLE_SECONDS    = 0.05     # how often we compute IK and send a point
REACH_SECONDS    = 0.10     # each command's time_from_start

# IK solver params
W_POS = 1.0
W_REG = 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE = 0

# Optional tiny deadband to avoid noise-triggered micro moves (set to 0.0 to disable)
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

# --- Receding-horizon params ---
RH_ENABLE     = True
RH_N_STEPS    = 20         # number of waypoints per short trajectory
RH_HORIZON_S  = 0.30       # total time covered by the short trajectory
RH_MIN_DT     = 0.02       # lower bound on per-point spacing

# IK weighting
W_CONT = 5e-2              # continuity weight (keeps elbow/posture steady)
# (keep your existing W_POS and W_REG)

def residual_wrist_cont(q_vars, model, data, fid_wrist, idx_q_vars,
                        target_W, q_prev_vars,
                        w_pos=W_POS, w_cont=W_CONT, w_reg=W_REG):
    """
    Residual = [ wrist position error (3),
                 continuity penalty on (q - q_prev) (len(q)),
                 small L2 regularization on q (len(q)) ]
    """
    q = full_q_from_vars(model, idx_q_vars, q_vars)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation

    err_pos  = (pW - target_W) * w_pos
    err_cont = (q_vars - q_prev_vars) * w_cont
    err_reg  = q_vars * w_reg
    return np.hstack((err_pos, err_cont, err_reg))

# ================================================================

# Shared UDP state
_udp_lock = threading.Lock()
_hand_pos = None
_larm_pos = None
_uarm_pos = None
_hand_rot = None
_larm_rot = None
_uarm_rot = None 

def udp_listener(port=UDP_PORT):
    """Receive wearable packet and keep the latest sample."""
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
                _hand_rot = (hw, hx, hy, hz)
                _larm_rot = (lw, lx, ly, lz)
                _uarm_rot = (uw, ux, uy, uz)


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
    """
    Map watch 3 pts (upper, lower, hand) to robot-base elbow & wrist targets.
    We only use the wrist here; elbow is available if you need it later.
    """
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)

    # flip Y if your right-arm mapping needs it (kept from your previous code)
    Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

    u1 = unit(El - Sh)  # shoulder->elbow dir
    u2 = unit(Wr - El)  # elbow->wrist  dir

    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot        + L2 * u2
    return E_robot, W_robot

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
        residual_wrist_only, np.array(seed, dtype=float),
        bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err


def ik_step_with_cont(robot: RobotWrapper, fid_wrist: int, idx_q_vars, lb, ub,
                      target_W, seed_vars, q_prev_vars):
    """
    Solve IK for a single target_W with continuity toward q_prev_vars.
    """
    model = robot.model
    data  = model.createData()
    seed  = np.array(seed_vars,  dtype=float)
    qprev = np.array(q_prev_vars, dtype=float)

    res = least_squares(
        residual_wrist_cont, seed, bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, qprev, W_POS, W_CONT, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full); pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err

def make_horizon_targets(W_now):
    """
    Simple horizon: split the current wrist target into RH_N_STEPS
    identical waypoints (you can replace with prediction later).
    """
    if not RH_ENABLE or RH_N_STEPS <= 0:
        return [W_now], RH_HORIZON_S
    dt   = max(RH_HORIZON_S / RH_N_STEPS, RH_MIN_DT)
    return [np.array(W_now, float) for _ in range(RH_N_STEPS)], dt



# ============================ NODE ============================

class MinimalIKStreamer(Node):
    def __init__(self):
        super().__init__("minimal_ik_streamer")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # Joint state sub
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # FJT action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # Warm-start memory
        self._last_qvars = None
        self._last_q_cmd = None  # for tiny deadband

    # --- helpers ---
    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        # if not yet available, we’ll fall back to 0.0
        return [float(d.get(n, default)) for n in names]

    def _send_single_point(self, q_target, reach_s):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, q_target))
        sec = int(reach_s)
        nsec = int((reach_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [pt]
        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    # --- main loop ---
    def run(self):
        # give /joint_states a moment
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

            # get latest wearable sample
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                continue

            # 1) wearable → robot wrist target
            _, W = scale_watch_to_right_robot(up, lp, hp)

            # ---- Build short horizon ----
            W_list, dt_step = make_horizon_targets(W)

            # Current joints for filling non-IK entries
            js_now = self._current_positions(JOINTS, default=0.0)

            # ---- Build a non-None, non-zero IK seed ----
            if self._last_qvars is not None:
                seed_vars = self._last_qvars.copy()   # warm start from last good IK
            else:
                # Project the *measured* joint state onto the IK DOFs (better than zeros)
                js_vec = np.array(js_now, dtype=float)
                seed_vars = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)

            q_prev_var = seed_vars.copy()  # continuity reference starts at the seed

            # ---- Build short horizon (unchanged) ----
            W_list, dt_step = make_horizon_targets(W) # continuity reference starts at last solution

            q_points = []
            last_qvars_solved = None   # <-- add this

            for Wi in W_list:
                ok, qvars, qfull, pW, err = ik_step_with_cont(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub,
                    Wi, seed_vars, q_prev_var
                )
                if not ok:
                    break

                # Build full q_cmd in controller order
                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)

                q_points.append(q_cmd)

                # update seeds/continuity for next waypoint
                seed_vars  = qvars
                q_prev_var = qvars
                last_qvars_solved = qvars    # <-- remember newest IK vars

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()

            # Remember last vars for next cycle’s warm start
            if q_points:
                self._last_qvars = np.array([q_points[-1][JOINTS.index(jn)] for jn in IK_JOINTS], float)

            # Send multi-point short trajectory (preempts previous)
            if q_points:
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

            print(f"[tick] W_target={np.round(W,3)} | IK_err={err*1000:.1f} mm")

# ============================ main ============================

def main():
    # start UDP in background
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = MinimalIKStreamer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
