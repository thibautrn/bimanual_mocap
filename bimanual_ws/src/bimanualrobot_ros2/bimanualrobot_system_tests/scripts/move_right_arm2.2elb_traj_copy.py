#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, os, json, datetime, math
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
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from collections import deque

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"    # IK target frame

# IK joints (include wrist_1 so wrist_2 position is actually controllable)
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    # "rightarm_wrist_1_joint",
]
GROUP_NAME = "rightarm"

# Receding-horizon (RHC) settings
RH_ENABLE       = True
RH_HORIZON_S    = 0.50      # total horizon (seconds)
RH_N_STEPS      = 10        # number of waypoints per send
RH_MIN_DT       = 0.06      # min step spacing (s)
VEL_EMA_ALPHA   = 0.7       # EMA for wrist velocity estimate (0…1)
HIST_MAX_S      = 0.6       # keep last ~0.6 s of wrist history

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

# Wearable → robot mapping (calibrated)
# Shoulder joint center (anchored)
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
# Link lengths (measured)
L1 = 0.298511306318538     # shoulder -> elbow (joint-to-joint)
L2 = 0.23293990641364998   # elbow -> wrist_2

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats = 100 bytes

# Cycle & timing
CYCLE_SECONDS       = 0.05   # loop tick for receiving + sending
PER_MOVE_SECONDS    = 0.10   # single-point fallback duration
SEND_TO_CONTROLLER  = True

# IK solver params
W_POS      = 1.0
W_REG      = 1e-3
MAX_ITERS  = 300
XTOL       = 1e-8
FTOL       = 1e-8
GTOL       = 1e-8
VERBOSE    = 0

# ============================ LOGGING ============================

LOG_DIR = os.path.expanduser("~/bimanual_logs")
os.makedirs(LOG_DIR, exist_ok=True)
_RUN = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

WATCH_LOG = os.path.join(LOG_DIR, f"watch_{_RUN}.jsonl")
IK_LOG    = os.path.join(LOG_DIR, f"ik_{_RUN}.jsonl")
QCMD_LOG  = os.path.join(LOG_DIR, f"qcmd_{_RUN}.jsonl")

# ================================================================

# UDP latest sample (protected by a lock)
_udp_lock = threading.Lock()
_hand_pos = None
_larm_pos = None
_uarm_pos = None

def udp_listener(port=UDP_PORT):
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

            # Log raw watch positions (+ quats if you want)
            try:
                rec = {
                    "t": time.time(),
                    "kind": "watch_sample",
                    "hand_pos_watch": [hpx, hpy, hpz],
                    "larm_pos_watch": [lpx, lpy, lpz],
                    "uarm_pos_watch": [upx, upy, upz],
                    "hand_q": [hw, hx, hy, hz],
                    "larm_q": [lw, lx, ly, lz],
                    "uarm_q": [uw, ux, uy, uz],
                }
                with open(WATCH_LOG, "a") as f:
                    f.write(json.dumps(rec) + "\n")
            except Exception:
                pass
        except Exception as e:
            print(f"[UDP ERROR] {e}")

# ---------------- mapping & helpers ----------------

def remap_watch_to_base(p):
    """(x,y,z)_watch -> (z, -x, y)_base"""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_right_robot(uarm_watch, larm_watch, hand_watch):
    """
    Map watch points (upper, lower, hand) to robot-base Elbow/Wrist targets
    using fixed link lengths L1/L2 from the real shoulder joint center.
    """
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)

    # mirror Y for right side
    Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

    u1 = unit(El - Sh)   # shoulder->elbow dir
    u2 = unit(Wr - El)   # elbow->wrist dir

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
    lb = lb_all[idx_q_vars].copy()
    ub = ub_all[idx_q_vars].copy()
    lb[~np.isfinite(lb)] = -1e9
    ub[~np.isfinite(ub)] = +1e9
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
                                 seed=None):
    model = robot.model
    data  = model.createData()
    seed = np.zeros(len(idx_q_vars), dtype=float) if seed is None else np.array(seed, dtype=float)

    res = least_squares(
        residual_wrist_only,
        seed,
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

    return res.success, q_vars_sol, q_full, pW, err, res.nfev

# ============================ SMOOTHING (optional) ============================

class LowPassEMA:
    def __init__(self, dim, fc_hz=3.0):
        self.dim = dim
        self.fc = fc_hz
        self.y = None

    def _alpha(self, dt):
        return 1.0 - math.exp(-2.0*math.pi*self.fc*max(dt, 1e-3))

    def reset(self): self.y = None

    def update(self, x, dt):
        x = np.asarray(x, dtype=float)
        if self.y is None:
            self.y = x.copy()
            return self.y
        a = self._alpha(dt)
        self.y = (1.0 - a) * self.y + a * x
        return self.y

# ============================ NODE ============================

class PinIKUDPToFJT(Node):
    def __init__(self):
        super().__init__("pinik_print_and_optionally_send")

        # --- Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        self.data: pin.Data = self.model.createData()
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Pinocchio frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # MoveIt validity check client
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # IK indexing & limits
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # Last good solutions / commands
        self._last_qvars = None
        self._last_q_cmd = None

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Action client
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # RHC helpers
        self._w_hist = deque()    # (timestamp, wrist_pos)
        self._w_vel  = np.zeros(3)
        self._last_tick_ts = None

        # Optional joint command low-pass
        self._q_lpf = LowPassEMA(dim=len(JOINTS), fc_hz=4.0)

    # ----- small logger
    def _log_event(self, path, kind, payload):
        try:
            rec = {"t": time.time(), "kind": kind}
            rec.update(payload)
            with open(path, "a") as f:
                f.write(json.dumps(rec) + "\n")
        except Exception:
            pass

    # ----- ROS helpers -----
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

    def _limit_step(self, q_cmd,
                    max_jump_rad=np.deg2rad(60.0),
                    fraction=0.5):
        if self._last_q_cmd is None:
            return q_cmd
        q_prev = np.array(self._last_q_cmd, dtype=float)
        q_next = np.array(q_cmd,           dtype=float)
        deltas = q_next - q_prev
        max_abs = float(np.max(np.abs(deltas)))
        if max_abs > max_jump_rad:
            q_limited = q_prev + fraction * deltas
            self.get_logger().warn(
                f"Rate-limited step: max Δ={np.degrees(max_abs):.1f}° > "
                f"{np.degrees(max_jump_rad):.1f}° → applying {int(fraction*100)}% step."
            )
            return q_limited.tolist()
        return q_cmd

    # ----- RHC helpers -----
    def _rh_update_history(self, W_now, t_now):
        self._w_hist.append((t_now, W_now.copy()))
        while self._w_hist and (t_now - self._w_hist[0][0]) > HIST_MAX_S:
            self._w_hist.popleft()
        if len(self._w_hist) >= 2:
            (t0, W0), (t1, W1) = self._w_hist[-2], self._w_hist[-1]
            dt = max(t1 - t0, RH_MIN_DT)
            v_inst = (W1 - W0) / dt
            self._w_vel = (1.0 - VEL_EMA_ALPHA) * self._w_vel + VEL_EMA_ALPHA * v_inst

    def _rh_predict_wrist_traj(self, W_now):
        if not RH_ENABLE or RH_N_STEPS <= 0:
            return [W_now]
        dt_step = max(RH_HORIZON_S / RH_N_STEPS, RH_MIN_DT)
        preds = []
        for k in range(1, RH_N_STEPS + 1):
            Wk = W_now + self._w_vel * (k * dt_step)
            preds.append(Wk)
        return preds

    def _rh_solve_horizon(self, W_list, seed_qvars, js_now):
        if not W_list:
            return [], 0.0
        dt_step = max(RH_HORIZON_S / max(len(W_list), 1), RH_MIN_DT)
        qvars_seed = np.array(seed_qvars if seed_qvars is not None else np.zeros(len(self.idx_q_vars)))

        q_points = []
        for Wi in W_list:
            ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, Wi, seed=qvars_seed
            )

            # Log IK for each predicted waypoint
            self._log_event(
                IK_LOG, "ik",
                {
                    "target_W": np.asarray(Wi).tolist(),
                    "reached_pW": np.asarray(pW).tolist(),
                    "pos_err_m": float(err),
                    "iters": int(iters),
                    "ok": bool(ok),
                    "ik_joints": IK_JOINTS,
                    "qvars_rad": np.asarray(qvars).tolist(),
                }
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

        return q_points, dt_step

    # ----- main loop -----
    def run_udp_follow(self):
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        last_sent = 0.0
        while rclpy.ok():
            now = time.time()
            if now - last_sent < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.02)
                continue
            last_sent = now

            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                continue

            # 1) wearable → robot target
            _, W = scale_watch_to_right_robot(up, lp, hp)

            dt = (now - self._last_tick_ts) if self._last_tick_ts is not None else CYCLE_SECONDS
            self._last_tick_ts = now

            # 2) RHC prediction from velocity estimate
            self._rh_update_history(W, now)
            W_list = self._rh_predict_wrist_traj(W) if RH_ENABLE else [W]

            # 3) seed IK, current js for non-IK joints
            seed   = self._last_qvars if self._last_qvars is not None else np.zeros(len(self.idx_q_vars))
            js_now = self._current_positions(JOINTS, default=0.0)

            if RH_ENABLE:
                # ---- sequential IK along the short horizon
                q_points, dt_step = self._rh_solve_horizon(W_list, seed, js_now)

                if q_points:
                    first_q = q_points[0]
                    self._last_qvars = [first_q[JOINTS.index(jn)] for jn in IK_JOINTS]

                    # Optional: rate limit the first point
                    q_points[0] = self._limit_step(q_points[0])

                    # Optional: LPF on *first* imminent point
                    # self._q_lpf.update(q_points[0], dt)

                    if SEND_TO_CONTROLLER:
                        self._send_followtraj_traj(q_points, dt_step)

                    # (brief console peek)
                    print(f"[RHC] W_now {np.round(W, 4)}  pts={len(q_points)}  dt={dt_step:.3f}s")
                else:
                    print("[RHC] No valid points; skipped.")
                    continue

            else:
                # ---- single-step fallback
                ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, W, seed=seed
                )
                self._log_event(
                    IK_LOG, "ik",
                    {
                        "target_W": np.asarray(W).tolist(),
                        "reached_pW": np.asarray(pW).tolist(),
                        "pos_err_m": float(err),
                        "iters": int(iters),
                        "ok": bool(ok),
                        "ik_joints": IK_JOINTS,
                        "qvars_rad": np.asarray(qvars).tolist(),
                    }
                )
                if not ok:
                    print("[IK] did not converge; skip.")
                    continue

                self._last_qvars = qvars.tolist()

                js_now = self._current_positions(JOINTS, default=0.0)
                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)

                q_cmd = self._limit_step(q_cmd)
                q_cmd = self._q_lpf.update(q_cmd, dt)

                if self._is_state_valid(q_cmd) and SEND_TO_CONTROLLER:
                    self._send_followtraj_point(q_cmd, reach_s=PER_MOVE_SECONDS)

    # -------- FollowJointTrajectory senders --------
    def _send_followtraj_traj(self, q_points, dt_step):
        """
        Send a short multi-point trajectory. New goals preempt the active one (RHC behavior).
        """
        if not q_points:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        t_accum = 0.0
        pts = []
        traj_pts_log = []
        for q in q_points:
            t_accum += dt_step
            pt = JointTrajectoryPoint()
            pt.positions = list(map(float, q))
            sec = int(t_accum)
            nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)

            traj_pts_log.append({
                "time_from_start_s": float(t_accum),
                "positions_rad": list(map(float, q))
            })

        goal.trajectory.points = pts
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        # Log the trajectory we are about to send
        self._log_event(
            QCMD_LOG, "traj_multi",
            {"joints": JOINTS, "points": traj_pts_log}
        )

        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    def _send_followtraj_point(self, q_cmd, reach_s=PER_MOVE_SECONDS):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, q_cmd))
        sec = int(reach_s)
        nsec = int((reach_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [pt]
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        # Log single point command
        self._log_event(
            QCMD_LOG, "traj_single",
            {"joints": JOINTS, "time_from_start_s": float(reach_s), "positions_rad": list(map(float, q_cmd))}
        )

        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

# ============================ main ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = PinIKUDPToFJT()
    try:
        node.run_udp_follow()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
