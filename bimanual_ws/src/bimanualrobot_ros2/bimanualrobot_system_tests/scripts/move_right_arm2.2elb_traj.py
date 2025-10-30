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
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

# ============================ CONFIG ============================


# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"    # IK target frame
LOG_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/bimanual_robot_log.txt"
# IK joints (include wrist_1 so wrist_2 position is actually controllable)
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
    # "rightarm_wrist_1_joint",
]
GROUP_NAME = "rightarm"


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
CYCLE_SECONDS       = 0.5   # how often we compute IK / (optionally) send
PER_MOVE_SECONDS    = 0.25    # big time first; we can crank down later
SEND_TO_CONTROLLER  = True  # start in "print-only" mode


MIN_W_STEP_M        = 0.030      # 1 cm: ignore smaller wrist target changes
MIN_JOINT_STEP_RAD  = np.deg2rad(1.0)   # 1°: ignore tiny joint diffs   

# IK solver params (from your working script)
W_POS      = 1.0
W_REG      = 1e-3
MAX_ITERS  = 300
XTOL       = 1e-8
FTOL       = 1e-8
GTOL       = 1e-8
VERBOSE    = 0

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
        except Exception as e:
            print(f"[UDP ERROR] {e}")

# ---------------- mapping & helpers (from your working code) ----------------
def remap_watch_to_base(p):
    """(x,y,z)_watch -> (z, -x, y)_base"""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_robot(uarm_watch, larm_watch, hand_watch):
    """
    Map watch points (upper, lower, hand) to robot-base Elbow/Wrist targets
    using fixed link lengths L1/L2 from the real shoulder joint center.
    """
    # remap to base frame
    Sh = remap_watch_to_base(uarm_watch)  # wearable "shoulder" marker (direction only)
    El = remap_watch_to_base(larm_watch)  # wearable elbow
    Wr = remap_watch_to_base(hand_watch)  # wearable wrist

    Sh[1] = -Sh[1]
    El[1] = -El[1]
    Wr[1] = -Wr[1]


    # unit directions along the two segments (watch frame geometry)
    u1 = unit(El - Sh)   # shoulder->elbow direction
    u2 = unit(Wr - El)   # elbow->wrist direction

    # anchor at the robot's shoulder joint center; step L1 then L2
    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot        + L2 * u2
    return E_robot, W_robot

# ---------------- Pinocchio IK (from your working least_squares) ------------
def build_index_maps(model: pin.Model, joint_names):
    """Return q-index array and bounds for the selected joints."""
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
    """Build a full configuration vector from the selected joint variables."""
    q = pin.neutral(model)
    for v, i in zip(q_vars, idx_q_vars):
        q[i] = float(v)
    return q

def residual_wrist_only(q_vars, model, data, fid_wrist, idx_q_vars, target_W, w_pos=W_POS, w_reg=W_REG):
    """Stacked residual: wrist position error (3) + small L2 regularization on q (DOF)."""
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

    # compute reached wrist
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)

    return res.success, q_vars_sol, q_full, pW, err, res.nfev

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
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")
        self._last_q_cmd = None

        self._last_W_cmd = None 


        # IK indexing & limits
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # Optional: remember last good IK to warm-start
        self._last_qvars = None

        # Joint states (to fill non-IK joints)
        self._latest_js = None

        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._log_stop = threading.Event()
        self._log_thread = threading.Thread(target=self._log_pan_loop, daemon=True)
        self._log_thread.start()
        # FJT action client
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

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
    
    def _limit_step(self, q_cmd,
                max_jump_rad=np.deg2rad(60.0),  # 120°
                fraction=0.5):
        """
        If any joint delta exceeds max_jump_rad, move only a fraction of that delta.
        Returns a possibly modified q_cmd (list).
        """
        if self._last_q_cmd is None:
            return q_cmd  # first time: nothing to limit

        q_prev = np.array(self._last_q_cmd, dtype=float)
        q_next = np.array(q_cmd,           dtype=float)
        deltas = q_next - q_prev

        max_abs = float(np.max(np.abs(deltas)))
        if max_abs > max_jump_rad:
            q_limited = q_prev + fraction * deltas
            self.get_logger().warn(
                f"Rate-limited step: max Δ={np.degrees(max_abs):.1f}° > "
                f"{np.degrees(max_jump_rad):.1f}° → applying {int(fraction*100)}% step this cycle."
            )
            return q_limited.tolist()

        return q_cmd

    def _log_pan_loop(self):
        """Logs shoulder pan joint angle (rad) at 5 Hz to LOG_PATH."""
            # t_sec = self.get_clock().now().nanoseconds * 1e-9
            # with open(LOG_PATH, "a") as f:
            #     f.write(f"{t_sec:.9f}\t{pan_rad:.9f}\n")
        pan_name = "rightarm_shoulder_pan_joint"
        while self._latest_js is None:
            a = 1
        while not self._log_stop.is_set():
            # read latest actual positions from /joint_states
            pan_rad = float(dict(zip(self._latest_js.name, self._latest_js.position)).get("rightarm_shoulder_pan_joint", 0.0))
            if pan_rad is not None:
                ts = time.time()
                pan = float(pan_rad)
                try:
                    with open(LOG_PATH, "a") as f:
                        f.write(f"{ts:.6f},{pan:.9f}\n")
                except Exception as e:
                    # keep silent in loop; optional: print once or throttle
                    pass
            time.sleep(0.2) 

    # ----- main loop -----
    def run_udp_follow(self):
        # Give /joint_states a moment (seed fallback)
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

            # fetch a consistent UDP sample
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                continue

            # 1) scale wearable -> robot targets (YOUR working scaling)
            _, W = scale_watch_to_robot(up, lp, hp)

            if self._last_W_cmd is not None:
                if np.linalg.norm(W - self._last_W_cmd) < MIN_W_STEP_M:
                    # too small to matter → skip whole cycle
                    # print("[SKIP] wrist delta below dead-band")
                    continue

            # 2) seed IK (prefer last solution; else zeros)
            seed = self._last_qvars if self._last_qvars is not None else np.zeros(len(self.idx_q_vars))

            # 3) solve wrist-only IK (YOUR working least_squares)
            startime = time.time()

            
            ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, W, seed=seed
            )
            time_end = time.time() - startime
            print(time_end)

            # pretty print IK every cycle
            print("\n===== IK cycle =====")
            print("Target W :", np.round(W, 6))
            print("Reached W:", np.round(pW, 6))
            print(f"Pos error: {err*1000:.3f} mm")
            for jn, v in zip(IK_JOINTS, qvars):
                print(f"  {jn:28s} {v:+.6f} rad ({np.degrees(v):+.3f} deg)")

            if not ok:
                print("[IK] did not converge this cycle; NOT sending.")
                continue

            self._last_qvars = qvars.tolist()

            # was made to check if the shoulder pan goes out of limits
            # pan_rad = float(dict(zip(self._latest_js.name, self._latest_js.position)).get("rightarm_shoulder_pan_joint", 0.0))
            # t_sec = self.get_clock().now().nanoseconds * 1e-9
            # with open(LOG_PATH, "a") as f:
            #     f.write(f"{t_sec:.9f}\t{pan_rad:.9f}\n")

            # 4) build full command (IK joints from solution, others from current)

            js_now = self._current_positions(JOINTS, default=0.0)
            q_cmd = list(js_now)
            for jn, val in zip(IK_JOINTS, qvars):
                if jn in JOINTS:
                    q_cmd[JOINTS.index(jn)] = float(val)

            print("Command (rad) in controller order:", [round(v, 6) for v in q_cmd])
            q_cmd = self._limit_step(q_cmd)

            if self._last_q_cmd is not None:
                dq_max = float(np.max(np.abs(np.array(q_cmd) - np.array(self._last_q_cmd))))
                if dq_max < MIN_JOINT_STEP_RAD:
                    # print(f"[SKIP] joint change {np.degrees(dq_max):.2f}° < dead-band")
                    continue
            if self._is_state_valid(q_cmd):
                self._send_followtraj_point(q_cmd, reach_s=PER_MOVE_SECONDS)
                self._last_q_cmd = q_cmd
                self._last_W_cmd = W.copy()
            else:
                self._last_q_cmd = q_cmd 
                self._last_W_cmd = W.copy()
                continue 

    # -------- FollowJointTrajectory send (unchanged) --------
    def _send_followtraj_point(self, q_cmd, reach_s=PER_MOVE_SECONDS):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, q_cmd))
        # pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [pt]
        # goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

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
        node._log_stop.set()
        if hasattr(node, "_log_thread"):
            node._log_thread.join(timeout=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
