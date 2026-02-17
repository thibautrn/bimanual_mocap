#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, math
from datetime import datetime
from pathlib import Path

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

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TKINTER = True
except ImportError:
    HAS_TKINTER = False

# ============================ CONFIG ============================

URDF_PATH = "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"
LOG_DIR   = Path("/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/joints")

IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
]

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
SHOULDER_ANCHOR  = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538
L2 = 0.23293990641364998

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"

# Timing
CYCLE_SECONDS   = 0.06
UPSAMPLE_FACTOR = 2

# Smoothing
LPF_CUTOFF_HZ      = 3.5
SPIKE_MAX_SPEED    = 2.0
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

# IK params
W_POS     = 1.0
W_REG     = 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE   = 0

# ============================ STARTUP / END POSITIONS ============================

STARTUP_JOINT_POSITIONS = {
    "rightarm_shoulder_pan_joint":  1.57,
    "rightarm_shoulder_lift_joint": 1.01,
    "rightarm_elbow_joint":        -0.113,
    "rightarm_wrist_1_joint":       0.0,
    "rightarm_wrist_2_joint":       0.0,
    "rightarm_wrist_3_joint":       0.0,
}

END_JOINT_POSITIONS = {
    "rightarm_shoulder_pan_joint":  1.6,
    "rightarm_shoulder_lift_joint": 0.85,
    "rightarm_elbow_joint":         0.0,
    "rightarm_wrist_1_joint":       0.0,
    "rightarm_wrist_2_joint":       0.0,
    "rightarm_wrist_3_joint":       0.0,
}

STARTUP_MOVE_TIME      = 3.0   # seconds to reach startup position
END_POSITION_MOVE_TIME = 2.0   # seconds to reach end position


# ============================ UDP LISTENER ============================

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
            (hw, hx, hy, hz, hpx, hpy, hpz,
             lw, lx, ly, lz, lpx, lpy, lpz,
             uw, ux, uy, uz, upx, upy, upz,
             qw, qx, qy, qz) = unpack(data)
            with _udp_lock:
                _hand_pos = (hpx, hpy, hpz)
                _larm_pos = (lpx, lpy, lpz)
                _uarm_pos = (upx, upy, upz)
        except Exception as e:
            print(f"[UDP ERROR] {e}")


# ============================ MAPPING HELPERS ============================

def remap_watch_to_base(p):
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_right_robot(uarm_watch, larm_watch, hand_watch):
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)
    Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]
    u1 = unit(El - Sh)
    u2 = unit(Wr - El)
    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot + L2 * u2
    return E_robot, W_robot


# ============================ LOW-PASS FILTER ============================

class LowPassEMA:
    def __init__(self, fc_hz=LPF_CUTOFF_HZ):
        self.fc     = float(fc_hz)
        self.y      = None
        self.t_last = None

    def update(self, x, t_now):
        x = np.asarray(x, float)
        if self.y is None or self.t_last is None:
            self.y = x.copy(); self.t_last = float(t_now); return self.y
        dt    = max(float(t_now - self.t_last), 1e-3)
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.fc * dt)
        self.y = (1.0 - alpha) * self.y + alpha * x
        self.t_last = float(t_now)
        return self.y


# ============================ PINOCCHIO HELPERS ============================

def build_index_maps(model: pin.Model, joint_names):
    idx_q_vars = []
    for jn in joint_names:
        jid = model.getJointId(jn)
        if jid == 0:
            raise RuntimeError(f"Joint not found: {jn}")
        idx_q_vars.append(model.joints[jid].idx_q)
    lb_all = np.array(model.lowerPositionLimit, dtype=float)
    ub_all = np.array(model.upperPositionLimit, dtype=float)
    lb = lb_all[idx_q_vars].copy(); ub = ub_all[idx_q_vars].copy()
    lb[~np.isfinite(lb)] = -1e9;    ub[~np.isfinite(ub)] = +1e9
    return np.array(idx_q_vars), lb, ub

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
    return np.hstack(((pW - target_W) * w_pos, w_reg * q_vars))

def solve_wrist_ik_least_squares(robot: RobotWrapper, fid_wrist: int,
                                 idx_q_vars, lb, ub, target_W, seed):
    model = robot.model; data = model.createData()
    res = least_squares(
        residual_wrist_only, np.array(seed, float),
        bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_full = full_q_from_vars(model, idx_q_vars, res.x)
    pin.forwardKinematics(model, data, q_full)
    pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    return res.success, res.x, q_full, pW, np.linalg.norm(pW - target_W)


# ============================ NODE ============================

class JointAngleLogger(Node):
    """Teleop that moves to startup, records joint angles, then moves to end position on stop."""

    def __init__(self):
        super().__init__("joint_angle_logger")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model    = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._traj_ac.wait_for_server()

        # Smoothing (kept — this is on raw Cartesian wearable input, not DMP)
        self._w_lpf    = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W   = None
        self._prev_W_t = None

        # IK warm start
        self._last_qvars = None
        self._last_q_cmd = None

        # Logging state
        self._logging = False
        self._log_fh  = None
        self._udp_enabled = False
        LOG_DIR.mkdir(parents=True, exist_ok=True)

        # End position movement state
        self._moving_to_end       = False
        self._end_move_start_time = None
        self._end_move_start_pos  = None  # joint positions when stop was pressed

        # Commands (set from GUI thread, consumed in run loop)
        self._cmd_start_log = False
        self._cmd_stop_log  = False

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        return [float(d.get(n, default)) for n in names]

    # ── Trajectory sending ──────────────────────────────────────────────────

    def _send_followtraj_traj(self, q_points, total_dt):
        """Send a list of joint angle waypoints as a single trajectory."""
        if not q_points:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        dt_step = float(total_dt) / float(len(q_points))
        t_accum = 0.0
        pts     = []
        for q in q_points:
            t_accum += dt_step
            pt = JointTrajectoryPoint()
            pt.positions = list(map(float, q))
            sec  = int(t_accum); nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)
        goal.trajectory.points = pts
        self._traj_ac.send_goal_async(goal)

    def _send_position(self, joint_positions: dict, move_time: float):
        """Send a single target position for all joints."""
        q = [float(joint_positions.get(jn, 0.0)) for jn in JOINTS]
        self._send_followtraj_traj([q], total_dt=move_time)

    # ── Startup move ────────────────────────────────────────────────────────

    def _move_to_startup(self):
        """Move robot to startup position and wait for it to arrive."""
        print(f"\n[STARTUP] Moving to startup position over {STARTUP_MOVE_TIME}s...")
        for jn, pos in STARTUP_JOINT_POSITIONS.items():
            print(f"  {jn}: {np.degrees(pos):.1f}°")
        self._send_position(STARTUP_JOINT_POSITIONS, move_time=STARTUP_MOVE_TIME)
        # Spin while waiting so ROS callbacks keep running
        t0 = time.time()
        while time.time() - t0 < STARTUP_MOVE_TIME + 0.5:
            rclpy.spin_once(self, timeout_sec=0.05)
        print("[STARTUP] ✓ Startup position reached\n")

    # ── Logging commands ────────────────────────────────────────────────────

    def request_start_logging(self):
        self._cmd_start_log = True
        print("[Command] Start logging requested")

    def request_stop_logging(self):
        self._cmd_stop_log = True
        print("[Command] Stop logging requested")

    def _do_start_logging(self):
        if self._logging:
            print("[Logging] Already logging!")
            return False

        ts         = datetime.now().strftime("%Y%m%d_%H%M%S")
        joints_path = LOG_DIR / f"joints_{ts}.txt"

        try:
            self._log_fh = open(joints_path, "w", buffering=1)
            self._log_fh.write("# t_sec  q1(shoulder_pan)  q2(shoulder_lift)  q3(elbow)   (joint angles in radians)\n")
            self._logging     = True
            self._udp_enabled = True
            print(f"\n[SUCCESS] Logging started: {joints_path}\n")
            return True
        except Exception as e:
            if self._log_fh:
                self._log_fh.close()
            self._log_fh = None
            print(f"[ERROR] Failed to start logging: {e}")
            return False

    def _do_stop_logging(self):
        if not self._logging:
            print("[Logging] Not currently logging")
            return False

        print(f"\n[STOP] Recording end position movement...")

        # Disable teleop immediately
        self._udp_enabled = False

        # Capture current joint positions as interpolation start
        js = dict(zip(self._latest_js.name, self._latest_js.position)) if self._latest_js else {}
        self._end_move_start_pos  = {jn: float(js.get(jn, 0.0)) for jn in JOINTS}
        self._end_move_start_time = time.time()
        self._moving_to_end       = True

        print(f"  Moving to end position over {END_POSITION_MOVE_TIME}s...")
        for jn, pos in END_JOINT_POSITIONS.items():
            print(f"  {jn}: {np.degrees(pos):.1f}°")

        return True

    def _update_end_movement(self, now):
        """
        Cosine-interpolate to end position while still logging.
        Returns True when movement is complete and files are closed.
        """
        elapsed    = now - self._end_move_start_time
        total_time = END_POSITION_MOVE_TIME + 0.5  # move + hold

        if elapsed < total_time:
            t        = min(elapsed / END_POSITION_MOVE_TIME, 1.0)
            t_smooth = 0.5 - 0.5 * np.cos(t * np.pi)

            q_cmd = []
            for jn in JOINTS:
                start = self._end_move_start_pos.get(jn, 0.0)
                end   = END_JOINT_POSITIONS.get(jn, start)
                q_cmd.append(float(start + t_smooth * (end - start)))

            self._send_followtraj_traj([q_cmd], total_dt=CYCLE_SECONDS)

            # Keep logging during the move
            self._log_current_joints(now)
            return False

        # Movement done — close log file
        path = None
        if self._log_fh:
            path = self._log_fh.name
            self._log_fh.close()
            self._log_fh = None

        self._logging       = False
        self._moving_to_end = False

        print(f"\n[DONE] End position reached. Log saved:")
        if path:
            print(f"  {path}")
        print("=" * 60 + "\n")
        return True

    # ── Logging helpers ─────────────────────────────────────────────────────

    def log_joint_angles(self, timestamp, q_vars):
        """Log the 3 IK joint angles."""
        if self._logging and self._log_fh:
            q1, q2, q3 = map(float, q_vars)
            self._log_fh.write(f"{timestamp:.6f} {q1:.6f} {q2:.6f} {q3:.6f}\n")

    def _log_current_joints(self, now):
        """Log current joint angles from /joint_states (used during end movement)."""
        if not self._logging or not self._log_fh or self._latest_js is None:
            return
        js = dict(zip(self._latest_js.name, self._latest_js.position))
        q1 = float(js.get(IK_JOINTS[0], 0.0))
        q2 = float(js.get(IK_JOINTS[1], 0.0))
        q3 = float(js.get(IK_JOINTS[2], 0.0))
        self._log_fh.write(f"{now:.6f} {q1:.6f} {q2:.6f} {q3:.6f}\n")

    # ── Command processing ──────────────────────────────────────────────────

    def process_commands(self):
        if self._cmd_start_log:
            self._cmd_start_log = False
            self._do_start_logging()
        if self._cmd_stop_log:
            self._cmd_stop_log = False
            self._do_stop_logging()

    # ── Main loop ───────────────────────────────────────────────────────────

    def run(self):
        # Move to startup before doing anything else
        self._move_to_startup()

        last_tick = time.monotonic()

        while rclpy.ok():
            now = time.monotonic()

            # Process GUI commands
            self.process_commands()

            # Handle end position movement (logs + interpolates)
            if self._moving_to_end:
                self._update_end_movement(now)
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            # Rate gate
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            # Skip teleop if UDP not enabled
            if not self._udp_enabled:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            # Get wearable data
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                continue

            _, W_raw = scale_watch_to_right_robot(up, lp, hp)
            W = self._w_lpf.update(W_raw, now)

            # Spike guard
            if self._prev_W is not None and self._prev_W_t is not None:
                dt = max(now - self._prev_W_t, 1e-3)
                if np.linalg.norm(W - self._prev_W) / dt > SPIKE_MAX_SPEED:
                    continue

            # Upsample
            if self._prev_W is not None and UPSAMPLE_FACTOR >= 2:
                W_list = [0.5 * (self._prev_W + W), W]
            else:
                W_list = [W]

            js_now = self._current_positions(JOINTS, default=0.0)

            # IK seed
            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()
            else:
                js_vec     = np.array(js_now, dtype=float)
                qvars_seed = np.array(
                    [js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float
                )

            # IK solve
            q_points          = []
            last_qvars_solved = None
            for Wk in W_list:
                ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist,
                    self.idx_q_vars, self.lb, self.ub,
                    Wk, seed=qvars_seed
                )
                if not ok:
                    break
                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)
                q_points.append(q_cmd)
                qvars_seed        = qvars
                last_qvars_solved = qvars

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()

            if not q_points:
                self._prev_W = W.copy(); self._prev_W_t = now
                continue

            # Deadband
            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    self._prev_W = W.copy(); self._prev_W_t = now
                    continue

            # Log joint angles (3 IK joints)
            if last_qvars_solved is not None:
                self.log_joint_angles(now, last_qvars_solved)

            # Send trajectory
            self._last_q_cmd = q_points[0]
            self._prev_W     = W.copy()
            self._prev_W_t   = now
            self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

            print(f"[tick] pts={len(q_points)} | W={np.round(W, 3)}")


# ============================ CONTROL GUI ============================

def control_gui(node):
    if not HAS_TKINTER:
        print("[GUI] Tkinter not available")
        return

    root = tk.Tk()
    root.title("Joint Angle Logger Control")
    root.geometry("300x200")

    status_var = tk.StringVar(value="Moving to startup...")

    def start_log():
        status_var.set("LOGGING ACTIVE")
        start_btn.config(state='disabled')
        stop_btn.config(state='normal')
        node.request_start_logging()

    def stop_log():
        status_var.set("Moving to end position...")
        stop_btn.config(state='disabled')
        node.request_stop_logging()

        # Re-enable start button once movement is done
        def wait_for_done():
            while node._moving_to_end or node._logging:
                time.sleep(0.1)
            root.after(0, lambda: [
                status_var.set("Ready"),
                start_btn.config(state='normal')
            ])
        threading.Thread(target=wait_for_done, daemon=True).start()

    status_label = ttk.Label(root, textvariable=status_var, font=('Arial', 12, 'bold'))
    status_label.pack(pady=20)

    start_btn = ttk.Button(root, text="START LOGGING", command=start_log, width=20, state='disabled')
    start_btn.pack(pady=5)

    stop_btn = ttk.Button(root, text="STOP LOGGING", command=stop_log, width=20, state='disabled')
    stop_btn.pack(pady=5)

    info_label = ttk.Label(root, text=f"Logs saved to {LOG_DIR}", font=('Arial', 9))
    info_label.pack(pady=10)

    # Enable start button once startup move is done
    def wait_for_startup():
        # Startup move takes STARTUP_MOVE_TIME + 0.5s
        time.sleep(STARTUP_MOVE_TIME + 1.0)
        root.after(0, lambda: [
            status_var.set("Ready"),
            start_btn.config(state='normal')
        ])
    threading.Thread(target=wait_for_startup, daemon=True).start()

    root.mainloop()


# ============================ MAIN ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = JointAngleLogger()

    if HAS_TKINTER:
        print("[GUI] Starting control window...")
        gui_thread = threading.Thread(target=lambda: control_gui(node), daemon=True)
        gui_thread.start()
        time.sleep(0.5)
        print("\n" + "=" * 60)
        print("CONTROLS:")
        print("=" * 60)
        print("  1. Robot moves to startup position automatically")
        print("  2. Press START LOGGING to begin recording")
        print("  3. Press STOP LOGGING — robot moves to end position")
        print("     while still recording, then saves file")
        print(f"\n  UDP: Listening on port {UDP_PORT}")
        print(f"  Logs: {LOG_DIR}/joints_YYYYMMDD_HHMMSS.txt")
        print(f"  Format: t_sec q1 q2 q3 (radians)")
        print("=" * 60 + "\n")

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()