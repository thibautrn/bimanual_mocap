#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time, struct, socket, threading, math, traceback
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

# ============================ CONFIG ============================

URDF_PATH = "/home/asurite.ad.asu.edu/troisin/Documents/robot/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"
LOG_DIR   = Path("/home/asurite.ad.asu.edu/troisin/Documents/robot/mujoco_bimanual/logs/robot_logs")

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

# Wearable → robot mapping
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
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
    "rightarm_shoulder_pan_joint":  1.80,
    "rightarm_shoulder_lift_joint": 0.85,
    "rightarm_elbow_joint":         0.0,
    "rightarm_wrist_1_joint":       0.0,
    "rightarm_wrist_2_joint":       0.0,
    "rightarm_wrist_3_joint":       0.0,
}

STARTUP_MOVE_TIME      = 2.0
END_POSITION_MOVE_TIME = 1.0


# ============================ KEYBOARD LISTENER ============================

def keyboard_listener(node):
    time.sleep(STARTUP_MOVE_TIME + 1.5)

    print("\n" + "="*60)
    print("  Robot ready!")
    print("  Press ENTER to START recording")
    print("="*60 + "\n")

    while True:
        try:
            input()
        except EOFError:
            time.sleep(0.1)
            continue

        if not node._udp_enabled and not node._moving_to_end and not node._should_exit:
            print("▶ START — recording. Press ENTER to stop.")
            node.request_start_logging()  # ← open file + log first frame + enable UDP
        elif node._udp_enabled and not node._moving_to_end:
            print("⏹ STOP")
            node.request_stop_logging()


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

    def __init__(self):
        super().__init__("joint_angle_logger")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model    = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._traj_ac.wait_for_server()

        self._w_lpf    = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W   = None
        self._prev_W_t = None

        self._last_qvars = None
        self._last_q_cmd = None

        # Logging state
        self._logging  = False
        self._log_fh   = None
        LOG_DIR.mkdir(parents=True, exist_ok=True)

        # Control flags
        self._udp_enabled         = False
        self._moving_to_end       = False
        self._end_move_start_time = None
        self._should_exit         = False
        self._cmd_stop_log        = False

    # ── ROS callbacks ──────────────────────────────────────────────────────

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _current_positions(self, names, default=0.0):
        d = {}
        if self._latest_js:
            d = dict(zip(self._latest_js.name, self._latest_js.position))
        return [float(d.get(n, default)) for n in names]

    # ── Trajectory sending ─────────────────────────────────────────────────

    def _send_followtraj_traj(self, q_points, total_dt):
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
        q = [float(joint_positions.get(jn, 0.0)) for jn in JOINTS]
        self._send_followtraj_traj([q], total_dt=move_time)

    # ── Logging ────────────────────────────────────────────────────────────

    def _open_log_file(self):
        ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
        joints_path = LOG_DIR / f"joints_{ts}.txt"
        self._log_fh = open(joints_path, "w", buffering=1)
        self._log_fh.write("# t_sec  q1(shoulder_pan)  q2(shoulder_lift)  q3(elbow)  (radians)\n")
        self._logging = True
        print(f"\n[LOGGING] Started → {joints_path}\n")

    def _log_current_joints(self, now):
        """Log from /joint_states — used during startup and end movement."""
        if not self._logging or not self._log_fh or self._latest_js is None:
            return
        js = dict(zip(self._latest_js.name, self._latest_js.position))
        q1 = float(js.get(IK_JOINTS[0], 0.0))
        q2 = float(js.get(IK_JOINTS[1], 0.0))
        q3 = float(js.get(IK_JOINTS[2], 0.0))
        self._log_fh.write(f"{now:.6f} {q1:.6f} {q2:.6f} {q3:.6f}\n")

    def _log_ik_joints(self, timestamp, q_vars):
        """Log IK-commanded joints — used during UDP teleop."""
        if not self._logging or not self._log_fh:
            return
        q1, q2, q3 = map(float, q_vars)
        self._log_fh.write(f"{timestamp:.6f} {q1:.6f} {q2:.6f} {q3:.6f}\n")

    def _close_log_file(self):
        if self._log_fh:
            path = self._log_fh.name
            self._log_fh.close()
            self._log_fh = None
            self._logging = False
            print(f"\n[DONE] Log saved: {path}\n")

    # ── Startup ────────────────────────────────────────────────────────────

    def _wait_for_joint_states(self, timeout=5.0):
        """Spin until first /joint_states message arrives."""
        print("[STARTUP] Waiting for /joint_states...")
        t0 = time.time()
        while self._latest_js is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                print("[WARNING] Timeout waiting for /joint_states — proceeding anyway")
                return
        print("[STARTUP] ✓ /joint_states received")

    def _move_to_startup(self):
        print(f"\n[STARTUP] Moving to startup position over {STARTUP_MOVE_TIME}s...")

        # Wait for joint states before opening log
        self._wait_for_joint_states()


        # Send startup trajectory
        self._send_position(STARTUP_JOINT_POSITIONS, move_time=STARTUP_MOVE_TIME)

        # Spin and log during startup movement
        t0 = time.time()
        while time.time() - t0 < STARTUP_MOVE_TIME + 0.5:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._log_current_joints(time.monotonic())

        print("[STARTUP] ✓ Startup position reached\n")

    # ── Stop command ───────────────────────────────────────────────────────

    def request_stop_logging(self):
        self._cmd_stop_log = True

    def request_start_logging(self):
        self._open_log_file()
        # Log the current position as the very first frame
        self._log_current_joints(time.monotonic())
        print("[LOGGING] First position recorded, UDP teleop active\n")
        self._udp_enabled = True

    def _do_stop(self):
        """Disable UDP, send end position, start end movement phase."""
        self._udp_enabled = False
        print(f"\n[STOP] Moving to end position...")
        self._send_position(END_JOINT_POSITIONS, move_time=END_POSITION_MOVE_TIME)
        self._moving_to_end       = True
        self._end_move_start_time = time.monotonic()

    def _check_end_movement_done(self, now):
        # Keep logging during end movement
        self._log_current_joints(now)

        elapsed = now - self._end_move_start_time
        if elapsed < END_POSITION_MOVE_TIME + 0.5:
            return False

        self._close_log_file()
        self._moving_to_end = False
        self._should_exit   = True
        return True

    # ── Main loop ──────────────────────────────────────────────────────────

    def run(self):
        try:
            self._move_to_startup()
        except Exception as e:
            print(f"[ERROR] _move_to_startup failed: {e}")
            traceback.print_exc()
            return

        last_tick = time.monotonic()

        try:
            while rclpy.ok():
                now = time.monotonic()

                if self._should_exit:
                    print("[EXIT] Shutting down")
                    break

                # Process stop command
                if self._cmd_stop_log:
                    self._cmd_stop_log = False
                    self._do_stop()

                # End movement phase — just log and wait
                if self._moving_to_end:
                    done = self._check_end_movement_done(now)
                    rclpy.spin_once(self, timeout_sec=0.02)
                    continue

                # Rate gate
                if now - last_tick < CYCLE_SECONDS:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue
                last_tick = now

                # Waiting for ENTER — log current joints at rest
                if not self._udp_enabled:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

                # ── UDP TELEOP ─────────────────────────────────────────────
                with _udp_lock:
                    hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
                if hp is None or lp is None or up is None:
                    rclpy.spin_once(self, timeout_sec=0.01)
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

                # Log IK joints
                if last_qvars_solved is not None:
                    self._log_ik_joints(now, last_qvars_solved)

                self._last_q_cmd = q_points[0]
                self._prev_W     = W.copy()
                self._prev_W_t   = now
                self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

                print(f"[tick] pts={len(q_points)} | W={np.round(W, 3)}")

        except Exception as e:
            print(f"[ERROR] in run loop: {e}")
            traceback.print_exc()


# ============================ MAIN ============================

def main():
    # Start UDP listener
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = JointAngleLogger()

    print("\n" + "=" * 60)
    print("CONTROLS:")
    print("=" * 60)
    print("  1. Robot moves to startup position (logging starts automatically)")
    print("  2. Press ENTER to START UDP teleop")
    print("  3. Press ENTER again to STOP — robot moves to end position,")
    print("     keeps logging, then saves the file")
    print(f"\n  UDP: Listening on port {UDP_PORT}")
    print(f"  Logs: {LOG_DIR}/joints_YYYYMMDD_HHMMSS.txt")
    print(f"  Format: t_sec q1 q2 q3 (radians)")
    print("=" * 60 + "\n")

    # Start keyboard listener
    kb_thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    kb_thread.start()

    try:
        node.run()
    finally:
        print("[CLEANUP] Shutting down...")
        if node._log_fh:
            node._log_fh.close()
        node.destroy_node()
        rclpy.shutdown()
        time.sleep(0.2)
        import os
        os._exit(0)


if __name__ == "__main__":
    main()