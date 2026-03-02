#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
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

from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

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

URDF_PATH = "/home/asurite.ad.asu.edu/troisin/Documents/robot/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"

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

class SimpleTeleop(Node):
    """Simple teleop - no logging, no start/end positions"""

    def __init__(self):
        super().__init__("simple_teleop")

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

        # MoveIt validity
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._traj_ac.wait_for_server()

        # Smoothing
        self._w_lpf    = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W   = None
        self._prev_W_t = None

        # IK warm start
        self._last_qvars = None
        self._last_q_cmd = None

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

        self.get_logger().warn(
            f"State INVALID: contacts={len(res.contacts)} "
            f"(group='{req.group_name}')"
        )
        self.get_logger().warn("Not sending q_cmd due to invalid state.")
        return False

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

    def run(self):
        print("\n[TELEOP] Starting simple teleop (no logging, no start/end positions)")
        print("         Move your arm with the wearable to control the robot\n")

        last_tick = time.monotonic()

        while rclpy.ok():
            now = time.monotonic()

            # Rate gate
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

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
                
                # Check collision
                if not self._is_state_valid(q_cmd):
                    break
                
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

            # Send trajectory
            self._last_q_cmd = q_points[0]
            self._prev_W     = W.copy()
            self._prev_W_t   = now
            self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

            print(f"[tick] pts={len(q_points)} | W={np.round(W, 3)}")


# ============================ MAIN ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = SimpleTeleop()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()