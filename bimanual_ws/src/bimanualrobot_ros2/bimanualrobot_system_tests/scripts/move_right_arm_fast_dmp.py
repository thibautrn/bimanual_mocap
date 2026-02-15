#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from pathlib import Path

# ============================ GLOBAL ARM SELECTION ============================
LEFT = False  # Set to False to use right arm

# ============================ CONFIG ============================

LOG_DIR = Path("/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs/joints")

def arm(name):
    """Replace ARM with left or right based on LEFT global"""
    if LEFT:
        return name.replace("ARM", "left")
    else:
        return name.replace("ARM", "right")

# Joint configuration
JOINTS = [
    arm("ARMarm_shoulder_pan_joint"),
    arm("ARMarm_shoulder_lift_joint"),
    arm("ARMarm_elbow_joint"),
    arm("ARMarm_wrist_1_joint"),
    arm("ARMarm_wrist_2_joint"),
    arm("ARMarm_wrist_3_joint"),
]

ACTION_NAME = arm("/ARM_arm_controller/follow_joint_trajectory")

CYCLE_SECONDS = 0.025
LPF_CUTOFF_HZ = 3.5
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

# DMP CONFIG
BASELINE_PATH = str(LOG_DIR / "baseline.npz")
WEIGHTS_PATH = str(LOG_DIR / "joints_20260206_182508_weights.npz")
DMP_DT = 0.025

# ============================ LOW-PASS FILTER ============================

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

# ============================ DMP ROLLOUT - JOINT SPACE ============================

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

def rollout_dmp_joints(y0, g, W, K, D, tau, dt, c, h, alpha_s):
    """
    DMP rollout for N-dimensional joint space
    y0, g: initial and goal joint angles (N-dim)
    W: weights (N x M) where N = num joints, M = num basis functions
    Returns: Y (timesteps x N) joint angle trajectory
    """
    N_steps = int(np.round(tau / dt)) + 1
    N_joints = len(y0)
    
    S = canonical_by_steps(N_steps, dt, tau, alpha_s)
    Phi = design_matrix(S, c, h)
    
    Y = np.zeros((N_steps, N_joints), float)
    Yd = np.zeros_like(Y)
    Y[0] = y0
    
    for k in range(N_steps - 1):
        acc = np.zeros(N_joints, float)
        for d in range(N_joints):
            f = float(Phi[k].dot(W[d]))
            acc[d] = (
                K * (g[d] - Y[k, d])
                - D * tau * Yd[k, d]
                + K * f * (g[d] - y0[d])
            ) / (tau ** 2)
        Yd[k + 1] = Yd[k] + acc * dt
        Y[k + 1] = Y[k] + Yd[k + 1] * dt
    
    return Y

class DMPJointRolloutSource:
    """DMP source that directly outputs joint angles"""
    
    def __init__(self, baseline_path, weights_path, dt):
        b = np.load(baseline_path)
        w = np.load(weights_path, allow_pickle=True)

        c, h = b["c"], b["h"]
        K, D = float(b["K"]), float(b["D"])
        alpha_s = float(b["alpha_s"])
        tau = float(b["run_time"])
        W = w["w"]  # Shape: (N_dmp_dims, M_basis_functions)

        y0 = w["y0"].astype(float)  # Initial joint angles (N_dmp_dims,)
        g = w["g"].astype(float)    # Goal joint angles (N_dmp_dims,)

        # Check dimensions
        n_dmp_dims = W.shape[0]
        print(f"[DMP] Loaded weights with {n_dmp_dims} dimensions")
        print(f"[DMP] y0 shape: {y0.shape}, g shape: {g.shape}")
        
        # Rollout DMP in joint space (only for the dims in the weights file)
        Y = rollout_dmp_joints(y0, g, W, K, D, tau, dt, c, h, alpha_s)

        self.Y = Y  # (timesteps, N_dmp_dims) - might be 3 or 6
        self.dt = float(dt)
        self.N = len(Y)
        self.k = 0
        self.n_dmp_dims = n_dmp_dims

    def next(self):
        """Return next joint angle vector (only the IK joints if weights are 3D)"""
        if self.k >= self.N:
            return None
        joint_angles = self.Y[self.k].copy()
        self.k += 1
        return joint_angles

# ============================ NODE ============================

class DMPRobotExecutor(Node):
    """Simple DMP executor - just sends joint angles from DMP"""
    
    def __init__(self, dmp_source: DMPJointRolloutSource):
        super().__init__("dmp_robot_executor")
        self._dmp = dmp_source

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # Smoothing - one filter per joint
        self._joint_lpf = [LowPassEMA(fc_hz=LPF_CUTOFF_HZ) for _ in JOINTS]
        
        self._last_dmp_time = None
        self._pending_joints = None
        self._last_q_cmd = None

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _send_joint_trajectory(self, joint_angles):
        """Send joint angles to robot"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        pt = JointTrajectoryPoint()
        pt.positions = list(map(float, joint_angles))
        pt.time_from_start = Duration(sec=0, nanosec=int(CYCLE_SECONDS * 1e9))
        
        goal.trajectory.points = [pt]
        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    def run(self):
        # Wait for joint states
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        arm_side = "LEFT" if LEFT else "RIGHT"
        n_dmp_dims = self._dmp.n_dmp_dims
        print(f"[DMP] Starting execution on {arm_side} arm")
        print(f"[DMP] DMP dimensions: {n_dmp_dims} (3 IK joints or 6 all joints)")
        print(f"[DMP] Controller joints: {JOINTS}")

        self._last_dmp_time = time.time()
        self._pending_joints = None
        self._last_q_cmd = None

        finished = False
        last_tick = 0.0

        while rclpy.ok() and not finished:
            now = time.time()

            # Advance DMP
            while (now - self._last_dmp_time) >= self._dmp.dt:
                joint_raw = self._dmp.next()  # Returns (3,) or (6,) depending on weights
                if joint_raw is None:
                    finished = True
                    break
                self._last_dmp_time += self._dmp.dt
                
                # If DMP only has 3 joints (IK joints), pad with zeros for wrist joints
                if n_dmp_dims == 3:
                    # joint_raw is [q1, q2, q3] (shoulder_pan, shoulder_lift, elbow)
                    # Pad with zeros for wrist joints: [q1, q2, q3, 0, 0, 0]
                    joint_raw_full = np.zeros(len(JOINTS), float)
                    joint_raw_full[:3] = joint_raw  # First 3 are IK joints
                    joint_raw = joint_raw_full
                
                # Apply low-pass filter to each joint
                joint_filtered = np.zeros(len(JOINTS), float)
                for i in range(len(JOINTS)):
                    joint_filtered[i] = self._joint_lpf[i].update(
                        joint_raw[i], self._last_dmp_time
                    )
                
                self._pending_joints = joint_filtered

            if finished:
                break

            # Control tick
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            if self._pending_joints is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            q_cmd = self._pending_joints.copy()

            # Deadband - skip if change too small
            if self._last_q_cmd is not None:
                dq = np.abs(q_cmd - self._last_q_cmd)
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

            # Send to robot
            self._last_q_cmd = q_cmd.copy()
            self._send_joint_trajectory(q_cmd)

            print(f"[DMP] q={np.round(q_cmd, 3)} | finished={finished}")

            rclpy.spin_once(self, timeout_sec=0.01)

        print("[DMP] Finished full rollout.")
        time.sleep(0.5)

# ============================ MAIN ============================

def main():
    dmp_src = DMPJointRolloutSource(
        baseline_path=BASELINE_PATH,
        weights_path=WEIGHTS_PATH,
        dt=DMP_DT,
    )

    rclpy.init()
    node = DMPRobotExecutor(dmp_src)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()