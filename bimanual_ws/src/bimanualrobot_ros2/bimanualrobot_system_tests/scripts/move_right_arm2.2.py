#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, numpy as np
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
from control_msgs.msg import JointTrajectoryControllerState

import random

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/Bimanual_Robot/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"
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

# IK joints (only pan + lift; lets you test those motors cleanly)
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
]

# Shoulder & link lengths (m)
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538
L2 = 0.23293990641364998


# Circle params
TIP_CENTER_C = np.array([0.51556, -0.48175, 1.554], dtype=float)  # your tip (also circle center)
CIRCLE_R_M   = 0.30        # 20 cm radius around C
SAFETY_MM    = 2.0         # shave a hair off max reach

SEND_RATE_HZ = 40.0         # how often you send new points (teleop feel)
N_POINTS     = 50         # samples along circle

LOOP_DT = 1.0 / SEND_RATE_HZ
# Per-point send timing (teleop-like)
SEND_TO_CONTROLLER   = True
USE_DISTANCE_TIMING  = True
MAX_SPEED_RAD_S      = 1.2   # cap for joint-based timing
MIN_DT_S             = 0.03  # never go faster than this per point
BASE_DT_S            = 0.05  # used if USE_DISTANCE_TIMING=False
START_PAUSE_S        = 0.0   # set to 2.0 if you want a pause before the loop

# IK solver params
W_POS, W_REG = 1.0, 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE = 0

# ============================ GEOMETRY ============================

def circle_points_on_reach(S, L1, L2, C, r=0.20, safety_mm=2.0, N=240, adjust_if_needed=True):
    """
    Return N (N,3) points of the circle = intersection of:
      - reach sphere centered at S, radius R = L1+L2 - safety
      - sphere centered at C (tip), radius r
    """
    S = np.asarray(S, float).reshape(3)
    C = np.asarray(C, float).reshape(3)
    R = float(L1 + L2) - safety_mm/1000.0

    d = np.linalg.norm(C - S)
    if d == 0.0:
        raise ValueError("S and C coincide; undefined circle.")

    if d > (R + r) or d < abs(R - r):
        if not adjust_if_needed:
            raise ValueError(f"No intersection: R={R:.4f}, r={r:.4f}, d={d:.4f}")
        r = max(1e-9, min(R + d - 1e-9, abs(R - d) + 1e-9))  # minimally adjust

    n = (C - S) / d
    a = (R**2 - r**2 + d**2) / (2*d)
    P0 = S + a * n                       # circle center in the plane
    h2 = max(R**2 - a**2, 0.0)
    h = math.sqrt(h2)                    # circle radius

    # orthonormal basis spanning plane ⟂ n
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, n)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, tmp); u /= np.linalg.norm(u)
    v = np.cross(n, u)

    theta = np.linspace(0.0, 2*np.pi, int(N), endpoint=False)
    pts = P0[None,:] + h*(np.cos(theta)[:,None]*u[None,:] + np.sin(theta)[:,None]*v[None,:])
    return pts

def triangle_points_on_reach(
    S, L1, L2, C, r=0.20, safety_mm=2.0,
    samples_per_edge=40, adjust_if_needed=True, theta0=0.0
):
    """
    Build an equilateral triangle inscribed in the same 'reach circle' you used before:
      - S: shoulder center (3,)
      - L1, L2: link lengths
      - C: tip center used to locate the circle plane (3,)
      - r: circle radius around C used to define the plane (same as before)
      - safety_mm: shave a little off max reach
      - samples_per_edge: how many points per triangle edge (>=2)
      - adjust_if_needed: minimally adjust r if spheres don't intersect
      - theta0: angular offset (radians) to rotate the triangle in-plane
    Returns: (M,3) points walking the polygon edges (closed path).
    """
    S = np.asarray(S, float).reshape(3)
    C = np.asarray(C, float).reshape(3)
    R = float(L1 + L2) - safety_mm/1000.0

    # --- same circle construction as circle_points_on_reach ---
    d = np.linalg.norm(C - S)
    if d == 0.0:
        raise ValueError("S and C coincide; undefined circle.")

    # ensure we have an intersection circle
    if d > (R + r) or d < abs(R - r):
        if not adjust_if_needed:
            raise ValueError(f"No intersection: R={R:.4f}, r={r:.4f}, d={d:.4f}")
        # minimally tweak r so the two spheres intersect
        r = max(1e-9, min(R + d - 1e-9, abs(R - d) + 1e-9))

    # plane normal & circle center in that plane
    n = (C - S) / d
    a = (R**2 - r**2 + d**2) / (2*d)
    P0 = S + a * n

    # circle radius in that plane
    h2 = max(R**2 - a**2, 0.0)
    h = math.sqrt(h2)

    # basis (u,v) spanning the circle plane
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, n)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, tmp); u /= np.linalg.norm(u)
    v = np.cross(n, u)

    # triangle vertices on the circle (equilateral, 120° apart)
    ang = [theta0, theta0 + 2*np.pi/3, theta0 + 4*np.pi/3]
    V = [P0 + h*(np.cos(a_)*u + np.sin(a_)*v) for a_ in ang]

    # sample along edges with straight-line interpolation (lies inside the circle)
    samples_per_edge = max(2, int(samples_per_edge))
    pts = []
    for i in range(3):
        A = V[i]
        B = V[(i+1) % 3]
        # exclude the last point to avoid duplicates when closing the loop
        for t in np.linspace(0.0, 1.0, samples_per_edge, endpoint=(i==2)):
            pts.append((1.0 - t) * A + t * B)
    return np.asarray(pts)

# ============================ IK ============================

def build_index_maps(model: pin.Model, joint_names):
    idx_q_vars = []
    for jn in joint_names:
        jid = model.getJointId(jn)
        if jid == 0:
            raise RuntimeError(f"Joint not found: {jn}")
        idx_q_vars.append(model.joints[jid].idx_q)
    lb_all = np.array(model.lowerPositionLimit, float)
    ub_all = np.array(model.upperPositionLimit, float)
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

def solve_wrist_ik_least_squares(robot: RobotWrapper, fid_wrist: int, idx_q_vars, lb, ub, target_W, seed=None):
    model = robot.model; data = model.createData()
    seed = np.zeros(len(idx_q_vars), float) if seed is None else np.array(seed, float)
    res = least_squares(
        residual_wrist_only, seed, bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full); pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err, res.nfev

# ============================ NODE ============================

class CircleTeleop(Node):
    def __init__(self):
        super().__init__("circle_points_ik_and_send")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # MoveIt validity (optional)
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Joint states (for filling non-IK)
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # FJT action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        self._last_q_cmd = None
        self._last_qvars = None 

        self._ctrl_state = None
        self.create_subscription(
            JointTrajectoryControllerState,
            "/right_arm_controller/state",
            self._on_ctrl_state,
            10
        )

    # --- helpers ---
    def _on_js(self, msg: JointState): self._latest_js = msg

    def _on_ctrl_state(self, msg: JointTrajectoryControllerState):
        self._ctrl_state = msg

    def _current_positions(self, names):
        # Prefer controller's actual positions
        if self._ctrl_state is not None:
            d = dict(zip(self._ctrl_state.joint_names, self._ctrl_state.actual.positions))
            missing = [n for n in names if n not in d]
            if missing:
                self.get_logger().error(f"Controller state missing joints: {missing}. Aborting.")
                raise RuntimeError("Missing joints in controller state")
            return [float(d[n]) for n in names]

        # Fallback to /joint_states (only if controller state isn't available)
        if self._latest_js is None:
            raise RuntimeError("No controller state or /joint_states yet")
        d = dict(zip(self._latest_js.name, self._latest_js.position))
        missing = [n for n in names if n not in d]
        if missing:
            self.get_logger().error(f"/joint_states missing joints: {missing}. Aborting.")
            raise RuntimeError("Missing joints in /joint_states")
        return [float(d[n]) for n in names]



    def _robot_state_from_qcmd(self, q_cmd):
        js = JointState(); js.name = JOINTS; js.position = list(map(float, q_cmd))
        js.header.stamp = self.get_clock().now().to_msg()
        rs = RobotState(); rs.joint_state = js
        return rs

    def _is_state_valid(self, q_cmd) -> bool:
        if not self._gsv.wait_for_service(timeout_sec=0.2):
            return True
        req = GetStateValidity.Request()
        req.robot_state = self._robot_state_from_qcmd(q_cmd)
        req.group_name = GROUP_NAME
        fut = self._gsv.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        return True if not res else bool(res.valid)

    def _send_single_point(self, q_target, reach_s):
        # Read current actual positions (same source you use everywhere else)
        try:
            q_now = self._current_positions(JOINTS)
        except RuntimeError:
            # Fallback to last command if available; else use target as both points
            q_now = self._last_q_cmd if self._last_q_cmd is not None else list(q_target)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        # Point 0: current pose at t=0
        pt0 = JointTrajectoryPoint()
        pt0.positions = list(map(float, q_now))
        pt0.time_from_start = Duration(sec=0, nanosec=0)

        # Point 1: target pose at t=reach_s
        pt1 = JointTrajectoryPoint()
        pt1.positions = list(map(float, q_target))
        sec = int(reach_s); nsec = int((reach_s - sec) * 1e9)
        pt1.time_from_start = Duration(sec=sec, nanosec=nsec)

        goal.trajectory.points = [pt0, pt1]
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)

        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

        self._last_q_cmd = q_target


    # --- main ---
    def run_once(self):
        # wait for state to be available
        while rclpy.ok() and (self._ctrl_state is None and self._latest_js is None):
            rclpy.spin_once(self, timeout_sec=0.05)

        # current measured pose (to fill non-IK joints)
        try:
            js_now = self._current_positions(JOINTS)
        except RuntimeError:
            if self._last_q_cmd is not None:
                js_now = list(self._last_q_cmd)  # last known good pose
            else:
                self.get_logger().error("No valid state to seed from; exiting to avoid zeros.")
                return

        # 1) Build circle points
        pts = circle_points_on_reach(
            SHOULDER_ANCHOR, L1, L2,
            TIP_CENTER_C, r=CIRCLE_R_M,
            safety_mm=SAFETY_MM, N=N_POINTS, adjust_if_needed=True
        )
        # pts = triangle_points_on_reach(
        #     SHOULDER_ANCHOR, L1, L2,
        #     TIP_CENTER_C,
        #     r=CIRCLE_R_M,
        #     safety_mm=SAFETY_MM,
        #     samples_per_edge=max(2, N_POINTS // 3),
        #     adjust_if_needed=True,
        #     theta0=0.0   # rotate triangle in-plane if you want
        # )

        self.get_logger().info(
            f"Generated {len(pts)} circle points around C={TIP_CENTER_C.tolist()} (r={CIRCLE_R_M:.3f} m)."
        )

        # --------- Construct a non-None seed every time ----------
        if self._last_qvars is not None:
            seed = np.array(self._last_qvars, dtype=float)
        else:
            # project current full joint vector onto IK_JOINTS
            js_vec = np.array(js_now, dtype=float)
            try:
                seed = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)
                if not np.all(np.isfinite(seed)):
                    raise ValueError("Non-finite in projected seed")
            except Exception:
                seed = np.zeros(len(self.idx_q_vars), dtype=float)
        # ---------------------------------------------------------

        # Optional: go to the first point then pause
        if START_PAUSE_S > 0 and len(pts) > 0:
            ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, pts[0], seed=seed
            )
            if not ok:
                # conservative fallback just in case
                seed_fallback = np.zeros(len(self.idx_q_vars), dtype=float)
                ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, pts[0], seed=seed_fallback
                )
            if ok:
                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)
                if self._is_state_valid(q_cmd) and SEND_TO_CONTROLLER:
                    self._send_single_point(q_cmd, reach_s=1.0)
                    time.sleep(START_PAUSE_S)
                # warm-start next solves
                seed = qvars.copy()
                js_now = q_cmd
                self._last_qvars = qvars.tolist()
                self._last_q_cmd = q_cmd

        prev_q = None
        # random.shuffle(pts)

        for i, W in enumerate(pts, start=0):
            ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, W, seed=seed
            )
            if not ok:
                # one retry from zeros (rare)
                seed_fallback = np.zeros(len(self.idx_q_vars), dtype=float)
                ok, qvars, qfull, pW, err, iters = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, W, seed=seed_fallback
                )
                if not ok:
                    self.get_logger().warn(f"IK failed at point {i}; stopping.")
                    break

            # build full command (IK joints from solution, others held)
            q_cmd = list(js_now)
            for jn, val in zip(IK_JOINTS, qvars):
                if jn in JOINTS:
                    q_cmd[JOINTS.index(jn)] = float(val)

            if not self._is_state_valid(q_cmd):
                self.get_logger().warn(f"Invalid state at point {i}; stopping.")
                break

            # per-point timing (distance-based) or fixed
            if USE_DISTANCE_TIMING and prev_q is not None:
                dq = np.abs(np.array(q_cmd) - np.array(prev_q))
                reach_s = max(float(np.max(dq))/float(MAX_SPEED_RAD_S), float(MIN_DT_S))
            else:
                reach_s = float(BASE_DT_S)

            # send
            self._send_single_point(q_cmd, reach_s=reach_s)

            # pace the loop at SEND_RATE_HZ
            if 't_next' not in locals():
                t_next = time.monotonic()
            t_next += LOOP_DT
            sleep_for = t_next - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                t_next = time.monotonic()

            # update warm-starts
            seed = qvars.copy()
            js_now = q_cmd
            prev_q = q_cmd
            self._last_qvars = qvars.tolist()
            self._last_q_cmd = q_cmd

        self.get_logger().info("Done streaming circle points.")


# ============================ main ============================

def main():
    rclpy.init()
    node = CircleTeleop()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
