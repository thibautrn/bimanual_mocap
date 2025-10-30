#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import struct
import socket
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    JointConstraint,
    RobotState,
)

# ============================ CONFIG ============================

GROUP_NAME        = "leftarm"
BASE_FRAME        = "base_link"

# With your naming: "elbow" frame is wrist_1_link, "wrist" is wrist_2_link
ELBOW_LINK_NAME   = "leftarm_wrist_1_link"
WRIST_LINK_NAME   = "leftarm_wrist_2_link"

# Leave wrist_1 free so the two spheres are actually controllable.
# Pin only the "useless" ones.
WRIST_JOINTS = [
    "leftarm_wrist_1_joint",
    "leftarm_wrist_2_joint",
]

# Shoulder anchor in BASE_FRAME
SHOULDER_ANCHOR = np.array([0.04, 0.32, 1.526], dtype=float)

# Fixed segment lengths (robot)
L1 = 0.2657  # shoulder -> elbow (wrist_1)
L2 = 0.2329  # elbow -> wrist_2

# UDP payload (25 floats = 100 bytes) and port
UDP_PORT = 50003

# Planner / constraint behavior (friendly to PickIK)
POS_RADIUS_ELBOW   = 0.04   # start generous; tighten to 0.02–0.01 once stable
POS_RADIUS_WRIST   = 0.04
CYCLE_SECONDS      = 0.20   # planning cadence on incoming packets
FREEZE_ALONG_PATH  = True  # goal-only pins → MUCH easier to solve
PLANNING_TIME_S    = 1.0
VEL_SCALE          = 1.0
ACC_SCALE          = 1.0
NUM_ATTEMPTS       = 1

# ================================================================

# Shared UDP state (latest sample)
hand_pos = None
larm_pos = None
uarm_pos = None
hand_rot = None
larm_rot = None
uarm_rot = None
hips_rot = None

def udp_listener(port=UDP_PORT):
    """Background thread: reads watch data and updates globals."""
    global hand_rot, hand_pos, larm_rot, larm_pos, uarm_rot, uarm_pos, hips_rot
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[UDP] Listening on udp://0.0.0.0:{port}")

    unpack = struct.Struct("ffff fff ffff fff ffff fff ffff").unpack_from
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

            hand_rot = (hw, hx, hy, hz)
            hand_pos = (hpx, hpy, hpz)

            larm_rot = (lw, lx, ly, lz)
            larm_pos = (lpx, lpy, lpz)

            uarm_rot = (uw, ux, uy, uz)
            uarm_pos = (upx, upy, upz)

            hips_rot = (qw, qx, qy, qz)
        except Exception as e:
            print(f"[UDP ERROR] {e}")


def remap_watch_to_base(p):
    """Map watch (x,y,z) -> robot base (x,y,z): (z, -x, y)."""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)


def scale_watch_to_robot(uarm, larm, hand):
    """
    Keep shoulder fixed; scale along L1 and L2 (fixed) to get robot elbow & wrist targets.
    Elbow = ELBOW_LINK_NAME; Wrist = WRIST_LINK_NAME.
    """
    sh = remap_watch_to_base(uarm)
    el = remap_watch_to_base(larm)
    wr = remap_watch_to_base(hand)

    v1 = el - sh
    v2 = wr - el
    u1 = v1 / (np.linalg.norm(v1) + 1e-9)
    u2 = v2 / (np.linalg.norm(v2) + 1e-9)

    sh_robot = SHOULDER_ANCHOR
    el_robot = sh_robot + L1 * u1
    wr_robot = el_robot + L2 * u2
    return sh_robot, el_robot, wr_robot


class MoveElbowWristPickIK(Node):
    def __init__(self):
        super().__init__("left_elbow_wrist_pickik_movegroup_udp")
        self._ac = ActionClient(self, MoveGroup, "move_action")

    # -------- Joint state helpers --------
    def _get_latest_joint_state(self, timeout=0.8):
        holder = {"msg": None}
        sub = self.create_subscription(
            JointState, "/joint_states", lambda m: holder.__setitem__("msg", m), 10
        )
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and holder["msg"] is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.destroy_subscription(sub)
        return holder["msg"]

    def _current_positions(self, names, timeout=0.8):
        msg = self._get_latest_joint_state(timeout=timeout)
        if msg is None:
            self.get_logger().warn("No /joint_states received; defaulting pinned joints to 0.0")
            return [0.0 for _ in names], None
        d = dict(zip(msg.name, msg.position))
        return [float(d.get(n, 0.0)) for n in names], msg

    def _fill_start_state(self, req: MotionPlanRequest, full_js_msg: JointState):
        if full_js_msg is None:
            return
        rs = RobotState()
        rs.joint_state = full_js_msg
        req.start_state = rs

    # -------- Constraint builders --------
    def _make_position_constraint(self, link_name, xyz, radius):
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = link_name
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(radius)]

        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
        pose.pose.orientation.w = 1.0  # irrelevant for a sphere

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose.pose)
        return pc

    # -------- Planner (uses PickIK under the hood via kinematics.yaml) --------
    def plan_elbow_wrist_with_pins(
        self,
        elbow_xyz,
        wrist_xyz,
        elbow_radius=POS_RADIUS_ELBOW,
        wrist_radius=POS_RADIUS_WRIST,
        freeze_along_path=FREEZE_ALONG_PATH,
        planning_time=PLANNING_TIME_S,
        vel_scale=VEL_SCALE,
        acc_scale=ACC_SCALE,
        attempts=NUM_ATTEMPTS,
    ):
        goal_c = Constraints()

        # Two position constraints (elbow + wrist)
        goal_c.position_constraints.append(
            self._make_position_constraint(ELBOW_LINK_NAME, elbow_xyz, elbow_radius)
        )
        goal_c.position_constraints.append(
            self._make_position_constraint(WRIST_LINK_NAME, wrist_xyz, wrist_radius)
        )

        # Pin wrist_2 & wrist_3 at goal (leave wrist_1 free!)
        pinned_now, full_js = self._current_positions(WRIST_JOINTS, timeout=0.8)
        for name, q in zip(WRIST_JOINTS, pinned_now):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(q)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            goal_c.joint_constraints.append(jc)

        # Build request
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP_NAME
        req.num_planning_attempts = int(attempts)
        req.allowed_planning_time = float(planning_time)
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)
        req.goal_constraints = [goal_c]

        if freeze_along_path:
            path_c = Constraints()
            for name, q in zip(WRIST_JOINTS, pinned_now):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = float(q)
                jc.tolerance_above = 0.03   # looser than goal (e.g., 0.02–0.05 rad)
                jc.tolerance_below = 0.03
                jc.weight = 1.0
                path_c.joint_constraints.append(jc)
            req.path_constraints = path_c

        # No path constraints — easier for solver
        self._fill_start_state(req, full_js)
        goal.request = req

        # Send
        self._ac.wait_for_server()
        self.get_logger().info(
            f"Targets: elbow={tuple(round(v,3) for v in elbow_xyz)} (r={elbow_radius}), "
            f"wrist={tuple(round(v,3) for v in wrist_xyz)} (r={wrist_radius})"
        )
        gh_future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_future)
        res_future = gh_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        rc = res_future.result().result.error_code.val
        self.get_logger().info(f"MoveIt error_code={rc}")
        return rc

    # ---------- main loop using latest UDP sample ----------
    def run_udp_follow(self):
        last_sent = 0.0
        while rclpy.ok():
            if hand_pos is None or larm_pos is None or uarm_pos is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            now = time.time()
            if now - last_sent < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.02)
                continue
            last_sent = now

            # Build targets (no orientation constraint)
            _, E, W = scale_watch_to_robot(uarm_pos, larm_pos, hand_pos)

            _ = self.plan_elbow_wrist_with_pins(
                elbow_xyz=tuple(E),
                wrist_xyz=tuple(W),
                elbow_radius=POS_RADIUS_ELBOW,
                wrist_radius=POS_RADIUS_WRIST,
                freeze_along_path=FREEZE_ALONG_PATH,
                planning_time=PLANNING_TIME_S,
                vel_scale=VEL_SCALE,
                acc_scale=ACC_SCALE,
                attempts=NUM_ATTEMPTS,
            )


def main():
    t = threading.Thread(target=udp_listener, daemon=True)
    t.start()

    rclpy.init()
    node = MoveElbowWristPickIK()
    try:
        node.run_udp_follow()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
