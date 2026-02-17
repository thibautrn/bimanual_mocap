#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, math, subprocess, re
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

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "rightarm_wrist_2_link"     # IK target frame

# IK DOFs
IK_JOINTS = [
    "rightarm_shoulder_pan_joint",
    "rightarm_shoulder_lift_joint",
    "rightarm_elbow_joint",
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

ALLOWED_CONTACT_PAIRS = {
    ("leftarm_racket_handle", "leftarm_racket_blade"),
    ("rightarm_racket_handle", "rightarm_racket_blade"),
}
GROUP_NAME = "right_arm"

# Wearable → robot mapping
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow
L2 = 0.23293990641364998   # elbow→wrist_2

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats
MIN_W_DIST_M = 0.002

# Loop & timing
CYCLE_SECONDS   = 0.025   # ~40 Hz
UPSAMPLE_FACTOR = 2       # mid + current

# Smoothing / guards
LPF_CUTOFF_HZ      = 3.5
SPIKE_MAX_SPEED    = 2.0       # m/s
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)

# IK solver params
W_POS    = 1.0
W_REG    = 1e-3
MAX_ITERS= 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE  = 0

# ============================ BALL CONFIG ============================

WORLD_NAME  = "default"

PINNED_BALL_MODEL      = "ball_only"       # spawned model name
FREE_BALL_MODEL_NAME   = "ball_only_free"  # new model name after release
FREE_BALL_SDF          = "/home/thibaut/Documents/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/ball_only_free.sdf"

# Fallback pinned position (used if pose/info not yet available)
PINNED_BALL_POS = np.array([0.80, -0.52, 1.25], dtype=float)

BALL_RADIUS       = 0.03
CONTACT_MARGIN    = 0.2   # racket-ball trigger distance tolerance
BALL_PRINT_PERIOD = 0.5    # debug print throttle

# ================================================================

# Shared UDP state
_udp_lock = threading.Lock()
_hand_pos = None
_larm_pos = None
_uarm_pos = None

def udp_listener(port=UDP_PORT):
    """Receive wearable packet and keep the latest sample (thread)."""
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

# ---------------- Gazebo pose reader (ball) ----------------
class GzBallPoseReader:
    """
    Read /world/<world>/pose/info and track the pinned ball pose.
    target_regex should match 'ball_only' model / link names.
    """
    def __init__(self, world="default",
                 target_regex=r"^ball_only(?:::.+)?$",
                 verbose=False):
        self.topic = f"/world/{world}/pose/info"
        self.target = re.compile(target_regex)
        self.verbose = bool(verbose)

        self._proc = None
        self._th = None
        self._stop = threading.Event()

        self.have = False
        self.xyz  = np.zeros(3, dtype=float)
        self.vel  = np.zeros(3, dtype=float)

        self._last_ts  = None
        self._last_xyz = np.zeros(3, dtype=float)

    def start(self):
        if self._proc is not None:
            return
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1
        )
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop.set()
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.terminate()
        except Exception:
            pass

    def _dbg(self, *a):
        if self.verbose:
            print("[GzBallPoseReader]", *a)

    def _commit(self, x, y, z, reason=""):
        px = self.xyz[0] if x is None else x
        py = self.xyz[1] if y is None else y
        pz = self.xyz[2] if z is None else z

        now = time.time()
        if self._last_ts is not None:
            dt = max(now - self._last_ts, 1e-6)
            self.vel[:] = [
                (px - self._last_xyz[0]) / dt,
                (py - self._last_xyz[1]) / dt,
                (pz - self._last_xyz[2]) / dt,
            ]
        self.xyz[:] = (px, py, pz)
        self._last_xyz[:] = self.xyz
        self._last_ts = now
        self.have = True
        self._dbg(
            f"COMMIT {reason}: xyz=({px:.6f},{py:.6f},{pz:.6f}) "
            f"vel=({self.vel[0]:.3f},{self.vel[1]:.3f},{self.vel[2]:.3f})"
        )

    def _run(self):
        in_pose = False
        in_pos  = False
        cur_name = ""
        bx = by = bz = None
        name_matched = False

        rx_name   = re.compile(r'name:\s+"([^"]+)"')
        rx_posblk = re.compile(r'position\s*\{([^}]*)\}')
        rx_num_x  = re.compile(r'\bx:\s*([-\d.e+]+)')
        rx_num_y  = re.compile(r'\by:\s*([-\d.e+]+)')
        rx_num_z  = re.compile(r'\bz:\s*([-\d.e+]+)')

        for ln in self._proc.stdout:
            if self._stop.is_set():
                break
            s = ln.lstrip()

            # start pose block
            if not in_pose and s.startswith("pose {"):
                in_pose = True
                in_pos = False
                cur_name = ""
                bx = by = bz = None
                name_matched = False
                continue

            if not in_pose:
                continue

            # name line
            m_name = rx_name.search(ln)
            if m_name:
                cur_name = m_name.group(1)
                name_matched = bool(self.target.match(cur_name))
                continue

            # inline position block
            m_inline = rx_posblk.search(ln)
            if m_inline:
                block = m_inline.group(1)
                mx = rx_num_x.search(block)
                my = rx_num_y.search(block)
                mz = rx_num_z.search(block)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if name_matched:
                    self._commit(bx, by, bz, reason="inline")
                continue

            # multiline position
            if "position {" in ln:
                in_pos = True
                continue
            if in_pos:
                mx = rx_num_x.search(ln)
                my = rx_num_y.search(ln)
                mz = rx_num_z.search(ln)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if "}" in ln:
                    in_pos = False
                    if name_matched:
                        self._commit(bx, by, bz, reason="block")
                continue

            # end pose block
            if "}" in ln and in_pose and not in_pos:
                if name_matched:
                    self._commit(bx, by, bz, reason="end_pose")
                in_pose = False
                name_matched = False
                continue

    def get(self):
        return bool(self.have), self.xyz.copy(), self.vel.copy()

# ---------------- Pinocchio helpers ----------------
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

def build_q_index_map(model: pin.Model):
    """Map joint_name -> index in q (skip universe)."""
    mapping = {}
    for name in model.names[1:]:
        jid = model.getJointId(name)
        if jid == 0:
            continue
        j = model.joints[jid]
        if j.nq > 0:
            mapping[name] = j.idx_q
    return mapping

def q_from_joint_state(model: pin.Model, q_index_map, js: JointState):
    q = pin.neutral(model)
    name_to_pos = dict(zip(js.name, js.position))
    for jname, idx in q_index_map.items():
        if jname in name_to_pos:
            q[idx] = float(name_to_pos[jname])
    return q

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
    err_pos = (pW - target_W) * w_pos
    err_reg = w_reg * q_vars
    return np.hstack((err_pos, err_reg))

def solve_wrist_ik_least_squares(robot: RobotWrapper,
                                 fid_wrist: int,
                                 idx_q_vars,
                                 lb, ub,
                                 target_W,
                                 seed):
    model = robot.model
    data  = model.createData()
    res = least_squares(
        residual_wrist_only,
        np.array(seed, float),
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
    return res.success, q_vars_sol, q_full, pW, err

def pick_racket_frame(model: pin.Model):
    """Guess a good racket frame."""
    candidates = [
        "rightarm_racket_blade",
        "rightarm_racket_handle",
        "rightarm_ee_link",
    ]
    for name in candidates:
        if model.existFrame(name):
            return name
    racket_like = [f.name for f in model.frames if "racket" in f.name.lower()]
    if racket_like:
        return racket_like[0]
    return None

# ============================ NODE ============================

class WatchMidpointStreamer(Node):
    def __init__(self):
        super().__init__("watch_midpoint_streamer")

        # Pinocchio
        self.robot: RobotWrapper = RobotWrapper.BuildFromURDF(URDF_PATH, [])
        self.model: pin.Model = self.robot.model
        if not self.model.existFrame(F_WRIST):
            raise RuntimeError(f"Frame not found: {F_WRIST}")
        self.fid_wrist = self.model.getFrameId(F_WRIST)

        # Racket frame (for ball trigger)
        racket_frame_name = pick_racket_frame(self.model)
        if racket_frame_name is None:
            self.get_logger().warn(
                "No racket frame found; ball release on hit disabled."
            )
            self.fid_racket = None
        else:
            self.fid_racket = self.model.getFrameId(racket_frame_name)
            self.get_logger().info(
                f"Using racket frame '{racket_frame_name}' for ball trigger."
            )

        # Joint index maps
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)
        self.q_index_map = build_q_index_map(self.model)

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        # MoveIt validity
        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

        # LPF + spike guard
        self._w_lpf = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W = None
        self._prev_W_t = None

        # IK warm-start
        self._last_qvars = None
        self._last_q_cmd = None

        # Ball pose reader
        self._ball_reader = GzBallPoseReader(world=WORLD_NAME, verbose=False)
        self._ball_reader.start()
        self._last_ball_print = 0.0

        # Ball release state
        self.ball_released = False

    # --- helpers ---
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

    def _norm_pair(self, a, b):
        return (a, b) if a <= b else (b, a)

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

        # ---- fast pass ----
        if res.valid:
            return True

        # ---- print why invalid ----
        # Note: MoveIt can mark invalid due to collision OR joint limits, etc.
        self.get_logger().warn(
            f"State INVALID: contacts={len(res.contacts)} "
            f"(group='{req.group_name}')"
        )

        # Build whitelist normalized pairs once
        whitelist = {self._norm_pair(a, b) for (a, b) in ALLOWED_CONTACT_PAIRS}

        if res.contacts:
            # Each element is a moveit_msgs/msg/ContactInformation
            pairs = [self._norm_pair(c.contact_body_1, c.contact_body_2)
                    for c in res.contacts]

            # Print every raw contact pair (deduplicated)
            uniq_pairs = sorted(set(pairs))
            self.get_logger().warn("Contacts reported by MoveIt:")
            for (a, b) in uniq_pairs:
                tag = "ALLOWED" if (a, b) in whitelist else "BLOCKED"
                self.get_logger().warn(f"  - {a}  <->  {b}   [{tag}]")

            non_whitelisted = [p for p in uniq_pairs if p not in whitelist]

            # If everything is whitelisted, treat as valid
            if not non_whitelisted:
                return True
        else:
            # No contacts list, but state invalid -> could be bounds/constraints
            # There isn't always extra info in this service response.
            self.get_logger().warn("No contacts returned. Likely joint limits/constraints.")

        self.get_logger().warn("Not sending q_cmd due to invalid state.")
        return False


    def _send_followtraj_traj(self, q_points, total_dt):
        if not q_points:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        dt_step = float(total_dt) / float(len(q_points))
        t_accum = 0.0
        pts = []
        for q in q_points:
            t_accum += dt_step
            pt = JointTrajectoryPoint()
            pt.positions = list(map(float, q))
            sec = int(t_accum)
            nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)

        goal.trajectory.points = pts
        self._traj_ac.wait_for_server()
        self._traj_ac.send_goal_async(goal)

    def _get_ball_pose(self):
        return self._ball_reader.get()

    def _compute_racket_position(self):
        if self.fid_racket is None or self._latest_js is None:
            return None
        q = q_from_joint_state(self.model, self.q_index_map, self._latest_js)
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, q)
        pin.updateFramePlacements(self.model, data)
        p = data.oMf[self.fid_racket].translation
        return np.array([p[0], p[1], p[2]], dtype=float)

    def _trigger_ball_release_if_hit(self):
        if self.ball_released or self.fid_racket is None:
            return

        racket_pos = self._compute_racket_position()
        if racket_pos is None:
            return

        have_ball, ball_pos, _ = self._get_ball_pose()
        center = ball_pos if have_ball else PINNED_BALL_POS

        dist = float(np.linalg.norm(racket_pos - center))
        if dist > (BALL_RADIUS + CONTACT_MARGIN):
            return

        self.get_logger().info(
            f"Racket-ball proximity {dist:.3f} m <= "
            f"{BALL_RADIUS + CONTACT_MARGIN:.3f} -> releasing ball."
        )

        # 1) Remove pinned model
        try:
            rm_cmd = [
                "gz", "service",
                "-s", f"/world/{WORLD_NAME}/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                '--req', f'name: "{PINNED_BALL_MODEL}", type: 2',
            ]
            self.get_logger().info("Removing pinned ball: " + " ".join(rm_cmd))
            subprocess.run(rm_cmd, check=False)
        except Exception as e:
            self.get_logger().error(f"Failed to remove {PINNED_BALL_MODEL}: {e}")

        # 2) Spawn free ball at same pose via /world/default/create
        try:
            create_req = (
                f'sdf_filename: "{FREE_BALL_SDF}", '
                f'name: "{FREE_BALL_MODEL_NAME}", '
                f'pose: {{ position: {{ x: {center[0]}, y: {center[1]}, z: {center[2]+0.50} }}, '
                f'orientation: {{ x: 0, y: 0, z: 0, w: 1 }} }}'
            )
            spawn_cmd = [
                "gz", "service",
                "-s", f"/world/{WORLD_NAME}/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", create_req,
            ]
            self.get_logger().info("Spawning free ball: " + " ".join(spawn_cmd))
            subprocess.run(spawn_cmd, check=False)
        except Exception as e:
            self.get_logger().error(f"Failed to spawn free ball: {e}")
            return

        self.ball_released = True
        self.get_logger().info("Ball released: pinned removed, free spawned.")

    # --- main loop ---
    def run(self):
        # wait for /joint_states
        t0 = time.time()
        while rclpy.ok() and self._latest_js is None and (time.time() - t0) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)

        last_tick = 0.0

        while rclpy.ok():
            now = time.time()

            # Debug ball pose (optional)
            have_ball, bp, bv = self._get_ball_pose()
            if have_ball and (now - self._last_ball_print) >= BALL_PRINT_PERIOD:
                print(
                    f"[ball] pos=({bp[0]:.3f},{bp[1]:.3f},{bp[2]:.3f}) "
                    f"vel=({bv[0]:.2f},{bv[1]:.2f},{bv[2]:.2f})"
                )
                self._last_ball_print = now

            # Check racket-ball proximity for release
            if not self.ball_released:
                self._trigger_ball_release_if_hit()

            # Loop timing
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            # Read wearable data
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            # Map to wrist target
            _, W_raw = scale_watch_to_right_robot(up, lp, hp)
            W = self._w_lpf.update(W_raw, now)

            # Spike guard
            if self._prev_W is not None and self._prev_W_t is not None:
                dt = max(now - self._prev_W_t, 1e-3)
                if np.linalg.norm(W - self._prev_W) / dt > SPIKE_MAX_SPEED:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

            # Waypoints (mid + current)
            if self._prev_W is not None and UPSAMPLE_FACTOR >= 2:
                W_mid = 0.5 * (self._prev_W + W)
                W_list = [W_mid, W]
            else:
                W_list = [W]

            js_now = self._current_positions(JOINTS, default=0.0)

            # IK seed
            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()
            else:
                js_vec = np.array(js_now, dtype=float)
                qvars_seed = np.array(
                    [js_vec[JOINTS.index(jn)] for jn in IK_JOINTS],
                    dtype=float,
                )

            q_points = []
            # Solve IK for each waypoint
            for Wk in W_list:
                ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist,
                    self.idx_q_vars, self.lb, self.ub,
                    Wk, seed=qvars_seed,
                )
                if not ok:
                    q_points = []
                    break

                q_cmd = list(js_now)
                for jn, val in zip(IK_JOINTS, qvars):
                    if jn in JOINTS:
                        q_cmd[JOINTS.index(jn)] = float(val)

                if not self._is_state_valid(q_cmd):
                    q_points = []
                    break

                q_points.append(q_cmd)
                qvars_seed = qvars

            if not q_points:
                self._prev_W = W.copy()
                self._prev_W_t = now
                rclpy.spin_once(self, timeout_sec=0.01)
                continue

            # Deadband
            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    self._prev_W = W.copy()
                    self._prev_W_t = now
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue

            # Remember last
            self._last_qvars = qvars_seed
            self._last_q_cmd = q_points[0]
            self._prev_W = W.copy()
            self._prev_W_t = now

            # Send traj for this cycle
            self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

            print(f"[tick] pts={len(q_points)} over {CYCLE_SECONDS:.3f}s | W={np.round(W,3)}")
            rclpy.spin_once(self, timeout_sec=0.01)

# ============================ main ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = WatchMidpointStreamer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
