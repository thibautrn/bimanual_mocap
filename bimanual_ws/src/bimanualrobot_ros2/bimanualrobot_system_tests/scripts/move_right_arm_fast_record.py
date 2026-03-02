#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, math
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

from std_srvs.srv import Trigger
from pathlib import Path

import subprocess, re, threading, time

# ============================ CONFIG ============================

# URDF & IK target frame
URDF_PATH = "/home/asurite.ad.asu.edu/troisin/Documents/robot/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_description/urdf/robots/bimanualrobot.urdf"
F_WRIST   = "leftarm_wrist_2_link"     # IK target frame
LOG_DIR = Path("/home/asurite.ad.asu.edu/troisin/Documents/robot/bimanual_mocap/bimanual_ws/src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs")  # change if you want

# IK DOFs
IK_JOINTS = [
    "leftarm_shoulder_pan_joint",
    "leftarm_shoulder_lift_joint",
    "leftarm_elbow_joint",
    # "leftarm_wrist_1_joint",
]

# Controller & joint order (must match the controller)
ACTION_NAME = "/left_arm_controller/follow_joint_trajectory"
JOINTS = [
    "leftarm_shoulder_pan_joint",
    "leftarm_shoulder_lift_joint",
    "leftarm_elbow_joint",
    "leftarm_wrist_1_joint",
    "leftarm_wrist_2_joint",
    "leftarm_wrist_3_joint",
]

ALLOWED_CONTACT_PAIRS = {
        ("rightarm_racket_handle", "rightarm_racket_blade"),
        ("leftarm_racket_handle", "leftarm_racket_blade"),
    }
GROUP_NAME = "left_arm"

# Wearable → robot mapping
SHOULDER_ANCHOR = np.array([0.045, 0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow
L2 = 0.23293990641364998   # elbow→wrist_2

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats
MIN_W_DIST_M = 0.002  

# Loop & timing
CYCLE_SECONDS      = 0.06     # loop at ~40 Hz
UPSAMPLE_FACTOR    = 2         # target ~2 points per cycle (midpoint + current)

# Smoothing / guards
LPF_CUTOFF_HZ      = 3.5       # light smoothing of wrist positions
SPIKE_MAX_SPEED    = 2.0       # m/s (reject absurd jumps)
MIN_JOINT_STEP_RAD = np.deg2rad(0.1)  # tiny deadband on first point

# IK solver params
W_POS = 1.0
W_REG = 1e-3
MAX_ITERS = 300
XTOL = FTOL = GTOL = 1e-8
VERBOSE = 0

# Contact detection
CONTACT_COOLDOWN_S = 1.5  # Don't log same contact twice within 1500ms
CONTACT_DISTANCE_THRESHOLD = 0.2  # 20cm - consider contact if ball-racket distance < this

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


class GzPoseReader:
    """
    Generic reader for Gazebo's aggregate pose stream.
    Reads position for any entity matching the target regex.
    """

    def __init__(self, world="default", target_regex=r"end_ball$", name="PoseReader", verbose=False):
        self.topic = f"/world/{world}/pose/info"
        self.target = re.compile(target_regex)
        self.name = name
        self.verbose = bool(verbose)

        self._proc = None
        self._th = None
        self._stop = threading.Event()

        # public state
        self.have = False
        self.xyz  = np.zeros(3, dtype=float)
        self.vel  = np.zeros(3, dtype=float)

        # internals
        self._last_ts  = None
        self._last_xyz = np.zeros(3, dtype=float)

    # ---------- lifecycle ----------
    def start(self):
        if self._proc is not None:
            return
        if self.verbose:
            print(f"[{self.name}] spawn: gz topic -e -t {self.topic}")
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

    # ---------- helpers ----------
    def _dbg(self, *a):
        if self.verbose:
            print(f"[{self.name}]", *a)

    def _commit(self, x, y, z, reason=""):
        # fill missing from last-known
        px = self.xyz[0] if x is None else x
        py = self.xyz[1] if y is None else y
        pz = self.xyz[2] if z is None else z

        now = time.time()
        if self._last_ts is not None:
            dt = max(now - self._last_ts, 1e-6)
            self.vel[:] = [(px - self._last_xyz[0]) / dt,
                           (py - self._last_xyz[1]) / dt,
                           (pz - self._last_xyz[2]) / dt]
        self.xyz[:] = (px, py, pz)
        self._last_xyz[:] = self.xyz
        self._last_ts = now
        self.have = True
        self._dbg(f"COMMIT {reason}: xyz=({px:.6f},{py:.6f},{pz:.6f})")

    # ---------- core parsing loop ----------
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

            # start pose
            if not in_pose and s.startswith("pose {"):
                in_pose = True; in_pos = False
                cur_name = ""; bx = by = bz = None
                name_matched = False
                self._dbg("POSE{")
                continue

            # outside pose: sometimes position appears anyway
            if not in_pose:
                m_any = rx_posblk.search(ln)
                if m_any:
                    block = m_any.group(1)
                    mx = rx_num_x.search(block); my = rx_num_y.search(block); mz = rx_num_z.search(block)
                    if mx: bx = float(mx.group(1))
                    if my: by = float(my.group(1))
                    if mz: bz = float(mz.group(1))
                    self._dbg(f"pos-outside: x={bx} y={by} z={bz}")
                continue

            # inside pose: name
            m_name = rx_name.search(ln)
            if m_name:
                cur_name = m_name.group(1)
                name_matched = bool(self.target.search(cur_name))
                self._dbg("NAME:", cur_name, "match=", name_matched)
                if name_matched and (bx is not None or by is not None or bz is not None):
                    self._commit(bx, by, bz, reason="on_name_match")
                continue

            # inline position
            m_inline = rx_posblk.search(ln)
            if m_inline:
                block = m_inline.group(1)
                mx = rx_num_x.search(block); my = rx_num_y.search(block); mz = rx_num_z.search(block)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                self._dbg(f"pos-inline: x={bx} y={by} z={bz}")
                if name_matched:
                    self._commit(bx, by, bz, reason="inline_after_name")
                continue

            # multiline position
            if "position {" in ln:
                in_pos = True; self._dbg("  POSITION{"); continue
            if in_pos:
                mx = rx_num_x.search(ln); my = rx_num_y.search(ln); mz = rx_num_z.search(ln)
                if mx: bx = float(mx.group(1))
                if my: by = float(my.group(1))
                if mz: bz = float(mz.group(1))
                if "}" in ln:
                    in_pos = False
                    self._dbg(f"  }}POSITION  (x={bx} y={by} z={bz})")
                    if name_matched:
                        self._commit(bx, by, bz, reason="end_position_block")
                continue

            # end pose
            if "}" in ln and in_pose and not in_pos:
                self._dbg("}POSE")
                if name_matched:
                    self._commit(bx, by, bz, reason="end_of_block")
                in_pose = False; name_matched = False
                continue

    # ---------- public getter ----------
    def get(self):
        """Returns (have_pose: bool, xyz: np.array(3), vel: np.array(3))"""
        return bool(self.have), self.xyz.copy(), self.vel.copy()


# Convenience aliases
class GzBallPoseReader(GzPoseReader):
    def __init__(self, world="default", verbose=False):
        super().__init__(world=world, target_regex=r"(?:^end_ball$|::end_ball$)", 
                        name="BallReader", verbose=verbose)
        
        self._world_to_real = np.array([0.78, 0.55, 2.10], dtype=float)

    def get(self):
        have, ball_world, vel = super().get()
        if not have:
            return False, np.zeros(3), np.zeros(3)

        # Express ball in your real-world frame (same as W)
        ball_real = ball_world + self._world_to_real
        return True, ball_real, vel


class GzRacketPoseReader(GzPoseReader):
    def __init__(self, world="default", verbose=False):
        super().__init__(
            world=world,
            # Match the left EE link from /world/.../pose/info
            # Works for names like:
            #   bimanualrobot::leftarm_ee_link
            #   leftarm_ee_link
            target_regex=r"(?:^leftarm_ee_link$|::leftarm_ee_link$)",
            name="RacketReader",
            verbose=verbose,
        )

        # TODO: set using your real transform:
        # measure in Gazebo: ee_link -> racket middle / handle tip
        # For now: along +Z in ee frame, adjust after checking.
        self._offset = np.array([0.0, 0.0, 0.09], dtype=float)

    def get(self):
        """
        Returns:
            have (bool),
            racket_center_xyz (np.array(3)),
            vel (np.array(3))
        """
        have, ee_xyz, vel = super().get()
        if not have:
            return False, np.zeros(3), np.zeros(3)

        racket_center = ee_xyz + self._offset
        return True, racket_center, vel




# ---------------- mapping helpers ----------------
def remap_watch_to_base(p):
    """(x,y,z)_watch -> (z, -x, y)_base"""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_left_robot(uarm_watch, larm_watch, hand_watch):
    """Map watch 3 pts (upper, lower, hand) to robot-base wrist (W)."""
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)

    # left-arm flip kept from your previous mapping
    # Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

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
        residual_wrist_only, np.array(seed, float),
        bounds=(lb, ub),
        args=(model, data, fid_wrist, idx_q_vars, target_W, W_POS, W_REG),
        max_nfev=MAX_ITERS, xtol=XTOL, ftol=FTOL, gtol=GTOL, verbose=VERBOSE
    )
    q_vars_sol = res.x
    q_full     = full_q_from_vars(model, idx_q_vars, q_vars_sol)
    pin.forwardKinematics(model, data, q_full); pin.updateFramePlacements(model, data)
    pW = data.oMf[fid_wrist].translation
    err = np.linalg.norm(pW - target_W)
    return res.success, q_vars_sol, q_full, pW, err

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

        # IK indexing
        self.idx_q_vars, self.lb, self.ub = build_index_maps(self.model, IK_JOINTS)

        # Joint states
        self._latest_js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

        self._gsv = self.create_client(GetStateValidity, "/check_state_validity")

        # Trajectory action
        self._traj_ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._traj_ac.wait_for_server() 

        # LPF + spike guard state
        self._w_lpf = LowPassEMA(fc_hz=LPF_CUTOFF_HZ)
        self._prev_W = None
        self._prev_W_t = None

        # IK warm-start
        self._last_qvars = None
        self._last_q_cmd = None

        # Pose readers
        self._ball_reader = GzBallPoseReader(world="default", verbose=False)
        self._ball_reader.start()
        
        self._racket_reader = GzRacketPoseReader(world="default", verbose=False)
        self._racket_reader.start()

        # Contact detection state
        self._last_contact_logged = None

        # Log file handles
        self._log_fh = None
        self._ball_log_fh = None
        self._racket_log_fh = None
        self._contact_log_fh = None
        
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._srv_log_start = self.create_service(Trigger, "~/log/start", self._log_start_cb)
        self._srv_log_stop  = self.create_service(Trigger, "~/log/stop",  self._log_stop_cb)

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
        now = self.get_clock().now().to_msg()
        js.header.stamp = now

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
        if res.valid:
            return True

        # If invalid, inspect contacts
        if res.contacts:
            pairs = [(self._norm_pair(c.contact_body_1, c.contact_body_2)) for c in res.contacts]
            non_whitelisted = [
                p for p in pairs
                if p not in { self._norm_pair(*pair) for pair in ALLOWED_CONTACT_PAIRS }
            ]
            if not non_whitelisted:
                return True

        self.get_logger().warn("State INVALID (collision or limits). Not sending.")
        return False
    
    def _log_start_cb(self, req, resp):
        if any([self._log_fh, self._ball_log_fh, self._racket_log_fh, self._contact_log_fh]):
            resp.success = False
            resp.message = "Already logging."
            return resp

        ts = time.strftime("%Y%m%d_%H%M%S")
        wrist_path = LOG_DIR / f"wrist_{ts}.txt"
        ball_path = LOG_DIR / f"ball_{ts}.txt"
        racket_path = LOG_DIR / f"racket_{ts}.txt"
        contact_path = LOG_DIR / f"contacts_{ts}.txt"

        try:
            self._log_fh = open(wrist_path, "w", buffering=1)
            self._log_fh.write("# t_sec  x  y  z   (wrist in base frame)\n")
            
            self._ball_log_fh = open(ball_path, "w", buffering=1)
            self._ball_log_fh.write("# t_sec  x  y  z  vx  vy  vz   (ball in world frame)\n")
            
            self._racket_log_fh = open(racket_path, "w", buffering=1)
            self._racket_log_fh.write("# t_sec  x  y  z   (racket blade center in world frame)\n")
            
            self._contact_log_fh = open(contact_path, "w", buffering=1)
            self._contact_log_fh.write("# t_sec  racket_x  racket_y  racket_z  ball_x  ball_y  ball_z  ball_vx  ball_vy  ball_vz  distance\n")
            
            resp.success = True
            resp.message = f"Logging to {wrist_path.parent} (4 files)"
            self.get_logger().info(resp.message)
            
        except Exception as e:
            # Clean up on error
            for fh in [self._log_fh, self._ball_log_fh, self._racket_log_fh, self._contact_log_fh]:
                try:
                    if fh: fh.close()
                except: pass
            self._log_fh = self._ball_log_fh = self._racket_log_fh = self._contact_log_fh = None
            resp.success = False
            resp.message = f"Failed to open logs: {e}"
            self.get_logger().error(resp.message)
        
        return resp

    def _log_stop_cb(self, req, resp):
        if not any([self._log_fh, self._ball_log_fh, self._racket_log_fh, self._contact_log_fh]):
            resp.success = False
            resp.message = "Not logging."
            return resp

        paths = []
        for fh in [self._log_fh, self._ball_log_fh, self._racket_log_fh, self._contact_log_fh]:
            try:
                if fh is not None:
                    paths.append(fh.name)
                    fh.close()
            except: pass
        
        self._log_fh = self._ball_log_fh = self._racket_log_fh = self._contact_log_fh = None
        
        resp.success = True
        resp.message = "Saved: " + ", ".join(map(str, paths)) if paths else f"Saved to {LOG_DIR}"
        self.get_logger().info(resp.message)
        return resp

    def _send_followtraj_traj(self, q_points, total_dt):
        """Send a tiny multi-point trajectory that spans total_dt seconds."""
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
            sec = int(t_accum); nsec = int((t_accum - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            pts.append(pt)

        goal.trajectory.points = pts
        self._traj_ac.send_goal_async(goal)

    # --- main loop ---
    def run(self):
        last_tick = time.monotonic()
        while rclpy.ok():
            now = time.monotonic()

            # RATE GATE FIRST: bail early with no extra work
            if now - last_tick < CYCLE_SECONDS:
                rclpy.spin_once(self, timeout_sec=0.01)
                continue
            last_tick = now

            # (A) Control tick starts here: snapshot sensors once per tick
            with _udp_lock:
                hp, lp, up = _hand_pos, _larm_pos, _uarm_pos
            if hp is None or lp is None or up is None:
                continue

            _, W_raw = scale_watch_to_left_robot(up, lp, hp)
            W = self._w_lpf.update(W_raw, now)

            # Log wrist
            if self._log_fh is not None:
                x, y, z = map(float, W)
                self._log_fh.write(f"{now:.6f} {x:.6f} {y:.6f} {z:.6f}\n")

            # Get ball and racket poses
            have_ball, ball_pos, ball_vel = self._ball_reader.get()
            have_racket, racket_pos, _ = self._racket_reader.get()

            # Log ball
            if self._ball_log_fh is not None and have_ball:
                self._ball_log_fh.write(f"{now:.6f} {ball_pos[0]:.6f} {ball_pos[1]:.6f} {ball_pos[2]:.6f} "
                                       f"{ball_vel[0]:.6f} {ball_vel[1]:.6f} {ball_vel[2]:.6f}\n")
            
            # Log racket
            if self._racket_log_fh is not None and have_racket:
                self._racket_log_fh.write(f"{now:.6f} {racket_pos[0]:.6f} {racket_pos[1]:.6f} {racket_pos[2]:.6f}\n")
                print(f"Racket pos: {racket_pos}")
            
            # Detect and log contact events (distance-based)
            if self._contact_log_fh is not None and have_ball and have_racket:
                distance = np.linalg.norm(ball_pos - racket_pos)
                print(distance)
                # Contact detected if distance is below threshold
                if distance < CONTACT_DISTANCE_THRESHOLD:
                    # Cooldown: don't log same contact multiple times
                    if (self._last_contact_logged is None or 
                        now - self._last_contact_logged > CONTACT_COOLDOWN_S):
                        
                        self._contact_log_fh.write(
                            f"{now:.6f} "
                            f"{racket_pos[0]:.6f} {racket_pos[1]:.6f} {racket_pos[2]:.6f} "
                            f"{ball_pos[0]:.6f} {ball_pos[1]:.6f} {ball_pos[2]:.6f} "
                            f"{ball_vel[0]:.6f} {ball_vel[1]:.6f} {ball_vel[2]:.6f} "
                            f"{distance:.6f}\n"
                        )
                        self._last_contact_logged = now
                        print(f"[CONTACT] Ball-racket contact detected at t={now:.3f}, distance={distance*1000:.1f}mm")

            # simple spike guard w.r.t. previous W
            if self._prev_W is not None and self._prev_W_t is not None:
                dt = max(now - self._prev_W_t, 1e-3)
                if np.linalg.norm(W - self._prev_W) / dt > SPIKE_MAX_SPEED:
                    # drop this sample; try next tick
                    continue

            # Build upsampled waypoint list: [midpoint, current] or [current]
            W_list = []
            if self._prev_W is not None and UPSAMPLE_FACTOR >= 2:
                W_mid = 0.5 * (self._prev_W + W)
                W_list = [W_mid, W]
            else:
                W_list = [W]

            # current measured joints (for non-IK joints)
            js_now = self._current_positions(JOINTS, default=0.0)

            if self._last_qvars is not None:
                qvars_seed = self._last_qvars.copy()   # warm start from last good IK
            else:
                # Project the *measured* joint state onto the IK DOFs (better than zeros)
                js_vec = np.array(js_now, dtype=float)
                qvars_seed = np.array([js_vec[JOINTS.index(jn)] for jn in IK_JOINTS], dtype=float)

            # IK each waypoint (warm-start)
            q_points = []
            last_qvars_solved = None
            for Wk in W_list:
                ok, qvars, qfull, pW, err = solve_wrist_ik_least_squares(
                    self.robot, self.fid_wrist, self.idx_q_vars, self.lb, self.ub, Wk, seed=qvars_seed
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
                qvars_seed = qvars  # warm-start next point
                last_qvars_solved = qvars

            if last_qvars_solved is not None:
                self._last_qvars = last_qvars_solved.copy()
            if not q_points:
                # remember W anyway
                self._prev_W = W.copy(); self._prev_W_t = now
                continue

            # tiny deadband on the first point to avoid spam
            if self._last_q_cmd is not None:
                dq = np.abs(np.array(q_points[0]) - np.array(self._last_q_cmd))
                if float(np.max(dq)) < MIN_JOINT_STEP_RAD:
                    self._prev_W = W.copy(); self._prev_W_t = now
                    continue

            # remember seeds
            self._last_qvars = qvars_seed
            self._last_q_cmd = q_points[0]
            self._prev_W = W.copy(); self._prev_W_t = now

            # Send as a tiny multi-point traj spanning this loop period
            self._send_followtraj_traj(q_points, total_dt=CYCLE_SECONDS)

            # light debug
            print(f"[tick] pts={len(q_points)} over {CYCLE_SECONDS:.3f}s | W={np.round(W,3)}")

# ============================ main ============================

def main():
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    rclpy.init()
    node = WatchMidpointStreamer()
    try:
        node.run()
    finally:
        node._ball_reader.stop()
        node._racket_reader.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()