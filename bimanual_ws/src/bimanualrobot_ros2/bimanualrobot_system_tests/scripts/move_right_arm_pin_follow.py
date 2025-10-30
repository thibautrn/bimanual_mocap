#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, socket, threading, math
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================ CONFIG ============================

# Wearable → robot mapping (same as your code)
SHOULDER_ANCHOR = np.array([0.045, -0.2925, 1.526], dtype=float)
L1 = 0.298511306318538     # shoulder→elbow (m)
L2 = 0.23293990641364998   # elbow→wrist_2 (m)

# UDP
UDP_PORT = 50003
PACK_FMT = "ffff fff ffff fff ffff fff ffff"  # 25 floats = 100 bytes

# Trail behavior
HISTORY_SECONDS = 1.0      # keep 1s of green trail
ANIM_FPS = 30              # refresh rate of the plot

# Plot bounds (adjust to your workspace)
X_LIM = ( -0.2,  1.0)
Y_LIM = ( -1.0,  0.4)
Z_LIM = (  1.0,  1.9)

# ============================ UDP LISTENER ============================

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

# ============================ GEOMETRY HELPERS ============================

def remap_watch_to_base(p):
    """(x,y,z)_watch -> (z, -x, y)_base"""
    x, y, z = map(float, p)
    return np.array([z, -x, y], dtype=float)

def unit(v):
    n = np.linalg.norm(v)
    return v / (n + 1e-12)

def scale_watch_to_right_robot(uarm_watch, larm_watch, hand_watch):
    """
    Map watch 3 pts (upper, lower, hand) to robot-base wrist (W).
    Also returns the elbow point E (not drawn, but available).
    """
    Sh = remap_watch_to_base(uarm_watch)
    El = remap_watch_to_base(larm_watch)
    Wr = remap_watch_to_base(hand_watch)

    # Right-arm flip (kept from your previous mapping)
    # Sh[1] = -Sh[1]; El[1] = -El[1]; Wr[1] = -Wr[1]

    u1 = unit(El - Sh)  # shoulder->elbow dir
    u2 = unit(Wr - El)  # elbow->wrist  dir

    E_robot = SHOULDER_ANCHOR + L1 * u1
    W_robot = E_robot        + L2 * u2
    return E_robot, W_robot

# ============================ LIVE PLOT ============================

def set_axes_equal(ax):
    """Make 3D axes have equal scale."""
    xlim = np.array(ax.get_xlim3d())
    ylim = np.array(ax.get_ylim3d())
    zlim = np.array(ax.get_zlim3d())
    xyz = np.array([xlim, ylim, zlim])
    spans = xyz[:,1] - xyz[:,0]
    centers = np.mean(xyz, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d([centers[0]-radius, centers[0]+radius])
    ax.set_ylim3d([centers[1]-radius, centers[1]+radius])
    ax.set_zlim3d([centers[2]-radius, centers[2]+radius])

def main():
    # start UDP thread
    th = threading.Thread(target=udp_listener, daemon=True)
    th.start()

    # wrist history: deque[(timestamp, np.array([x,y,z]))]
    wrist_hist = deque()

    # --- Matplotlib figure ---
    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(X_LIM); ax.set_ylim(Y_LIM); ax.set_zlim(Z_LIM)
    # ax.invert_yaxis()
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("Wrist live path: red = latest, green = last 1s")
    set_axes_equal(ax)

    # Artists
    (shoulder_scatter,) = ax.plot([SHOULDER_ANCHOR[0]], [SHOULDER_ANCHOR[1]], [SHOULDER_ANCHOR[2]],
                                  marker="o", markersize=6, linestyle="None", color="red", label="Shoulder anchor")
    (latest_scatter,) = ax.plot([], [], [], marker="o", markersize=6, linestyle="None", color="red", label="Wrist (latest)")
    (latest_segment,) = ax.plot([], [], [], color="red", linewidth=2, label="Shoulder→Wrist")
    (trail_line,)     = ax.plot([], [], [], color="green", linewidth=1, alpha=0.8, label="Wrist trail (1s)")
    (trail_scatter,)  = ax.plot([], [], [], marker="o", linestyle="None", markersize=3, color="green", alpha=0.7)

    ax.legend(loc="upper left")

    last_update = time.time()

    def update(_frame):
        nonlocal last_update
        t_now = time.time()

        # Copy latest sample
        with _udp_lock:
            hp = _hand_pos; lp = _larm_pos; up = _uarm_pos

        if hp is None or lp is None or up is None:
            # nothing yet; keep current frame
            return latest_scatter, latest_segment, trail_line, trail_scatter

        # Compute wrist in base frame
        _, W = scale_watch_to_right_robot(up, lp, hp)
        

        # Append to history & purge >1s
        wrist_hist.append((t_now, W.copy()))
        while wrist_hist and (t_now - wrist_hist[0][0]) > HISTORY_SECONDS:
            wrist_hist.popleft()

        # Prepare arrays for trail (green)
        if wrist_hist:
            Ws = np.array([w for (_, w) in wrist_hist])
        else:
            Ws = np.empty((0,3))

        # Update artists:
        # latest wrist (red dot)
        latest_scatter.set_data([W[0]], [W[1]])
        latest_scatter.set_3d_properties([W[2]])

        # shoulder→latest wrist (red segment)
        latest_segment.set_data([SHOULDER_ANCHOR[0], W[0]],
                                [SHOULDER_ANCHOR[1], W[1]])
        latest_segment.set_3d_properties([SHOULDER_ANCHOR[2], W[2]])

        # trail (green line + points)
        if len(Ws) >= 2:
            trail_line.set_data(Ws[:,0], Ws[:,1])
            trail_line.set_3d_properties(Ws[:,2])
            trail_scatter.set_data(Ws[:,0], Ws[:,1])
            trail_scatter.set_3d_properties(Ws[:,2])
        else:
            trail_line.set_data([], [])
            trail_line.set_3d_properties([])
            trail_scatter.set_data([], [])
            trail_scatter.set_3d_properties([])

        return latest_scatter, latest_segment, trail_line, trail_scatter

    ani = FuncAnimation(fig, update, interval=1000/ANIM_FPS, blit=False)
    plt.show(block=True)

if __name__ == "__main__":
    main()
