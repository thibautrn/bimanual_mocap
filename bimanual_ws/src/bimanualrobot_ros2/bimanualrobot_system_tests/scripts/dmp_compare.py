#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ================== CONFIG ==================

# Path to wrist positions log (format: t x y z)

# Paths to DMP baseline + weights (same ones used in your DMP code)
LOG_DIR = Path("src/bimanualrobot_ros2/bimanualrobot_system_tests/scripts/logs")  # or set to your logs/good directory
BASELINE_PATH = LOG_DIR / "baseline2.npz"
WEIGHTS_PATH = LOG_DIR / "wrist_20251103_004433_weights.npz"
WRIST_FILE = LOG_DIR / "wrist_20251103_004433.txt"
DMP_DT = 1.0 / 40.0          # matches DT_STEP / DMP_DT in your control code
TAU_OVERRIDE = None          # keep None to use tau from baseline.npz
USE_WEIGHTED_OFFSET_GOAL = True
ADDITIONAL_DZ = 0.0

# ================== LOAD WRIST POS ==================

def load_wrist_positions(path: Path):
    """
    Expects: columns [t_sec, x, y, z], possibly with a header starting with '#'.
    Returns (N, 3) array of [x, y, z].
    """
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        # Single line case
        data = data.reshape(1, -1)
    # use columns 1,2,3 for x,y,z
    return data[:, 1:4]

# ================== DMP CORE (from your logic) ==================

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
        psi = np.exp(-h * (s - c) ** 2)
        Phi[k, :] = (psi / (psi.sum() + 1e-12)) * s
    return Phi

def rollout_dmp_3d(y0, g, W, K, D, tau, dt, c, h, alpha_s):
    """
    3D DMP rollout exactly following your DMPRolloutSource + rollout_dmp_3d logic.
    y0, g : (3,)
    W     : (3, n_basis)
    Returns: Y shape (N, 3)
    """
    N = int(np.round(tau / dt)) + 1
    S = canonical_by_steps(N, dt, tau, alpha_s)
    Phi = design_matrix(S, c, h)

    Y = np.zeros((N, 3), float)
    Yd = np.zeros_like(Y)
    Y[0] = y0

    for k in range(N - 1):
        acc = np.zeros(3, float)
        for d in range(3):
            f = float(Phi[k].dot(W[d]))
            acc[d] = (
                K * (g[d] - Y[k, d])
                - D * tau * Yd[k, d]
                + K * f * (g[d] - y0[d])
            ) / (tau ** 2)
        Yd[k + 1] = Yd[k] + acc * dt
        Y[k + 1] = Y[k] + Yd[k + 1] * dt

    return Y

def compute_dmp_trajectory(
    baseline_path: Path,
    weights_path: Path,
    dt: float,
    tau_override=None,
    use_weighted_offset_goal=True,
    add_dz: float = 0.0,
):
    """
    Rebuilds DMPRolloutSource behavior from your code (no ROS).
    Loads baseline.npz and generated_dmp_weights.npz and outputs (N,3) DMP positions.
    """
    # Baseline params
    b = np.load(baseline_path)
    c = b["c"]
    h = b["h"]
    K = float(b["K"])
    D = float(b["D"])
    alpha_s = float(b["alpha_s"])
    tau_file = float(b["tau"])
    tau = float(tau_override) if tau_override is not None else tau_file

    # Weights + endpoints
    w = np.load(weights_path, allow_pickle=True)
    W = w["w"]          # shape (3, n_basis)
    y0_w = w["y0"].astype(float)
    g_w = w["g"].astype(float)

    # Start at recorded y0
    y0 = y0_w.copy()

    # Goal logic: optionally use weighted offset goal like in DMPRolloutSource
    if use_weighted_offset_goal:
        g = y0 + (g_w - y0_w)
    else:
        g = g_w.copy()

    # Optional Z offset
    g = g + np.array([0.0, 0.0, float(add_dz)], float)

    # Rollout
    Y = rollout_dmp_3d(y0, g, W, K, D, tau, dt, c, h, alpha_s)
    return Y

# ================== PLOTTING ==================

def plot_wrist_points_3d(wrist_xyz):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(wrist_xyz[:, 0], wrist_xyz[:, 1], wrist_xyz[:, 2], s=5)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Wrist positions (3D points)")
    ax.view_init(elev=25, azim=-60)  # tweak viewing angle if you like
    return fig, ax

def plot_dmp_trajectory_3d(dmp_xyz):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(dmp_xyz[:, 0], dmp_xyz[:, 1], dmp_xyz[:, 2])
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("DMP rollout trajectory")
    ax.view_init(elev=25, azim=-60)
    return fig, ax

# ================== MAIN ==================

def main():
    # 1) Load and plot wrist positions
    wrist_xyz = load_wrist_positions(WRIST_FILE)
    plot_wrist_points_3d(wrist_xyz)

    # 2) Compute and plot DMP trajectory from your baseline + weights
    dmp_xyz = compute_dmp_trajectory(
        BASELINE_PATH,
        WEIGHTS_PATH,
        dt=DMP_DT,
        tau_override=TAU_OVERRIDE,
        use_weighted_offset_goal=USE_WEIGHTED_OFFSET_GOAL,
        add_dz=ADDITIONAL_DZ,
    )
    plot_dmp_trajectory_3d(dmp_xyz)

    # Show both figures
    plt.show()

if __name__ == "__main__":
    main()
