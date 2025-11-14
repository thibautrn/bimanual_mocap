#!/usr/bin/env python3
import math
import time
import subprocess
import re
import threading
import argparse
from pathlib import Path

import numpy as np


# ---------------------- ARGS ---------------------- #

def parse_args():
    ap = argparse.ArgumentParser(
        description=(
            "Soft spherical tether for a Gazebo ball link.\n"
            "Inside radius L: free.\n"
            "Outside radius L: spring+damper pulls back toward anchor."
        )
    )
    ap.add_argument(
        '--world', default='default',
        help='Gazebo world name (for /world/<world>/pose/info and /world/<world>/wrench).'
    )
    ap.add_argument(
        '--ball-link', required=True,
        help='Scoped LINK name to apply wrench to, e.g. rope_square_rig::end_ball'
    )
    ap.add_argument(
        '--ball-name-regex', default=r'^(?:rope_square_rig)$',
        help='Regex for the entity name in /pose/info to TRACK (model or link).'
    )
    ap.add_argument(
        '--anchor', nargs=3, type=float, default=[0.0, 0.0, 0.0],
        metavar=('X', 'Y', 'Z'),
        help='Center of spherical tether region (rope attach point).'
    )
    ap.add_argument(
        '--L', type=float, required=True,
        help='Sphere radius / slack length (m). Free inside; tether outside.'
    )
    ap.add_argument(
        '--K', type=float, default=60.0,
        help='Spring stiffness (N/m) when outside radius.'
    )
    ap.add_argument(
        '--mass', type=float, default=0.20,
        help='Ball mass (kg); used for default damping and warmup support.'
    )
    ap.add_argument(
        '--C', type=float, default=None,
        help='Damping (N*s/m); default 2*sqrt(m*K) on radial component.'
    )
    ap.add_argument(
        '--rate', type=float, default=240.0,
        help='Control loop rate (Hz).'
    )
    ap.add_argument(
        '--ready-file', default='/tmp/tether_ready',
        help='Touch this file once pose is locked and tether is active.'
    )
    ap.add_argument(
        '--warmup', type=float, default=0.30,
        help='Seconds after first pose where we hold ~mg upward.'
    )
    ap.add_argument(
        '--log-interval', type=float, default=0.25,
        help='Seconds between pose log lines.'
    )
    return ap.parse_args()


# ---------------------- WRENCH PUBLISH ---------------------- #

def publish_wrench(world: str, link_scoped_name: str, Fx: float, Fy: float, Fz: float):
    """Publish a one-shot wrench on /world/<world>/wrench for the given LINK."""
    payload = (
        f'entity: {{name: "{link_scoped_name}", type: LINK}}, '
        f'wrench: {{force: {{x: {Fx:.6f}, y: {Fy:.6f}, z: {Fz:.6f}}}}}'
    )
    subprocess.run(
        [
            "gz", "topic",
            "-t", f"/world/{world}/wrench",
            "-m", "gz.msgs.EntityWrench",
            "-p", payload,
        ],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )


def clear_persistent_wrench(world: str, link_scoped_name: str):
    """Best-effort clear of any persistent wrench on that link."""
    payload = (
        f'entity: {{name: "{link_scoped_name}", type: LINK}}, '
        f'wrench: {{force: {{x: 0.0, y: 0.0, z: 0.0}}}}'
    )
    subprocess.run(
        [
            "gz", "topic",
            "-t", f"/world/{world}/wrench/persistent",
            "-m", "gz.msgs.EntityWrench",
            "-p", payload,
        ],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )


# ---------------------- POSE READER ---------------------- #

class GzPoseReader:
    """
    Stream parser for:
      gz topic -e -t /world/<world>/pose/info

    Tracks the last pose of any entity whose name matches target_regex.
    """

    def __init__(self, world: str, target_regex: str, verbose: bool = False):
        self.topic = f"/world/{world}/pose/info"
        self.target = re.compile(target_regex)
        self.verbose = bool(verbose)

        self.have = False      # at least one pose sample
        self.found = False     # matched the entity name at least once
        self.xyz = np.zeros(3)
        self.vel = np.zeros(3)

        self._last_t = None
        self._last_xyz = np.zeros(3)

        self._proc = None
        self._stop = threading.Event()

    def start(self):
        self._proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", self.topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()
        if self._proc and self._proc.poll() is None:
            try:
                self._proc.terminate()
            except Exception:
                pass

    def _commit(self, x: float, y: float, z: float):
        now = time.monotonic()

        px = self.xyz[0] if x is None else x
        py = self.xyz[1] if y is None else y
        pz = self.xyz[2] if z is None else z

        if self._last_t is not None:
            dt = max(now - self._last_t, 1e-6)
            self.vel[:] = [
                (px - self._last_xyz[0]) / dt,
                (py - self._last_xyz[1]) / dt,
                (pz - self._last_xyz[2]) / dt,
            ]

        self.xyz[:] = (px, py, pz)
        self._last_xyz[:] = self.xyz
        self._last_t = now
        self.have = True

    def _run(self):
        in_pose = False
        name_match = False
        bx = by = bz = None

        rx_name = re.compile(r'name:\s+"([^"]+)"')
        rx_posblk = re.compile(r'position\s*\{([^}]*)\}')
        rx_x = re.compile(r'\bx:\s*([-\d.e+]+)')
        rx_y = re.compile(r'\by:\s*([-\d.e+]+)')
        rx_z = re.compile(r'\bz:\s*([-\d.e+]+)')

        last_print = time.monotonic()

        for ln in self._proc.stdout:
            if self._stop.is_set():
                break

            s = ln.lstrip()

            # Start of pose block
            if not in_pose and s.startswith("pose {"):
                in_pose = True
                name_match = False
                bx = by = bz = None
                continue

            if not in_pose:
                continue

            # name: "..."
            mname = rx_name.search(ln)
            if mname:
                nm = mname.group(1)
                name_match = bool(self.target.fullmatch(nm))
                if name_match and not self.found:
                    print(f"[tether] tracking entity pose of: {nm}")
                    self.found = True
                continue

            # Inline position { ... }
            mblk = rx_posblk.search(ln)
            if mblk:
                blk = mblk.group(1)
                mx = rx_x.search(blk)
                my = rx_y.search(blk)
                mz = rx_z.search(blk)
                if mx:
                    bx = float(mx.group(1))
                if my:
                    by = float(my.group(1))
                if mz:
                    bz = float(mz.group(1))
                continue

            # x / y / z maybe on separate lines
            m = rx_x.search(ln)
            if m:
                bx = float(m.group(1))
            m = rx_y.search(ln)
            if m:
                by = float(m.group(1))
            m = rx_z.search(ln)
            if m:
                bz = float(m.group(1))

            # End of pose block
            if "}" in ln:
                if name_match and bx is not None and by is not None and bz is not None:
                    self._commit(bx, by, bz)
                    if self.verbose and (time.monotonic() - last_print) > 0.25:
                        print(
                            f"[tether] pos=({self.xyz[0]:.3f},{self.xyz[1]:.3f},{self.xyz[2]:.3f}) "
                            f"vel=({self.vel[0]:.2f},{self.vel[1]:.2f},{self.vel[2]:.2f})"
                        )
                        last_print = time.monotonic()
                in_pose = False
                name_match = False
                bx = by = bz = None

    def get(self):
        return bool(self.have), self.xyz.copy(), self.vel.copy()


# ---------------------- MAIN LOOP ---------------------- #

def main():
    args = parse_args()

    anchor = np.array(args.anchor, dtype=float)
    L = float(args.L)
    K = float(args.K)
    m = float(args.mass)
    C = float(args.C) if args.C is not None else 2.0 * math.sqrt(m * K)
    dt = 1.0 / float(args.rate)
    g = 9.81

    print(
        f"[tether] params: L={L:.3f} K={K:.1f} C={C:.2f} m={m:.2f} "
        f"rate={1.0/dt:.0f}Hz anchor=({anchor[0]:.3f},{anchor[1]:.3f},{anchor[2]:.3f})"
    )
    print(f"[tether] ball_link={args.ball_link}  track_regex={args.ball_name_regex}")

    reader = GzPoseReader(args.world, args.ball_name_regex, verbose=False)
    reader.start()

    print("[tether] waiting for entity name match in /pose/info...")
    while not reader.found:
        time.sleep(0.02)

    print("[tether] entity matched. waiting for first pose sample...")
    have_pose = False
    t_lock = None
    last_log = 0.0
    last_force_log = 0.0

    while True:
        have, p, v = reader.get()

        if have and not have_pose:
            have_pose = True
            t_lock = time.monotonic()
            print("[tether] pose locked; tether active.")
            print(f"[tether] first pose: ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
            print(f"[tether] wrench LINK: {args.ball_link}")  # <-- see note below

            clear_persistent_wrench(args.world, args.ball_link)

            if args.ready_file:
                try:
                    Path(args.ready_file).touch(exist_ok=True)
                except Exception as e:
                    print(f"[tether] WARN: could not touch ready_file: {e}")

        if not have_pose:
            time.sleep(dt)
            continue

        # Warmup: support ~mg so the ball doesn't just drop while things settle
        if (time.monotonic() - t_lock) < args.warmup:
            publish_wrench(args.world, args.ball_link, 0.0, 0.0, m * g)
            time.sleep(dt)
            continue

        # Core tether: compute distance from anchor
        r = p - anchor
        d = float(np.linalg.norm(r))
        now = time.monotonic()

        # Pose logging
        if now - last_log > args.log_interval:
            print(
                f"[tether] pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) "
                f"vel=({v[0]:.2f},{v[1]:.2f},{v[2]:.2f}) d={d:.3f}"
            )
            last_log = now

                # Always act like a spring with rest length L
        if d > 1e-6:
            u = r / d
            v_radial = float(np.dot(v, u))

            # Spring with rest length L:
            #  - if d > L  -> F pulls toward anchor
            #  - if d < L  -> F pushes away from anchor
            Fmag = K * (d - L) - C * v_radial

            Fx = -Fmag * u[0]
            Fy = -Fmag * u[1]
            Fz = -Fmag * u[2]

            publish_wrench(args.world, args.ball_link, Fx, Fy, Fz)


        time.sleep(dt)


if __name__ == "__main__":
    main()
