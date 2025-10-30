#!/usr/bin/env python3
import socket, struct, time, csv, os
from pathlib import Path

UDP_PORT  = 50003
PACK_FMT  = "ffff fff ffff fff ffff fff ffff"   # same as your sender (25 floats)
CSV_PATH  = Path("/home/thibaut/Documents/Bimanual_Robot/logs/udp_samples.csv")

# Make sure folder exists
CSV_PATH.parent.mkdir(parents=True, exist_ok=True)

# Prepare socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))
unpack = struct.Struct(PACK_FMT).unpack_from

# Write header if file doesn’t exist
write_header = not CSV_PATH.exists()
f = open(CSV_PATH, "a", newline="")
w = csv.writer(f)
if write_header:
    w.writerow([
        "ts",
        "hand_px","hand_py","hand_pz",
        "larm_px","larm_py","larm_pz",
        "uarm_px","uarm_py","uarm_pz",
        "hand_qw","hand_qx","hand_qy","hand_qz",
        "larm_qw","larm_qx","larm_qy","larm_qz",
        "uarm_qw","uarm_qx","uarm_qy","uarm_qz",
        "hips_qw","hips_qx","hips_qy","hips_qz",
    ])

print(f"[UDP] logging on port {UDP_PORT} → {CSV_PATH}")
try:
    while True:
        data, _ = sock.recvfrom(1024)
        if len(data) < 100:    # need at least 25 floats (100 bytes)
            continue

        (hw, hx, hy, hz,
         hpx, hpy, hpz,
         lw, lx, ly, lz,
         lpx, lpy, lpz,
         uw, ux, uy, uz,
         upx, upy, upz,
         qw, qx, qy, qz) = unpack(data)

        ts = time.time()
        w.writerow([
            f"{ts:.6f}",
            hpx,hpy,hpz,
            lpx,lpy,lpz,
            upx,upy,upz,
            hw,hx,hy,hz,
            lw,lx,ly,lz,
            uw,ux,uy,uz,
            qw,qx,qy,qz
        ])
        f.flush()  # (optional) ensure it’s on disk

except KeyboardInterrupt:
    print("\n[UDP] stopped.")
finally:
    f.close()
    sock.close()