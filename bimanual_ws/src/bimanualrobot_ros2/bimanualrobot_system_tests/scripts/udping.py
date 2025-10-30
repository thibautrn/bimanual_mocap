#!/usr/bin/env python3
import argparse, socket, struct, sys

# Packet layout (bytes)
# 0..15   : hand_rot (w,x,y,z)   -> 4 floats
# 16..27  : hand_pos (x,y,z)     -> 3 floats
# 28..43  : larm_rot (w,x,y,z)   -> 4 floats
# 44..55  : larm_pos (x,y,z)     -> 3 floats
# 56..71  : uarm_rot (w,x,y,z)   -> 4 floats
# 72..83  : uarm_pos (x,y,z)     -> 3 floats
# 84..99  : hips_rot (w,x,y,z)   -> 4 floats
# (>=100B means optional tail data)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True, help="UDP port to listen on (must match config.PORT_PUB_LEFT_ARM)")
    ap.add_argument("--endian", choices=["<", "!", ">"], default="<",
                    help="Struct endianness: '<' little (default), '!' network(big), '>' big")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"[listener] listening on 0.0.0.0:{args.port}  (endianness={args.endian!r})")
    print("[listener] now send from your Mac using your VM's IP as the destination.")

    while True:
        data, addr = sock.recvfrom(4096)
        print(f"\n[listener] got {len(data)} bytes from {addr}")
        if len(data) < 100:
            print("  packet too short for pose (need >= 100 bytes)"); continue

        e = args.endian
        try:
            hand_rot = struct.unpack(f'{e}ffff', data[0:16])
            hand_pos = struct.unpack(f'{e}fff',  data[16:28])
            larm_rot = struct.unpack(f'{e}ffff', data[28:44])
            larm_pos = struct.unpack(f'{e}fff',  data[44:56])
            uarm_rot = struct.unpack(f'{e}ffff', data[56:72])
            uarm_pos = struct.unpack(f'{e}fff',  data[72:84])
            hips_rot = struct.unpack(f'{e}ffff', data[84:100])
        except struct.error as se:
            print(f"  unpack error: {se}"); continue

        print(f"  hand_rot(wxyz) = {hand_rot}")
        print(f"  hand_pos(xyz)  = {hand_pos}")
        print(f"  larm_rot(wxyz) = {larm_rot}")
        print(f"  larm_pos(xyz)  = {larm_pos}")
        print(f"  uarm_rot(wxyz) = {uarm_rot}")
        print(f"  uarm_pos(xyz)  = {uarm_pos}")
        print(f"  hips_rot(wxyz) = {hips_rot}")

        if len(data) > 100:
            print(f"  tail bytes     = {len(data)-100}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
