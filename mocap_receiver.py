#!/usr/bin/env python3
"""
mocap_receiver.py  (flexible — works with any sensor count)
============================================================
Run on the PC to verify the UDP stream before hooking into Unreal Engine 5.

Usage:
    python mocap_receiver.py [--port 12345]

Packet format (flexible, set by hub):
    Bytes 0-4  : ASCII "MOCAP"
    Byte  5    : uint8  N  — number of active bones
    Bytes 6…   : N × MocapBone  (17 bytes each)
        byte 0      : uint8  boneId
        bytes 1-4   : float  qi  (little-endian)
        bytes 5-8   : float  qj
        bytes 9-12  : float  qk
        bytes 13-16 : float  qr  (real / w)

Examples:
    2 sensors  →  5 + 1 + 2×17 = 40 bytes
    9 sensors  →  5 + 1 + 9×17 = 159 bytes
"""

import socket
import struct
import argparse
import math
import time

# ── Bone names (add/change freely — only for display) ─────────────────────────
BONE_NAMES = {
    0: "TORSO      ",
    1: "L_UPPER_ARM",
    2: "L_FOREARM  ",
    3: "R_UPPER_ARM",
    4: "R_FOREARM  ",
    5: "L_THIGH    ",
    6: "L_CALF     ",
    7: "R_THIGH    ",
    8: "R_CALF     ",
}

HEADER    = b"MOCAP"
BONE_FMT  = "<Bffff"                     # uint8 + 4 floats, little-endian
BONE_SIZE = struct.calcsize(BONE_FMT)    # 17 bytes


def quat_to_euler_deg(qi, qj, qk, qr):
    """ZYX Euler angles (degrees) from quaternion (i, j, k, real)."""
    sinr = 2.0 * (qr * qi + qj * qk)
    cosr = 1.0 - 2.0 * (qi * qi + qj * qj)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (qr * qj - qk * qi)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

    siny = 2.0 * (qr * qk + qi * qj)
    cosy = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw


def parse_packet(data: bytes):
    """
    Parse a flexible MOCAP UDP packet.
    Returns a list of (boneId, qi, qj, qk, qr) tuples, or None on error.
    """
    if len(data) < 6:
        return None
    if data[:5] != HEADER:
        return None

    n_bones = data[5]
    expected = 6 + n_bones * BONE_SIZE
    if len(data) != expected:
        return None

    bones = []
    offset = 6
    for _ in range(n_bones):
        bid, qi, qj, qk, qr = struct.unpack_from(BONE_FMT, data, offset)
        bones.append((bid, qi, qj, qk, qr))
        offset += BONE_SIZE
    return bones


def main():
    parser = argparse.ArgumentParser(
        description="Receive and display flexible MOCAP UDP stream from ESP32 hub"
    )
    parser.add_argument("--port", type=int, default=12345, help="UDP listen port")
    parser.add_argument("--hz",   action="store_true",
                        help="Show approximate packet rate instead of per-packet dump")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))
    print(f"Listening on UDP port {args.port}  (flexible packet size)")
    print("Press Ctrl+C to stop.\n")

    pkt_count  = 0
    rate_count = 0
    t_start    = time.time()

    try:
        while True:
            data, addr = sock.recvfrom(512)
            bones = parse_packet(data)

            if bones is None:
                print(f"[WARN] Bad packet ({len(data)} bytes) from {addr}")
                continue

            pkt_count  += 1
            rate_count += 1

            if args.hz:
                # Rate mode: print Hz every second, not every packet
                now = time.time()
                if now - t_start >= 1.0:
                    hz = rate_count / (now - t_start)
                    print(f"\r{len(bones)} bones @ {hz:.1f} Hz   (packet #{pkt_count})",
                          end="", flush=True)
                    rate_count = 0
                    t_start    = now
            else:
                # Verbose mode: print every packet
                print(f"\n── Packet #{pkt_count}  ({len(bones)} bones)  from {addr[0]} ───")
                for bid, qi, qj, qk, qr in bones:
                    roll, pitch, yaw = quat_to_euler_deg(qi, qj, qk, qr)
                    name = BONE_NAMES.get(bid, f"BONE_{bid}  ")
                    print(f"  {name}  "
                          f"q=({qi:+.3f},{qj:+.3f},{qk:+.3f},{qr:+.3f})  "
                          f"R{roll:+6.1f}° P{pitch:+6.1f}° Y{yaw:+6.1f}°")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
