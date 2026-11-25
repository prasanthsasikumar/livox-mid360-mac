#!/usr/bin/env python3
"""First-light test for a Livox Mid-360: tell it where to send data, start it,
and count what comes back.

Protocol details (frame layout, keys, ports) are taken from Livox-SDK2:
sdk_core/comm/sdk_protocol.cpp, sdk_core/comm/define.h, include/livox_lidar_def.h.

    python3 livox_probe.py                 # 5 second capture
    python3 livox_probe.py --seconds 10
    python3 livox_probe.py --standby       # stop the motor again afterwards
"""
import argparse
import select
import socket
import struct
import sys
import time
import zlib

LIDAR_CMD_PORT = 56100
HOST_CMD_PORT, HOST_PUSH_PORT, HOST_POINT_PORT, HOST_IMU_PORT = 56101, 56201, 56301, 56401
LIDAR_PUSH_PORT, LIDAR_POINT_PORT, LIDAR_IMU_PORT = 56200, 56300, 56400

CMD_PARAM_SET = 0x0100  # kCommandIDLidarWorkModeControl: key/value parameter write
KEY_STATE_HOST, KEY_POINT_HOST, KEY_IMU_HOST, KEY_WORK_MODE = 0x0005, 0x0006, 0x0007, 0x001A
MODE_NORMAL, MODE_STANDBY = 0x01, 0x02  # kLivoxLidarNormal, kLivoxLidarWakeUp

POINT_HEADER = struct.Struct("<BHHHHBBB12sI8s")  # 36 bytes, then the points
# data_type -> (name, bytes per point)
POINT_TYPES = {0: ("imu", 24), 1: ("xyz 32-bit mm", 14), 2: ("xyz 16-bit cm", 8), 3: ("spherical", 10)}


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(seq, cmd_id, payload):
    head = struct.pack("<BBHIHBB6s", 0xAA, 0, 24 + len(payload), seq & 0xFFFF, cmd_id, 0, 0, b"")
    crc32 = zlib.crc32(payload) & 0xFFFFFFFF if payload else 0
    return head + struct.pack("<HI", crc16_ccitt_false(head), crc32) + payload


def key_values(pairs):
    out = struct.pack("<HH", len(pairs), 0)
    for key, value in pairs:
        out += struct.pack("<HH", key, len(value)) + value
    return out


def host_target(host_ip, host_port, lidar_port):
    return socket.inet_aton(host_ip) + struct.pack("<HH", host_port, lidar_port)


def bind(host_ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.bind((host_ip, port))
    return sock


def send_command(sock, lidar_ip, seq, payload, what):
    """Send one parameter-write and wait for its ack. Returns True if accepted."""
    frame = build_frame(seq, CMD_PARAM_SET, payload)
    for attempt in range(3):
        sock.sendto(frame, (lidar_ip, LIDAR_CMD_PORT))
        deadline = time.time() + 1.0
        while time.time() < deadline:
            if not select.select([sock], [], [], max(0, deadline - time.time()))[0]:
                break
            data, _ = sock.recvfrom(2048)
            if len(data) < 27 or data[0] != 0xAA:
                continue
            _, _, _, ack_seq, cmd_id, cmd_type, _ = struct.unpack_from("<BBHIHBB", data)
            if cmd_id != CMD_PARAM_SET or cmd_type != 1 or ack_seq != (seq & 0xFFFF):
                continue
            ret_code, error_key = struct.unpack_from("<BH", data, 24)
            if ret_code == 0:
                print("  ok    %s" % what)
                return True
            print("  FAIL  %s: lidar returned code 0x%02x on key 0x%04x" % (what, ret_code, error_key))
            return False
    print("  FAIL  %s: no reply from %s:%d after 3 tries" % (what, lidar_ip, LIDAR_CMD_PORT))
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lidar", default="192.168.1.119")
    ap.add_argument("--host", default="192.168.1.50", help="this computer's address on the lidar cable")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--standby", action="store_true", help="put the lidar back in standby when done")
    args = ap.parse_args()

    cmd = bind(args.host, HOST_CMD_PORT)
    push = bind(args.host, HOST_PUSH_PORT)
    points = bind(args.host, HOST_POINT_PORT)
    imu = bind(args.host, HOST_IMU_PORT)

    print("Configuring %s to send to %s" % (args.lidar, args.host))
    targets = key_values([
        (KEY_STATE_HOST, host_target(args.host, HOST_PUSH_PORT, LIDAR_PUSH_PORT)),
        (KEY_POINT_HOST, host_target(args.host, HOST_POINT_PORT, LIDAR_POINT_PORT)),
        (KEY_IMU_HOST, host_target(args.host, HOST_IMU_PORT, LIDAR_IMU_PORT)),
    ])
    if not send_command(cmd, args.lidar, 1, targets, "set data destinations"):
        return 1
    if not send_command(cmd, args.lidar, 2, key_values([(KEY_WORK_MODE, bytes([MODE_NORMAL]))]), "start scanning"):
        return 1

    print("Listening for %.0f s ..." % args.seconds)
    stats = {"point_pkts": 0, "points": 0, "valid": 0, "imu_pkts": 0, "push_pkts": 0, "bytes": 0}
    types_seen = {}
    ranges = []
    first_point_at = None
    last_imu = None
    start = time.time()
    end = start + args.seconds
    while True:
        remaining = end - time.time()
        if remaining <= 0:
            break
        for sock in select.select([points, imu, push], [], [], min(remaining, 0.5))[0]:
            data, _ = sock.recvfrom(2048)
            stats["bytes"] += len(data)
            if sock is push:
                stats["push_pkts"] += 1
                continue
            if len(data) < POINT_HEADER.size:
                continue
            _, _, _, dot_num, _, _, data_type, _, _, _, _ = POINT_HEADER.unpack_from(data)
            body = data[POINT_HEADER.size:]
            if sock is imu:
                stats["imu_pkts"] += 1
                if len(body) >= 24:
                    last_imu = struct.unpack_from("<6f", body)
                continue
            if first_point_at is None:
                first_point_at = time.time() - start
            stats["point_pkts"] += 1
            stats["points"] += dot_num
            types_seen[data_type] = types_seen.get(data_type, 0) + 1
            if data_type in (1, 2):
                fmt, scale = ("<iiiBB", 0.001) if data_type == 1 else ("<hhhBB", 0.01)
                size = struct.calcsize(fmt)
                for i in range(min(dot_num, len(body) // size)):
                    x, y, z, _, _ = struct.unpack_from(fmt, body, i * size)
                    if x or y or z:
                        stats["valid"] += 1
                        if i % 8 == 0:  # sample; enough for a range summary
                            ranges.append((x * x + y * y + z * z) ** 0.5 * scale)

    elapsed = time.time() - start
    print()
    print("Point packets : %d  (%.0f/s)" % (stats["point_pkts"], stats["point_pkts"] / elapsed))
    print("Points        : %d  (%.0f/s), %d with a real return" % (
        stats["points"], stats["points"] / elapsed, stats["valid"]))
    for data_type, count in sorted(types_seen.items()):
        print("  format      : %s (%d packets)" % (POINT_TYPES.get(data_type, ("type %d" % data_type, 0))[0], count))
    if first_point_at is not None:
        print("  first packet: %.2f s after start command" % first_point_at)
    if ranges:
        ranges.sort()
        print("  distance    : nearest %.2f m, median %.2f m, farthest %.2f m" % (
            ranges[0], ranges[len(ranges) // 2], ranges[-1]))
    print("IMU packets   : %d  (%.0f/s)" % (stats["imu_pkts"], stats["imu_pkts"] / elapsed))
    if last_imu:
        gx, gy, gz, ax, ay, az = last_imu
        print("  last sample : gyro (%.3f, %.3f, %.3f) rad/s, accel (%.3f, %.3f, %.3f) g, |accel| %.3f" % (
            gx, gy, gz, ax, ay, az, (ax * ax + ay * ay + az * az) ** 0.5))
    print("Status packets: %d" % stats["push_pkts"])
    print("Data rate     : %.2f Mbit/s" % (stats["bytes"] * 8 / elapsed / 1e6))

    if args.standby:
        send_command(cmd, args.lidar, 3, key_values([(KEY_WORK_MODE, bytes([MODE_STANDBY]))]), "back to standby")

    return 0 if stats["point_pkts"] else 2


if __name__ == "__main__":
    sys.exit(main())
