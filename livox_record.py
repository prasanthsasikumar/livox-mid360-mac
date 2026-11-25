#!/usr/bin/env python3
"""Record raw Mid-360 point and motion-sensor packets to a file while you walk.

    python3 livox_record.py              # records until you press Ctrl+C
    python3 livox_record.py --seconds 60

Nothing is processed here: every packet is saved exactly as the lidar sent it,
so the same recording can be replayed through any stitching software later.

File format (recordings/walk-<time>.lvraw), little-endian:
    8 bytes   magic "LVRAW001"
    then per packet:
        double  time received (seconds since 1970, this computer's clock)
        uint8   kind: 1 = points, 2 = motion sensor
        uint16  packet length
        bytes   the UDP payload (36-byte Livox header + data; the header holds
                the lidar's own nanosecond timestamp)
"""
import argparse
import os
import select
import struct
import sys
import time

import livox_probe as lp

MAGIC = b"LVRAW001"
RECORD = struct.Struct("<dBH")
KIND_POINTS, KIND_IMU = 1, 2


def read_packets(path):
    """Yield (received_time, kind, payload) for every packet in a recording."""
    with open(path, "rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise ValueError("%s is not a Livox raw recording" % path)
        while True:
            head = f.read(RECORD.size)
            if len(head) < RECORD.size:
                return
            received, kind, length = RECORD.unpack(head)
            payload = f.read(length)
            if len(payload) < length:
                return  # recording was cut off mid-packet
            yield received, kind, payload


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lidar", default="192.168.1.119")
    ap.add_argument("--host", default="192.168.1.50")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (default: until Ctrl+C)")
    ap.add_argument("--still", type=float, default=5.0, help="seconds to stand still at the start")
    args = ap.parse_args()

    cmd = lp.bind(args.host, lp.HOST_CMD_PORT)
    points = lp.bind(args.host, lp.HOST_POINT_PORT)
    imu = lp.bind(args.host, lp.HOST_IMU_PORT)
    targets = lp.key_values([
        (lp.KEY_POINT_HOST, lp.host_target(args.host, lp.HOST_POINT_PORT, lp.LIDAR_POINT_PORT)),
        (lp.KEY_IMU_HOST, lp.host_target(args.host, lp.HOST_IMU_PORT, lp.LIDAR_IMU_PORT)),
    ])
    ok = lp.send_command(cmd, args.lidar, 1, targets, "set data destinations")
    ok = ok and lp.send_command(cmd, args.lidar, 2, lp.key_values([(lp.KEY_WORK_MODE, bytes([lp.MODE_NORMAL]))]),
                                "start scanning")
    if not ok:
        return 1
    if not select.select([points], [], [], 15)[0]:
        print("No points arrived within 15 s.")
        return 1

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "walk-%s.lvraw" % time.strftime("%Y%m%d-%H%M%S"))

    n_points = n_imu = n_bytes = 0
    last_count = None   # (kind -> last udp counter) to spot dropped packets
    dropped = 0
    last_seen = {}
    start = time.time()
    next_status = start
    print("Recording to %s" % path)
    print("Press Ctrl+C to stop.")
    try:
        with open(path, "wb", buffering=1024 * 1024) as f:
            f.write(MAGIC)
            while not args.seconds or time.time() - start < args.seconds:
                for sock in select.select([points, imu], [], [], 0.2)[0]:
                    data = sock.recv(2048)
                    now = time.time()
                    kind = KIND_POINTS if sock is points else KIND_IMU
                    f.write(RECORD.pack(now, kind, len(data)) + data)
                    n_bytes += len(data)
                    if len(data) >= lp.POINT_HEADER.size:
                        dot_num, udp_cnt = struct.unpack_from("<HH", data, 5)
                        if kind == KIND_POINTS:
                            n_points += dot_num
                            if kind in last_seen:
                                gap = (udp_cnt - last_seen[kind] - 1) & 0xFFFF
                                if gap < 0x8000:
                                    dropped += gap
                            last_seen[kind] = udp_cnt
                        else:
                            n_imu += 1
                now = time.time()
                if now >= next_status:
                    elapsed = now - start
                    phase = ("STAND STILL %2.0f" % (args.still - elapsed + 1)) if elapsed < args.still else "WALK           "
                    sys.stdout.write("\r%s  %5.0f s   %6.1f M points   %6d motion samples   %5.0f MB   %d dropped  " % (
                        phase, elapsed, n_points / 1e6, n_imu, n_bytes / 1e6, dropped))
                    sys.stdout.flush()
                    next_status = now + 0.5
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    print()
    print("Saved %.0f s: %.1f million points, %d motion samples, %.0f MB, %d point packets dropped" % (
        elapsed, n_points / 1e6, n_imu, n_bytes / 1e6, dropped))
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
