#!/usr/bin/env python3
"""Record raw Mid-360 point and motion-sensor packets to a file while you walk.

    python3 livox_record.py              # records until you press Ctrl+C
    python3 livox_record.py --seconds 60
    python3 livox_record.py --seconds 60 --phone     # also record on the iPhone (LidarCam app)

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

import json

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
    ap.add_argument("--phone", action="store_true", help="start and stop the LidarCam iPhone app and fetch its video")
    ap.add_argument("--lens", choices=["wide", "ultrawide"], default="ultrawide",
                    help="phone lens: ultrawide (0.5x, default: sees most, sharp at every distance) or wide (1x, more detail)")
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
    name = "walk-%s" % time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(out_dir, name + ".lvraw")

    phone, phone_info = None, None
    if args.phone:
        import livox_phone
        phone = livox_phone.Phone()
        offset, round_trip = phone.clock_offset()
        phone_info = phone.start(name, args.lens)
        if "error" in phone_info:
            print("Phone refused to start: %s" % phone_info["error"])
            return 1
        # add clock_offset_s to a phone timestamp to get this computer's clock (as used in the .lvraw file)
        phone_info.update(clock_offset_s=offset, clock_round_trip_s=round_trip, lidar_file=os.path.basename(path))
        print("Phone recording (%s lens, shutter 1/%.0f s, ISO %.0f)" % (
            phone_info.get("lens"), 1 / phone_info.get("shutter_s", 1), phone_info.get("iso", 0)))

    # Starting the phone took a few seconds while lidar data piled up unread; throw that backlog away
    # so the recording starts with fresh packets and the dropped-packet count only reflects real losses.
    for sock in (points, imu):
        sock.setblocking(False)
        try:
            while True:
                sock.recv(2048)
        except BlockingIOError:
            pass
        sock.setblocking(True)

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

    if phone:
        try:
            stopped = phone.stop()
            phone_info.update(stopped)
            with open(os.path.join(out_dir, name + ".phone.json"), "w") as f:
                json.dump(phone_info, f, indent=2)
            if "error" in stopped:
                print("Phone reported a problem stopping: %s" % stopped["error"])
            if stopped.get("file"):
                print("Phone video: %.0f s, %.0f MB" % (stopped.get("seconds", 0), stopped.get("bytes", 0) / 1e6))
                print(phone.fetch(stopped["file"], out_dir))
        except Exception as error:   # usually Wi-Fi out of reach at the end of a walk
            print("Could not reach the phone to stop or fetch (%s)." % error)
            print("The video is safe on the phone. When back in range: python3 livox_phone.py stop; "
                  "python3 livox_phone.py fetch %s.mov" % name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
