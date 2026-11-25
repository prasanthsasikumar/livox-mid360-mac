#!/usr/bin/env python3
"""Stitch a walking recording into one point cloud.

    .venv/bin/python livox_stitch.py                      # newest recording
    .venv/bin/python livox_stitch.py recordings/walk-....lvraw

Works out how the lidar moved, places every scan in a common frame, and writes
a .ply plus the same .html viewer as livox_cloud.py into scans/.

Two methods:
    --method icp   (default) KISS-ICP: points only.
    --method lio   RKO-LIO: points plus the motion sensor. Needs the lidar to be
                   completely still for the first few seconds.
Neither has loop closure, so long walks drift. Afterwards the map is levelled
on its largest flat horizontal surface (ceiling or floor), so it comes out
upright even if the lidar was being held at the start.
"""
import argparse
import base64
import glob
import os
import struct
import subprocess
import sys
import time

import numpy as np

import livox_cloud
import livox_probe as lp
import livox_record as rec

HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_SECONDS = 0.1
MIN_RANGE, MAX_RANGE = 0.7, 30.0   # below 0.7 m is mostly the person carrying it
MAP_VOXEL = 0.02                   # output cloud keeps one point per 2 cm cube
VIEWER_MAX_POINTS = 1500000
POINT_DTYPE = {1: (np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1"), ("tag", "u1")]), 0.001),
               2: (np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("r", "u1"), ("tag", "u1")]), 0.01)}


def frames(path):
    """Yield (xyz metres, reflectivity, per-point lidar time in seconds) for each 0.1 s of points."""
    start = None
    xyz, refl, stamps = [], [], []
    for _, kind, data in rec.read_packets(path):
        if kind != rec.KIND_POINTS or len(data) <= lp.POINT_HEADER.size or data[10] not in POINT_DTYPE:
            continue
        dtype, scale = POINT_DTYPE[data[10]]
        interval, dot_num = struct.unpack_from("<HH", data, 3)
        t0 = struct.unpack_from("<Q", data, 28)[0] * 1e-9
        if start is None:
            start = t0
        if t0 - start >= FRAME_SECONDS and xyz:
            yield np.concatenate(xyz), np.concatenate(refl), np.concatenate(stamps)
            xyz, refl, stamps = [], [], []
            start = t0
        body = data[lp.POINT_HEADER.size:]
        pts = np.frombuffer(body, dtype=dtype, count=len(body) // dtype.itemsize)
        p = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64) * scale
        times = t0 + np.arange(len(pts)) * (interval * 1e-7 / max(dot_num, 1))
        dist = np.linalg.norm(p, axis=1)
        keep = (dist > MIN_RANGE) & (dist < MAX_RANGE)
        xyz.append(p[keep]); refl.append(pts["r"][keep]); stamps.append(times[keep])


def up_rotation(path, seconds=2.0):
    """Rotation that makes 'up' (measured while standing still at the start) the z axis."""
    samples, first = [], None
    for _, kind, data in rec.read_packets(path):
        if kind != rec.KIND_IMU or len(data) < lp.POINT_HEADER.size + 24:
            continue
        t = struct.unpack_from("<Q", data, 28)[0] * 1e-9
        first = t if first is None else first
        if t - first > seconds:
            break
        samples.append(struct.unpack_from("<3f", data, lp.POINT_HEADER.size + 12))
    if not samples:
        return np.eye(3)
    a = np.mean(samples, axis=0)
    a /= np.linalg.norm(a)
    v, c = np.cross(a, [0, 0, 1]), a[2]
    if c < -0.999:
        return np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def imu_samples(path):
    """All motion-sensor samples: (lidar time ns, gyro rad/s, acceleration m/s^2)."""
    times, gyro, acc = [], [], []
    for _, kind, data in rec.read_packets(path):
        if kind == rec.KIND_IMU and len(data) >= lp.POINT_HEADER.size + 24:
            v = struct.unpack_from("<6f", data, lp.POINT_HEADER.size)
            times.append(struct.unpack_from("<Q", data, 28)[0])
            gyro.append(v[:3]); acc.append(v[3:])
    return np.array(times, np.int64), np.array(gyro), np.array(acc) * 9.80665   # Livox reports g


class IcpOdometry:
    """Points only (KISS-ICP). Output is rotated so the still start defines 'up'."""

    def __init__(self, path):
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        config = KISSConfig()
        config.data.min_range, config.data.max_range, config.data.deskew = 0.0, 100.0, True  # already cropped
        config.mapping.voxel_size = 0.15
        self.odometry = KissICP(config)
        self.level = np.eye(4)
        self.level[:3, :3] = up_rotation(path)

    def register(self, xyz, stamps):
        relative = (stamps - stamps[0]) / FRAME_SECONDS
        deskewed, _ = self.odometry.register_frame(xyz, relative)
        return np.asarray(deskewed), self.level @ self.odometry.last_pose


class LioOdometry:
    """Points plus motion sensor (RKO-LIO). Gravity-aligned from the still start."""

    # Where the Mid-360's motion sensor sits relative to the lidar origin (Livox manual), metres.
    IMU_OFFSET = (0.011, 0.02329, -0.04412)

    def __init__(self, path):
        from rko_lio.config import LIOConfig
        from rko_lio.lio import LIO
        self.lio = LIO(LIOConfig(voxel_size=0.25, min_range=MIN_RANGE, max_range=MAX_RANGE,
                                 initialization_phase=True, max_num_threads=0))
        self.imu2base = np.eye(4)
        self.imu2base[:3, 3] = self.IMU_OFFSET
        self.times, self.gyro, self.acc = imu_samples(path)
        self.next = 0

    def register(self, xyz, stamps):
        ns = np.round(stamps * 1e9).astype(np.int64)
        end = int(ns.max())
        while self.next < len(self.times) and self.times[self.next] <= end:
            i = self.next
            self.lio.add_imu_measurement(self.acc[i], self.gyro[i], int(self.times[i]), self.imu2base)
            self.next += 1
        deskewed = self.lio.register_scan(xyz, ns)
        return np.asarray(deskewed), self.lio.pose()


def level_rotation(xyz, max_tilt_deg=30.0):
    """Rotation that makes the largest roughly-horizontal plane (ceiling or floor) exactly horizontal."""
    import open3d as o3d
    sample = xyz[np.random.default_rng(0).choice(len(xyz), min(len(xyz), 400000), replace=False)].astype(np.float64)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sample))
    for _ in range(4):   # the biggest plane may be a wall; peel planes off until one is near-horizontal
        if len(cloud.points) < 1000:
            break
        (a, b, c, _), inliers = cloud.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=1000)
        n = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        n = n if n[2] > 0 else -n
        tilt = np.degrees(np.arccos(min(1.0, n[2])))
        if tilt < max_tilt_deg:
            v, cos = np.cross(n, [0, 0, 1]), n[2]
            k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            return np.eye(3) + k + k @ k / (1 + cos), tilt
        cloud = cloud.select_by_index(inliers, invert=True)
    return np.eye(3), 0.0


def thin(xyz, refl, voxel):
    """Keep one point per voxel."""
    keys = np.floor(xyz / voxel).astype(np.int64)
    packed = (keys[:, 0] + (1 << 20)) | ((keys[:, 1] + (1 << 20)) << 21) | ((keys[:, 2] + (1 << 20)) << 42)
    _, idx = np.unique(packed, return_index=True)
    return xyz[idx], refl[idx]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--method", choices=["icp", "lio"], default="icp")
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()
    path = args.recording or (sorted(glob.glob(os.path.join(HERE, "recordings", "*.lvraw"))) or [None])[-1]
    if not path:
        sys.exit("No recording found. Run livox_record.py first.")

    odometry = (LioOdometry if args.method == "lio" else IcpOdometry)(path)
    chunks_xyz, chunks_refl, track = [], [], []
    began = time.time()
    for i, (xyz, refl, stamps) in enumerate(frames(path)):
        if len(xyz) < 500:
            continue
        try:
            deskewed, pose = odometry.register(xyz, stamps)
        except Exception as e:   # a scan the odometry rejects (e.g. no motion data yet) is skipped
            print("\nskipping frame %d: %s" % (i, e))
            continue
        if len(deskewed) == 0:
            continue
        world = deskewed @ pose[:3, :3].T + pose[:3, 3]
        r = refl if len(refl) == len(world) else np.full(len(world), 128, np.uint8)
        world, r = thin(world, r, MAP_VOXEL)
        chunks_xyz.append(world.astype(np.float32)); chunks_refl.append(r)
        track.append(pose[:3, 3].copy())
        if i % 20 == 0:
            sys.stdout.write("\rframe %d, %.1f m moved  " % (i, np.linalg.norm(np.diff(track, axis=0), axis=1).sum()))
            sys.stdout.flush()
    if not track:
        sys.exit("\nThe recording has no usable frames.")

    xyz, refl = thin(np.concatenate(chunks_xyz), np.concatenate(chunks_refl), MAP_VOXEL)
    track = np.array(track)
    level, tilt = level_rotation(xyz)
    xyz, track = (xyz @ level.T).astype(np.float32), track @ level.T
    print("\rLevelled the map by %.1f degrees" % tilt)
    walked = np.linalg.norm(np.diff(track, axis=0), axis=1).sum()
    print("%d frames in %.0f s" % (len(track), time.time() - began))
    print("Path length      : %.1f m" % walked)
    print("Start-to-end gap : %.2f m (if you finished where you started, this is the drift)" % (
        np.linalg.norm(track[-1] - track[0])))
    print("Points in map    : {:,}".format(len(xyz)))

    out_dir = os.path.join(HERE, "scans")
    os.makedirs(out_dir, exist_ok=True)
    name = os.path.splitext(os.path.basename(path))[0] + "-stitched"
    base = os.path.join(out_dir, name)

    ply = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "u1")])
    ply["x"], ply["y"], ply["z"], ply["intensity"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], refl
    with open(base + ".ply", "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\n"
                 "property float z\nproperty uchar intensity\nend_header\n" % len(xyz)).encode())
        f.write(ply.tobytes())
    np.savetxt(base + "-path.csv", track, fmt="%.4f", delimiter=",", header="x,y,z", comments="")

    if len(xyz) > VIEWER_MAX_POINTS:
        pick = np.random.default_rng(0).choice(len(xyz), VIEWER_MAX_POINTS, replace=False)
        vx, vr = xyz[pick], refl[pick]
    else:
        vx, vr = xyz, refl
    z_lo, z_hi = np.percentile(vx[:, 2], [1, 99])
    radius = float(np.percentile(np.abs(vx[:, :2]), 99)) * 1.1
    html = (livox_cloud.VIEWER.replace("__TITLE__", name).replace("__COUNT__", "{:,}".format(len(vx)))
            .replace("__ZLO__", "%.3f" % z_lo).replace("__ZHI__", "%.3f" % z_hi)
            .replace("__RADIUS__", "%.3f" % radius)
            .replace("__XYZ__", base64.b64encode(np.ascontiguousarray(vx, np.float32).tobytes()).decode())
            .replace("__REFL__", base64.b64encode(np.ascontiguousarray(vr, np.uint8).tobytes()).decode()))
    with open(base + ".html", "w") as f:
        f.write(html)
    print("Wrote %s.html, .ply, -path.csv" % base)
    if not args.no_open:
        subprocess.run(["open", base + ".html"])


if __name__ == "__main__":
    main()
