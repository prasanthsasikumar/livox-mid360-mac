#!/usr/bin/env python3
"""Cut a stitched cloud down to one rectangular room, give it a flat floor and tidy it.

    .venv/bin/python livox_room.py                       # newest stitched cloud, room found automatically
    .venv/bin/python livox_room.py scans/walk-...-stitched.ply --box -5.5 4.65 -6.6 14.9

What it does:
  1. Turns the cloud so the walls run along x and y.
  2. Finds the room the lidar was in: the strongest wall on each side of it
     (override with --box xmin xmax ymin ymax, in the turned frame, metres).
  3. Deletes everything outside those walls, above the ceiling or below the floor.
  4. Replaces the scanned floor with a complete flat one, wall to wall.
  5. Flattens the ceiling and the four walls: points within a few cm of each
     plane are moved onto it. Real detail smaller than that is lost on purpose.
  6. Drops stray points.
With --close it also completes the ceiling and walls, giving a sealed box with
the scanned furniture inside; real doorways and windows are covered over.
Writes <name>-room.ply and <name>-room-path.csv next to the input; run
livox_mesh.py on the -room.ply to mesh it.
"""
import argparse
import glob
import os
import sys

import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))
PLY_DTYPE = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "u1")]


def _load_ply(path):
    raw = open(path, "rb").read()
    end = raw.index(b"end_header\n") + 11
    names = [line.split()[-1] for line in raw[:end].decode().splitlines() if line.startswith("property")]
    dtype = PLY_DTYPE + ([("red", "u1"), ("green", "u1"), ("blue", "u1")] if "red" in names else [])
    return np.frombuffer(raw[end:], dtype=dtype)


def read_ply(path):
    """(xyz, intensity) from a cloud .ply written by these scripts."""
    data = _load_ply(path)
    return np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64), data["intensity"].copy()


def read_ply_rgb(path):
    """Photo colours (N x 3 uint8) if the cloud has them, else None."""
    data = _load_ply(path)
    if "red" not in data.dtype.names:
        return None
    return np.stack([data["red"], data["green"], data["blue"]], axis=1)


def write_ply(path, xyz, intensity, rgb=None):
    dtype = PLY_DTYPE + ([("red", "u1"), ("green", "u1"), ("blue", "u1")] if rgb is not None else [])
    out = np.empty(len(xyz), dtype=dtype)
    out["x"], out["y"], out["z"], out["intensity"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], intensity
    props = "property float x\nproperty float y\nproperty float z\nproperty uchar intensity\n"
    if rgb is not None:
        out["red"], out["green"], out["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        props += "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n%send_header\n" % (len(xyz), props)).encode())
        f.write(out.tobytes())


def strongest_peak(values, lo, hi, step=0.05):
    """Centre of the densest 15 cm of `values` between lo and hi."""
    hist, edges = np.histogram(values, bins=np.arange(lo, hi + step, step))
    smooth = np.convolve(hist, np.ones(3), mode="same")
    i = int(np.argmax(smooth))
    return float((edges[i] + edges[i + 1]) / 2)


def analyse(xyz):
    """Return (rotation to wall axes, [xmin, xmax, ymin, ymax], floor z, ceiling z)."""
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(0.04)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    p, n = np.asarray(cloud.points), np.asarray(cloud.normals)

    flat = np.abs(n[:, 2]) > 0.9
    ceiling = strongest_peak(p[flat & (p[:, 2] > 0.5), 2], 0.5, 6.0)
    floor = strongest_peak(p[flat & (p[:, 2] < -0.2), 2], -4.0, -0.2)

    # Walls: vertical surfaces between table height and just under the ceiling.
    wall = (np.abs(n[:, 2]) < 0.15) & (p[:, 2] > floor + 1.2) & (p[:, 2] < ceiling - 0.3)
    angle = np.degrees(np.arctan2(n[wall, 1], n[wall, 0])) % 90
    hist, edges = np.histogram(angle, bins=180, range=(0, 90))
    yaw = np.radians((edges[np.argmax(hist)] + edges[np.argmax(hist) + 1]) / 2)
    turn = np.array([[np.cos(-yaw), -np.sin(-yaw), 0], [np.sin(-yaw), np.cos(-yaw), 0], [0, 0, 1]])
    w, wn = p[wall] @ turn.T, n[wall] @ turn.T

    box = []
    for axis in (0, 1):
        facing = np.abs(wn[:, axis]) > 0.9      # walls whose face is across this axis
        along = w[facing, axis]
        box += [strongest_peak(along[along < -0.5], -40, -0.5), strongest_peak(along[along > 0.5], 0.5, 40)]
    return turn, box, floor, ceiling


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cloud", nargs="?")
    ap.add_argument("--box", type=float, nargs=4, metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                    help="wall positions in the turned frame, instead of finding them")
    ap.add_argument("--margin", type=float, default=0.4, help="how far beyond each wall to keep (alcoves, wall depth)")
    ap.add_argument("--snap", type=float, default=0.10, help="flatten points this close to a wall plane")
    ap.add_argument("--snap-ceiling", type=float, default=0.18, help="flatten points this close to the ceiling")
    ap.add_argument("--close", action="store_true",
                    help="also complete the ceiling and the four walls as flat surfaces (hides real openings)")
    ap.add_argument("--spacing", type=float, default=0.03, help="spacing of the added floor points")
    args = ap.parse_args()

    candidates = [f for f in glob.glob(os.path.join(HERE, "scans", "*-stitched.ply"))]
    path = args.cloud or (sorted(candidates, key=os.path.getmtime) or [None])[-1]
    if not path:
        sys.exit("No stitched cloud found. Run livox_stitch.py first.")
    base = os.path.splitext(path)[0]
    xyz, intensity = read_ply(path)
    print("Loaded {:,} points".format(len(xyz)))

    turn, box, floor, ceiling = analyse(xyz)
    if args.box:
        box = args.box
    xmin, xmax, ymin, ymax = box
    print("Room: %.2f m x %.2f m, floor to ceiling %.2f m" % (xmax - xmin, ymax - ymin, ceiling - floor))
    print("Walls at x = %.2f and %.2f, y = %.2f and %.2f (lidar start at 0, 0)" % (xmin, xmax, ymin, ymax))
    xyz = xyz @ turn.T

    m = args.margin
    keep = ((xyz[:, 0] > xmin - m) & (xyz[:, 0] < xmax + m) & (xyz[:, 1] > ymin - m) & (xyz[:, 1] < ymax + m)
            & (xyz[:, 2] > floor + 0.06) & (xyz[:, 2] < ceiling + args.snap_ceiling))
    xyz, intensity = xyz[keep], intensity[keep]
    print("Inside the room: {:,} points (scanned floor removed)".format(len(xyz)))

    # Flatten walls, then the ceiling (leaving wall points alone so walls still reach the ceiling).
    on_wall = np.zeros(len(xyz), bool)
    for axis, value in ((0, xmin), (0, xmax), (1, ymin), (1, ymax)):
        near = np.abs(xyz[:, axis] - value) < args.snap
        xyz[near, axis] = value
        on_wall |= near
    near = (np.abs(xyz[:, 2] - ceiling) < args.snap_ceiling) & ~on_wall
    xyz[near, 2] = ceiling
    above = xyz[:, 2] > ceiling
    xyz[above, 2] = ceiling

    # Stray points: anything without enough neighbours.
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    _, kept = cloud.remove_statistical_outlier(nb_neighbors=24, std_ratio=2.0)
    xyz, intensity = xyz[kept], intensity[kept]
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    _, kept = cloud.remove_radius_outlier(nb_points=3, radius=0.10)
    xyz, intensity = xyz[kept], intensity[kept]

    # One point per 2 cm cube (snapping piles points on top of each other).
    keys = np.floor(xyz / 0.02).astype(np.int64)
    packed = (keys[:, 0] + (1 << 20)) | ((keys[:, 1] + (1 << 20)) << 21) | ((keys[:, 2] + (1 << 20)) << 42)
    _, first = np.unique(packed, return_index=True)
    xyz, intensity = xyz[first], intensity[first]

    # New floor, wall to wall. With --close, the ceiling and the four walls are completed the same way.
    def sheet(u, v, fixed_axis, fixed_value):
        gu, gv = np.meshgrid(np.arange(u[0], u[1] + 1e-6, args.spacing), np.arange(v[0], v[1] + 1e-6, args.spacing))
        cols = [gu.ravel(), gv.ravel()]
        cols.insert(fixed_axis, np.full(gu.size, fixed_value))
        return np.stack(cols, axis=1)

    xs, ys, zs = (xmin, xmax), (ymin, ymax), (floor, ceiling)
    added = [sheet(xs, ys, 2, floor)]
    if args.close:
        added += [sheet(xs, ys, 2, ceiling), sheet(ys, zs, 0, xmin), sheet(ys, zs, 0, xmax),
                  sheet(xs, zs, 1, ymin), sheet(xs, zs, 1, ymax)]
    added = np.concatenate(added)
    xyz = np.concatenate([xyz, added])
    intensity = np.concatenate([intensity, np.full(len(added), 60, np.uint8)])
    print("After clean-up: {:,} points, including {:,} added for the new %s".format(len(xyz), len(added))
          % ("floor, ceiling and walls" if args.close else "floor"))

    write_ply(base + "-room.ply", xyz, intensity)
    track_file = base + "-path.csv"
    if os.path.exists(track_file):
        track = np.loadtxt(track_file, delimiter=",", skiprows=1).reshape(-1, 3) @ turn.T
        np.savetxt(base + "-room-path.csv", track, fmt="%.4f", delimiter=",", header="x,y,z", comments="")
    print("Wrote %s-room.ply" % base)


if __name__ == "__main__":
    main()
