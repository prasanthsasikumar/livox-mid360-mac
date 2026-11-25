#!/usr/bin/env python3
"""Train a detailed Gaussian splat from THETA stills taken during a lidar recording.

    .venv/bin/python livox_stitch.py recordings/walk-<time>.lvraw --no-open
    .venv/bin/python livox_gsplat.py recordings/walk-<time>.lvraw            # dataset, then training
    .venv/bin/python livox_gsplat.py recordings/walk-<time>.lvraw --steps 7000   # quicker, softer

Capture (livox_record.py --theta --theta-every 12): stop and hold the rig still for every shot, at many
spots, and go closer to anything that should be readable (screens, keyboards, labels). Moving people blur.

What it does:
  1. Places each still: the lidar's pose at the moment of the shot plus the camera's fixed place on the rig
     (theta-calibration.json, from livox_pano.py). The THETA levels its own photos and the map is level,
     so each still only needs a position and a heading.
  2. Cuts each 360 still into ordinary flat views (the trainer needs normal photos), leaving out the view
     back towards the person holding the rig, and masks the rig itself at the bottom.
  3. Writes them as a COLMAP-format dataset in splats/<name>/, with the photo-coloured lidar points as the
     starting geometry, and trains it with Brush (https://github.com/ArthurBrussee/brush) on the Mac's GPU.
  4. The result lands in scans/<name>-trained-splat.ply, which http://localhost:8360/splat lists.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np

import livox_pano
import livox_room

HERE = os.path.dirname(os.path.abspath(__file__))
BRUSH = os.environ.get("BRUSH", os.path.expanduser("~/tools/brush-dl/brush-app-aarch64-apple-darwin/brush_app"))
VIEW_SIZE = 1600          # pixels; 90° across, about the still's own sharpness (6720 px / 360°)
FOV = 90.0
RIG_CUT = -60.0           # degrees below the horizon where the rig itself shows: masked
# (azimuth, pitch) of each flat view, in the camera's frame: azimuth 0 is the lidar's dome direction,
# 180 is back towards the holder (left out unless --keep-back)
VIEWS = [(a, 0) for a in (0, 50, 100, -50, -100)] + \
        [(a, -40) for a in (0, 60, 120, -60, -120)] + \
        [(a, 40) for a in (0, 90, -90)] + [(0, 85)]
BACK_VIEWS = [(180, 0), (150, -40), (-150, -40), (180, 40)]
INIT_POINTS = 600_000


def view_axes(azimuth, pitch):
    """Forward, right, down of a flat view in the camera frame (x forward, y left, z up)."""
    a, p = np.radians(azimuth), np.radians(pitch)
    fwd = np.array([np.cos(p) * np.cos(a), np.cos(p) * np.sin(a), np.sin(p)])
    right = np.array([np.sin(a), -np.cos(a), 0.0])
    down = np.cross(fwd, right)
    return fwd, right, down


def cut_view(pano, azimuth, pitch, size=VIEW_SIZE):
    """A flat (pinhole) view out of an equirectangular photo, and a mask of where the rig is."""
    h, w = pano.shape[:2]
    f = size / 2 / np.tan(np.radians(FOV / 2))
    fwd, right, down = view_axes(azimuth, pitch)
    px = (np.arange(size) + 0.5 - size / 2) / f
    gx, gy = np.meshgrid(px, px)
    rays = fwd[None, None] + gx[..., None] * right + gy[..., None] * down
    lon = np.arctan2(rays[..., 1], rays[..., 0])
    lat = np.arcsin(rays[..., 2] / np.linalg.norm(rays, axis=2))
    u = ((0.5 - lon / (2 * np.pi)) * w).astype(np.float32)
    v = ((0.5 - lat / np.pi) * h).astype(np.float32)
    img = cv2.remap(pano, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
    mask = (np.degrees(lat) > RIG_CUT).astype(np.uint8) * 255
    return img, mask, f


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def quaternion(r):
    """(w, x, y, z) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def speed_at(poses, t, span=0.6):
    """How far the rig moved (metres) and turned (degrees) in the `span` seconds around t."""
    a, b = livox_pano.pose_at(poses, t - span / 2), livox_pano.pose_at(poses, t + span / 2)
    turn = np.degrees(np.arccos(np.clip((np.trace(a[1].T @ b[1]) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(b[0] - a[0])), float(turn)


def build_dataset(name, rec_dir, out, keep_back=False):
    base = os.path.join(HERE, "scans", name + "-stitched")
    need = [base + ".ply", base + "-poses.csv", os.path.join(rec_dir, name + ".theta.json"),
            os.path.join(HERE, "theta-calibration.json")]
    for n in need:
        if not os.path.exists(n):
            sys.exit("Missing %s (record with --theta, run livox_stitch.py, and calibrate once with livox_pano.py)" % n)
    poses = np.loadtxt(base + "-poses.csv", delimiter=",", skiprows=1)
    info = json.load(open(need[2]))
    cal = json.load(open(need[3]))
    if cal["mirror"]:
        sys.exit("theta-calibration.json says mirrored; this script expects a normal THETA photo")

    if os.path.exists(out):
        shutil.rmtree(out)
    os.makedirs(os.path.join(out, "images"))
    os.makedirs(os.path.join(out, "masks"))
    os.makedirs(os.path.join(out, "sparse", "0"))

    views = VIEWS + (BACK_VIEWS if keep_back else [])
    lines, used, focal, start = [], 0, None, None
    for p in info["photos"]:
        t = p["triggered"] + livox_pano.SHUTTER_DELAY
        i = int(np.clip(np.searchsorted(poses[:, 0], t), 0, len(poses) - 1))
        if abs(poses[i, 0] - t) > 1.0:
            print("%s: no lidar pose near the shot, skipped" % p["file"])
            continue
        moved, turned = speed_at(poses, t)
        if moved > 0.08 or turned > 4:
            print("%s: rig was moving (%.2f m, %.0f° in 0.6 s), skipped as blurred" % (p["file"], moved, turned))
            continue
        pano = cv2.imread(os.path.join(rec_dir, p["file"]))
        if pano is None:
            print("%s: could not read, skipped" % p["file"])
            continue
        pos, rot = livox_pano.pose_at(poses, t)
        centre = pos + rot @ np.array(cal["offset_lidar"])
        fwd = rot @ np.array(cal["forward_lidar"])
        cam_to_world = rot_z(np.arctan2(fwd[1], fwd[0]))      # camera frame (forward, left, up) -> map
        stem = os.path.splitext(p["file"])[0]
        for azimuth, pitch in views:
            img, mask, focal = cut_view(pano, azimuth, pitch)
            fname = "%s_a%+04d_p%+03d.jpg" % (stem, azimuth, pitch)
            cv2.imwrite(os.path.join(out, "images", fname), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            cv2.imwrite(os.path.join(out, "masks", os.path.splitext(fname)[0] + ".png"), mask)
            f, r, d = (cam_to_world @ v for v in view_axes(azimuth, pitch))
            r_wc = np.stack([r, d, f])                       # rows: COLMAP camera x (right), y (down), z (forward)
            q = quaternion(r_wc)
            tvec = -r_wc @ centre
            lines.append("%d %.9f %.9f %.9f %.9f %.6f %.6f %.6f 1 %s\n\n" % ((len(lines) + 1,) + tuple(q) + tuple(tvec) + (fname,)))
        if not used:
            start = {"camera": centre.tolist(), "look": (centre + 2.0 * np.array([fwd[0], fwd[1], 0.0])).tolist()}
        used += 1
        print("%s: %d views (rig moved %.2f m, turned %.1f° around the shot)" % (p["file"], len(views), moved, turned))
    if not used:
        sys.exit("No usable stills.")

    with open(os.path.join(out, "sparse", "0", "cameras.txt"), "w") as fh:
        fh.write("1 PINHOLE %d %d %.6f %.6f %.6f %.6f\n" % (VIEW_SIZE, VIEW_SIZE, focal, focal, VIEW_SIZE / 2, VIEW_SIZE / 2))
    with open(os.path.join(out, "sparse", "0", "images.txt"), "w") as fh:
        fh.writelines(lines)

    # starting geometry: the lidar points, photo-coloured if livox_pano.py has been run
    cloud = base + "-pano-colour.ply" if os.path.exists(base + "-pano-colour.ply") else base + ".ply"
    xyz, intensity = livox_room.read_ply(cloud)
    rgb = livox_room.read_ply_rgb(cloud)
    if rgb is None:
        g = (40 + 215 * (np.minimum(intensity, 160) / 160.0) ** 0.6).astype(np.uint8)
        rgb = np.stack([g, g, g], 1)
    if len(xyz) > INIT_POINTS:
        pick = np.random.default_rng(0).choice(len(xyz), INIT_POINTS, replace=False)
        xyz, rgb = xyz[pick], rgb[pick]
    with open(os.path.join(out, "sparse", "0", "points3D.txt"), "w") as fh:
        for k, (p, c) in enumerate(zip(xyz, rgb)):
            fh.write("%d %.4f %.4f %.4f %d %d %d 0\n" % (k + 1, p[0], p[1], p[2], c[0], c[1], c[2]))
    json.dump(start, open(os.path.join(out, "start-view.json"), "w"))
    print("Dataset: %d stills -> %d views, %d starting points (%s) in %s" % (
        used, len(lines), len(xyz), os.path.basename(cloud), out))
    return used


def train(dataset, name, steps, viewer):
    if not os.path.exists(BRUSH):
        sys.exit("Brush not found at %s (set BRUSH=/path/to/brush_app)" % BRUSH)
    export_dir = os.path.join(dataset, "export")
    os.makedirs(export_dir, exist_ok=True)
    cmd = [BRUSH, dataset, "--total-steps", str(steps), "--export-every", str(max(1000, steps // 5)),
           "--export-path", export_dir, "--export-name", "splat_{iter}.ply"] + (["--with-viewer"] if viewer else [])
    print("Training: " + " ".join(cmd))
    subprocess.run(cmd, check=True)
    done = sorted(glob.glob(os.path.join(export_dir, "splat_*.ply")), key=os.path.getmtime)
    if not done:
        sys.exit("Brush wrote no splat to %s" % export_dir)
    out = os.path.join(HERE, "scans", name + "-trained-splat.ply")
    shutil.copy(done[-1], out)
    shutil.copy(os.path.join(dataset, "start-view.json"), out + ".json")   # the splat page starts here
    print("Wrote %s  (open http://localhost:8360/splat)" % out)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording", help="the .lvraw file recorded with --theta")
    ap.add_argument("--steps", type=int, default=30000, help="training steps (30000 best, 7000 quick look)")
    ap.add_argument("--keep-back", action="store_true", help="also use the view back towards the holder")
    ap.add_argument("--dataset-only", action="store_true", help="write the dataset and stop")
    ap.add_argument("--viewer", action="store_true", help="show Brush's window while it trains")
    args = ap.parse_args()
    name = os.path.splitext(os.path.basename(args.recording))[0]
    rec_dir = os.path.dirname(os.path.abspath(args.recording))
    dataset = os.path.join(HERE, "splats", name)
    build_dataset(name, rec_dir, dataset, args.keep_back)
    if not args.dataset_only:
        train(dataset, name, args.steps, args.viewer)


if __name__ == "__main__":
    main()
