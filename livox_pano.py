#!/usr/bin/env python3
"""Colour a stitched lidar cloud from the 360 photos a THETA took during the recording.

    .venv/bin/python livox_pano.py recordings/walk-<time>.lvraw     # after livox_stitch.py

Needs the recording's .theta.json and photos (from livox_record.py --theta) and the stitched
scans/walk-<time>-stitched.ply and -poses.csv (from livox_stitch.py).

Where each photo was taken: the lidar's pose at the moment of the shot, plus the fixed offset of the
camera on the rig. The THETA levels its own photos and the stitched map is level too, so the only
unknowns are which way the camera faces and that offset. They are found once, from the first photo,
by lining up edges in the photo with edges in a panorama rendered from the lidar points, and then
reused for every photo, since the camera is fixed to the lidar.
"""
import argparse
import json
import os
import subprocess
import sys

import cv2
import numpy as np

import livox_room

HERE = os.path.dirname(os.path.abspath(__file__))
NEAR = 0.45          # metres: points closer than this to the camera are the rig or the holder
FLOOR_CUT = -62.0    # degrees: below this the photo shows the rig itself
SHUTTER_DELAY = 1.0  # seconds from trigger to exposure


def directions(xyz, centre, yaw, mirror):
    """Longitude and latitude (radians) of each point as seen from `centre`, camera turned by `yaw`."""
    d = xyz - centre
    lon = np.arctan2(d[:, 1], d[:, 0]) - yaw
    if mirror:
        lon = -lon
    lon = (lon + np.pi) % (2 * np.pi) - np.pi
    rng = np.linalg.norm(d, axis=1)
    lat = np.arcsin(np.clip(d[:, 2] / np.maximum(rng, 1e-9), -1, 1))
    return lon, lat, rng


def to_pixels(lon, lat, width, height):
    u = (0.5 - lon / (2 * np.pi)) * width
    v = (0.5 - lat / np.pi) * height
    return np.clip(u.astype(int), 0, width - 1), np.clip(v.astype(int), 0, height - 1)


def depth_buffer(u, v, rng, width, height, grow=5):
    buf = np.full(width * height, np.inf)
    np.minimum.at(buf, v * width + u, rng)
    buf = buf.reshape(height, width)
    return cv2.erode(buf, np.ones((grow, grow), np.uint8))   # nearest depth in a patch closes gaps between points


def lidar_panorama(xyz, intensity, centre, mirror, width=1024, height=512):
    """Edge image of the lidar points as seen from `centre` (yaw 0), plus where the lidar has data."""
    lon, lat, rng = directions(xyz, centre, 0.0, mirror)
    keep = (rng > NEAR) & (np.degrees(lat) > FLOOR_CUT)
    u, v = to_pixels(lon[keep], lat[keep], width, height)
    buf = depth_buffer(u, v, rng[keep], width, height, grow=3)
    front = rng[keep] <= buf[v, u] * 1.03 + 0.05
    img = np.zeros((height, width), np.float32)
    img[v[front], u[front]] = intensity[keep][front]
    has = cv2.dilate((img > 0).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    img = cv2.dilate(img, np.ones((3, 3), np.uint8))
    depth = np.where(np.isfinite(buf), np.log(np.maximum(buf, 0.1)), 0).astype(np.float32)
    edges = edge_image(img / max(img.max(), 1)) + 0.5 * edge_image(depth / max(depth.max(), 1e-6))
    return edges, has


def edge_image(img):
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
    return cv2.GaussianBlur(np.sqrt(gx * gx + gy * gy), (0, 0), 2.0)


def photo_edges(photo, width=1024, height=512):
    grey = cv2.cvtColor(cv2.resize(photo, (width, height), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
    return edge_image(grey.astype(np.float32) / 255.0)


def best_shift(lidar_edges, has, photo_e):
    """Correlate over every horizontal (yaw) shift at once. Returns (score, shift in columns)."""
    height, width = photo_e.shape
    rows = slice(int(height * 0.08), int(height * (0.5 - FLOOR_CUT / 180.0)))   # skip the poles and the rig
    a = np.where(has, lidar_edges, 0)[rows]
    b = photo_e[rows]
    a = (a - a[has[rows]].mean()) * has[rows]
    b = b - b.mean()
    # circular cross-correlation along columns, summed over rows
    corr = np.fft.irfft(np.fft.rfft(b, axis=1) * np.conj(np.fft.rfft(a, axis=1)), n=width, axis=1).sum(0)
    norm = np.sqrt((a * a).sum() * (b * b).sum()) + 1e-9
    shift = int(np.argmax(corr))
    return corr[shift] / norm, shift


def calibrate(xyz, intensity, lidar_pos, photo):
    """Find the camera's yaw, mirror sense and offset from the lidar by matching edges."""
    photo_e = photo_edges(photo)
    width = photo_e.shape[1]
    best = (-1, None)
    offsets = [(dx, dy, dz) for dz in (0.05, 0.1, 0.15, 0.2, 0.25)
               for dx in (-0.08, 0, 0.08) for dy in (-0.08, 0, 0.08)]
    for mirror in (False, True):
        for off in offsets:
            edges, has = lidar_panorama(xyz, intensity, lidar_pos + off, mirror)
            score, shift = best_shift(edges, has, photo_e)
            if score > best[0]:
                best = (score, (mirror, np.array(off), shift))
    score, (mirror, off, shift) = best
    # finer offsets around the best one
    for dz in np.arange(-0.04, 0.041, 0.02):
        for dx in np.arange(-0.04, 0.041, 0.02):
            for dy in np.arange(-0.04, 0.041, 0.02):
                o = off + (dx, dy, dz)
                edges, has = lidar_panorama(xyz, intensity, lidar_pos + o, mirror)
                s, sh = best_shift(edges, has, photo_e)
                if s > score:
                    score, best_off, shift = s, o, sh
                    off = best_off
    # a shift of k columns means the photo is turned by k columns relative to the lidar panorama
    yaw = 2 * np.pi * shift / width * (1 if mirror else -1)
    return {"mirror": bool(mirror), "offset": off.tolist(), "yaw": float(yaw), "score": float(score)}


def pose_at(poses, t):
    i = int(np.clip(np.searchsorted(poses[:, 0], t), 0, len(poses) - 1))
    return poses[i, 1:4], poses[i, 4:13].reshape(3, 3)


def colour(xyz, shots, status=True):
    total = np.zeros((len(xyz), 3))
    weight = np.zeros(len(xyz))
    for shot in shots:
        photo = shot["image"]
        height, width = photo.shape[:2]
        lon, lat, rng = directions(xyz, shot["centre"], shot["yaw"], shot["mirror"])
        keep = (rng > NEAR) & (np.degrees(lat) > FLOOR_CUT)
        idx = np.flatnonzero(keep)
        bw, bh = 2048, 1024
        bu, bv = to_pixels(lon[keep], lat[keep], bw, bh)
        buf = depth_buffer(bu, bv, rng[keep], bw, bh)
        visible = rng[keep] <= buf[bv, bu] * 1.04 + 0.08
        u, v = to_pixels(lon[keep][visible], lat[keep][visible], width, height)
        w = 1.0 / (0.5 + rng[keep][visible]) ** 2          # nearer photos see more detail
        total[idx[visible]] += photo[v, u, ::-1] * w[:, None]
        weight[idx[visible]] += w
        if status:
            print("%s: coloured %d points" % (shot["file"], visible.sum()))
    seen = weight > 0
    rgb = np.zeros((len(xyz), 3), np.uint8)
    rgb[seen] = np.clip(total[seen] / weight[seen, None], 0, 255).astype(np.uint8)
    return rgb, seen


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("recording", help="the .lvraw file (its stitched cloud and THETA photos are found from it)")
    ap.add_argument("--recalibrate", action="store_true", help="find the camera offset again even if saved")
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()

    name = os.path.splitext(os.path.basename(args.recording))[0]
    rec_dir = os.path.dirname(os.path.abspath(args.recording))
    base = os.path.join(HERE, "scans", name + "-stitched")
    for need in (base + ".ply", base + "-poses.csv", os.path.join(rec_dir, name + ".theta.json")):
        if not os.path.exists(need):
            sys.exit("Missing %s (run livox_stitch.py on the recording first; it needs --theta photos)" % need)

    xyz, intensity = livox_room.read_ply(base + ".ply")
    xyz = xyz.astype(np.float64)
    intensity = intensity.astype(np.float32)
    poses = np.loadtxt(base + "-poses.csv", delimiter=",", skiprows=1)
    info = json.load(open(os.path.join(rec_dir, name + ".theta.json")))
    if not info["photos"]:
        sys.exit("The recording has no THETA photos.")

    shots = []
    for p in info["photos"]:
        t = p["triggered"] + SHUTTER_DELAY
        pos, rot = pose_at(poses, t)
        image = cv2.imread(os.path.join(rec_dir, p["file"]))
        if image is None:
            print("Could not read %s, skipped" % p["file"])
            continue
        lag = abs(poses[np.clip(np.searchsorted(poses[:, 0], t), 0, len(poses) - 1), 0] - t)
        if lag > 1.0:
            print("%s was taken %.1f s away from any lidar pose, skipped" % (p["file"], lag))
            continue
        shots.append({"file": p["file"], "image": image, "lidar_pos": pos, "lidar_rot": rot})
    if not shots:
        sys.exit("None of the photos line up with the lidar recording in time.")

    # The camera is fixed to the lidar: store its offset and turn in the lidar's own axes, so a walk
    # (where the lidar turns) reuses one calibration.
    cal_path = os.path.join(HERE, "theta-calibration.json")
    cal = None if args.recalibrate or not os.path.exists(cal_path) else json.load(open(cal_path))
    if cal is None:
        first = shots[0]
        print("Finding where the THETA sits on the rig (from %s)..." % first["file"])
        found = calibrate(xyz, intensity, first["lidar_pos"], first["image"])
        rot = first["lidar_rot"]
        # camera forward in the map, then in lidar axes
        fwd_map = np.array([np.cos(found["yaw"]), np.sin(found["yaw"]), 0.0])
        cal = {"offset_lidar": (rot.T @ np.array(found["offset"])).tolist(),
               "forward_lidar": (rot.T @ fwd_map).tolist(),
               "mirror": found["mirror"], "score": found["score"], "from": first["file"]}
        json.dump(cal, open(cal_path, "w"), indent=2)
        print("Camera sits %s m from the lidar centre (map axes), facing %.0f°, match score %.2f -> %s" % (
            np.round(found["offset"], 2).tolist(), np.degrees(found["yaw"]), found["score"], cal_path))

    for s in shots:
        rot = s["lidar_rot"]
        s["centre"] = s["lidar_pos"] + rot @ np.array(cal["offset_lidar"])
        fwd = rot @ np.array(cal["forward_lidar"])
        s["yaw"] = float(np.arctan2(fwd[1], fwd[0]))
        s["mirror"] = cal["mirror"]

    rgb, seen = colour(xyz, shots)
    print("%.0f%% of points were seen by a photo" % (100 * seen.mean()))
    grey = (40 + 215 * (np.minimum(intensity, 160) / 160.0) ** 0.6).astype(np.uint8)
    rgb[~seen] = grey[~seen, None]
    out = base + "-pano-colour.ply"
    livox_room.write_ply(out, xyz.astype(np.float32), intensity, rgb)
    print("Wrote %s" % out)
    subprocess.run([sys.executable, os.path.join(HERE, "livox_view.py"), out] + (["--no-open"] if args.no_open else []),
                   check=True)


if __name__ == "__main__":
    main()
