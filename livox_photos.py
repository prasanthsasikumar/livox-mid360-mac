#!/usr/bin/env python3
"""Locate a set of photos inside a stitched lidar cloud, ready for livox_colour.py.

    .venv/bin/python livox_photos.py "~/Downloads/office images" scans/<name>-stitched.ply --name office

Steps:
  1. Copies the photos into photos/<name>/images as JPEGs (uses macOS `sips`, which reads HEIC).
  2. Reconstructs camera positions and a sparse 3D cloud from the photos alone (COLMAP).
     Slow: about 10 minutes for 26 photos on a laptop. Skipped if already done.
  3. Fits that reconstruction onto the lidar cloud. A photo reconstruction has no
     real-world size or direction, so this finds: which way is up (from the floor
     the photos see), the scale (the camera is assumed to be 1.0-2.0 m above the
     floor), then heading and position by sliding one floor plan over the other,
     and finally a fine 3D fit.
Writes photos/<name>/sfm_to_lidar.npy and prints how good the fit is.

Works with ordinary walk-around photo sets and, as it turned out, with a handheld
spin on the spot, as long as neighbouring photos overlap heavily.
"""
import argparse
import glob
import os
import subprocess
import sys

import numpy as np
import open3d as o3d
import pycolmap

import livox_room

HERE = os.path.dirname(os.path.abspath(__file__))
REG = o3d.pipelines.registration


def prepare_images(source, out_dir, longest=3200):
    os.makedirs(out_dir, exist_ok=True)
    files = sorted(f for f in glob.glob(os.path.join(os.path.expanduser(source), "*"))
                   if f.lower().endswith((".heic", ".jpg", ".jpeg", ".png")))
    for f in files:
        target = os.path.join(out_dir, os.path.splitext(os.path.basename(f))[0] + ".jpg")
        if not os.path.exists(target):
            subprocess.run(["sips", "-s", "format", "jpeg", "-s", "formatOptions", "90", "-Z", str(longest),
                            f, "--out", target], check=True, capture_output=True)
    return len(files)


def reconstruct(root):
    model = os.path.join(root, "sfm", "0")
    if os.path.exists(os.path.join(model, "images.bin")):
        return pycolmap.Reconstruction(model)
    os.makedirs(os.path.join(root, "sfm"), exist_ok=True)
    database = os.path.join(root, "db.db")
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"          # livox_colour.py assumes this model
    pycolmap.extract_features(database, os.path.join(root, "images"),
                              camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader)
    pycolmap.match_exhaustive(database)
    maps = pycolmap.incremental_mapping(database, os.path.join(root, "images"), os.path.join(root, "sfm"))
    if not maps:
        sys.exit("The photos could not be reconstructed. They need more overlap, or some sideways movement.")
    return pycolmap.Reconstruction(model)


def rotation_to_z(direction):
    v, c = np.cross(direction, [0, 0, 1]), direction[2]
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def level(rec):
    """Return (4x4 that puts the camera centre at the origin with z up, levelled points, floor height)."""
    images = list(rec.images.values())
    centres = np.array([im.projection_center() for im in images])
    points = np.array([p.xyz for p in rec.points3D.values() if p.track.length() >= 3 and p.error < 2])
    origin = centres.mean(0)
    up = np.mean([-im.cam_from_world().rotation.matrix()[1] for im in images], axis=0)   # photos are held upright
    up /= np.linalg.norm(up)
    height = (points - origin) @ up
    low = points[height < np.percentile(height, 40)]
    spread = np.median(np.linalg.norm(points - origin, axis=1))
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(low))
    (a, b, c, _), inliers = cloud.segment_plane(0.015 * spread, 3, 3000)
    normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    normal = normal if normal @ up > 0 else -normal
    if np.degrees(np.arccos(normal @ up)) > 25:
        sys.exit("Could not find the floor in the photos; they need to show some floor.")
    turn = rotation_to_z(normal)
    to_level = np.eye(4)
    to_level[:3, :3], to_level[:3, 3] = turn, -turn @ origin
    levelled = (points - origin) @ turn.T
    floor = float(np.median(((low[inliers] - origin) @ turn.T)[:, 2]))
    return to_level, levelled, floor


def coarse_candidates(levelled, floor, lidar, lidar_floor, lidar_ceiling, cell=0.1, size=512, per_scale=3):
    """Slide the photo floor plan over the lidar one for every scale and heading; return the best few per scale."""
    def plan(xy):
        ij = np.floor(xy / cell).astype(int) + size // 2
        ok = (ij >= 0).all(1) & (ij < size).all(1)
        grid = np.zeros((size, size), np.float32)
        grid[ij[ok, 0], ij[ok, 1]] = 1
        return grid

    band = lidar[(lidar[:, 2] > lidar_floor + 0.25) & (lidar[:, 2] < lidar_ceiling - 0.25)]
    lidar_fft = np.fft.rfft2(plan(band[:, :2]))
    # The plan-matching score is not comparable between scales (it favours large ones), so keep the
    # best few headings from every scale and let the 3D fit decide between them.
    candidates = []
    for camera_height in np.arange(1.0, 2.01, 0.04):
        scale = camera_height / abs(floor)
        pts = levelled * scale
        pts = pts[(pts[:, 2] - floor * scale > 0.25) & (np.linalg.norm(pts[:, :2], axis=1) < 12)]
        found = []
        for yaw in np.radians(np.arange(0, 360, 1.0)):
            cy, sy = np.cos(yaw), np.sin(yaw)
            xy = pts[:, :2] @ np.array([[cy, -sy], [sy, cy]]).T
            corr = np.fft.irfft2(lidar_fft * np.conj(np.fft.rfft2(plan(xy))), s=(size, size))
            k = np.unravel_index(np.argmax(corr), corr.shape)
            shift = [((k[i] + size // 2) % size - size // 2) * cell for i in (0, 1)]
            found.append((corr[k], yaw, shift[0], shift[1]))
        found.sort(reverse=True)
        kept = []
        for _, yaw, sx, sy in found:
            if all(abs((yaw - other + np.pi) % (2 * np.pi) - np.pi) > np.radians(8) for other, _, _ in kept):
                kept.append((yaw, sx, sy))
            if len(kept) == per_scale:
                break
        candidates += [(scale, yaw, sx, sy) for yaw, sx, sy in kept]
    return candidates


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("photo_folder")
    ap.add_argument("cloud", help="the stitched lidar cloud (.ply)")
    ap.add_argument("--name", default="photos", help="working folder name under photos/")
    args = ap.parse_args()

    root = os.path.join(HERE, "photos", args.name)
    count = prepare_images(args.photo_folder, os.path.join(root, "images"))
    print("%d photos ready in %s" % (count, os.path.join(root, "images")))
    rec = reconstruct(root)
    print("Photo reconstruction: %d of %d photos located, %d 3D points" % (
        rec.num_reg_images(), count, rec.num_points3D()))

    to_level, levelled, floor = level(rec)
    lidar, _ = livox_room.read_ply(args.cloud)
    _, _, lidar_floor, lidar_ceiling = livox_room.analyse(lidar)
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(lidar)).voxel_down_sample(0.04)

    reach = 5.2 * abs(floor)        # about 5-10 m: beyond that, depth from photos is too rough to fit with
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(levelled[np.linalg.norm(levelled, axis=1) < reach]))
    best = None
    for scale, yaw, sx, sy in coarse_candidates(levelled, floor, np.asarray(target.points), lidar_floor, lidar_ceiling):
        fit = np.eye(4)
        fit[:3, :3] = scale * np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        fit[:3, 3] = [sx, sy, lidar_floor - floor * scale]
        for reach_m in (0.5, 0.25, 0.12):
            fit = REG.registration_icp(source, target, reach_m, fit,
                                       REG.TransformationEstimationPointToPoint(with_scaling=True),
                                       REG.ICPConvergenceCriteria(max_iteration=80)).transformation
        quality = REG.evaluate_registration(source, target, 0.08, fit)
        if best is None or quality.fitness > best[0]:
            best = (quality.fitness, quality.inlier_rmse, fit)

    fitness, rmse, fit = best
    to_lidar = fit @ to_level
    np.save(os.path.join(root, "sfm_to_lidar.npy"), to_lidar)
    scale = np.cbrt(np.linalg.det(fit[:3, :3]))
    centres = np.array([im.projection_center() for im in rec.images.values()])
    centres = centres @ to_lidar[:3, :3].T + to_lidar[:3, 3]
    print("Fit: %.0f%% of nearby photo points lie within 8 cm of a lidar surface (typical miss %.0f cm)" % (
        100 * fitness, 100 * rmse))
    print("Cameras sit around (%.2f, %.2f, %.2f) m in the scan, %.2f m above the floor" % (
        *np.median(centres, axis=0), np.median(centres[:, 2]) - lidar_floor))
    print("Wrote %s" % os.path.join(root, "sfm_to_lidar.npy"))
    if fitness < 0.4:
        print("That is a weak fit; expect colours to be offset from the geometry.")


if __name__ == "__main__":
    main()
