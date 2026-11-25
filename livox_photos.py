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


def prepare_video(video, out_dir, fps=2.0, longest=2000):
    """Pull still frames out of a video at `fps`. Returns {image name: seconds from the first frame}."""
    os.makedirs(out_dir, exist_ok=True)
    if not glob.glob(os.path.join(out_dir, "frame_*.jpg")):
        subprocess.run(["ffmpeg", "-loglevel", "error", "-i", os.path.expanduser(video),
                        "-vf", "fps=%g,scale=%d:-2" % (fps, longest), "-q:v", "2",
                        os.path.join(out_dir, "frame_%05d.jpg")], check=True)
    names = sorted(os.path.basename(f) for f in glob.glob(os.path.join(out_dir, "frame_*.jpg")))
    return {name: i / fps for i, name in enumerate(names)}


def best_model(root):
    """COLMAP can split a set into several pieces; use the one that located the most photos."""
    found = [pycolmap.Reconstruction(os.path.dirname(f))
             for f in glob.glob(os.path.join(root, "sfm", "*", "images.bin"))]
    return max(found, key=lambda r: r.num_reg_images()) if found else None


def reconstruct(root, sequential=False):
    existing = best_model(root)
    if existing is not None:
        return existing
    os.makedirs(os.path.join(root, "sfm"), exist_ok=True)
    database = os.path.join(root, "db.db")
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL"          # livox_colour.py assumes this model
    pycolmap.extract_features(database, os.path.join(root, "images"),
                              camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader)
    if sequential:      # video: each frame only needs comparing with its neighbours in time
        pycolmap.match_sequential(database, pairing_options=pycolmap.SequentialPairingOptions(overlap=12))
    else:
        pycolmap.match_exhaustive(database)
    maps = pycolmap.incremental_mapping(database, os.path.join(root, "images"), os.path.join(root, "sfm"))
    if not maps:
        sys.exit("The photos could not be reconstructed. They need more overlap, or some sideways movement.")
    return best_model(root)


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


def save_cameras(root, rec, names, rotations, centres):
    """Write photos/<name>/cameras.npz: every located photo's pose in the lidar frame, in metres.

    rotations[i] holds the camera's axes (x right, y down, z forward) as columns, in lidar coordinates.
    """
    camera = next(iter(rec.cameras.values()))
    np.savez(os.path.join(root, "cameras.npz"), names=np.array(names), rotations=np.array(rotations),
             centres=np.array(centres), intrinsics=np.array(camera.params, dtype=float),   # f, cx, cy, k
             size=np.array([camera.width, camera.height]))
    print("Wrote %s (%d cameras)" % (os.path.join(root, "cameras.npz"), len(names)))


def save_cameras_from_fit(root, rec, to_lidar):
    """Cameras for a reconstruction placed as a whole by a scale-rotate-shift fit."""
    scale = np.cbrt(np.linalg.det(to_lidar[:3, :3]))
    turn = to_lidar[:3, :3] / scale
    images = sorted(rec.images.values(), key=lambda im: im.name)
    save_cameras(root, rec, [im.name for im in images],
                 [turn @ im.cam_from_world().rotation.matrix().T for im in images],
                 [to_lidar[:3, :3] @ im.projection_center() + to_lidar[:3, 3] for im in images])


def similarity(source, target):
    """Best scale, rotation and shift taking source points onto target points (Umeyama). Returns 4x4."""
    ms, mt = source.mean(0), target.mean(0)
    a, b = source - ms, target - mt
    u, d, vt = np.linalg.svd(b.T @ a / len(a))
    sign = np.diag([1, 1, np.sign(np.linalg.det(u @ vt))])
    rot = u @ sign @ vt
    scale = np.trace(np.diag(d) @ sign) / (a ** 2).sum(1).mean()
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = scale * rot, mt - scale * rot @ ms
    return out


def rotation_axis(rot):
    """Axis times angle (radians) of a rotation matrix."""
    angle = np.arccos(np.clip((np.trace(rot) - 1) / 2, -1, 1))
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    norm = np.linalg.norm(axis)
    return axis / norm * angle if norm > 1e-9 else np.zeros(3)


def nearest_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt


def cameras_from_rig(rec, frame_times, video_start, poses_file):
    """Camera poses for video frames shot on the same rig as the lidar.

    A reconstruction from a rig that mostly turns on the spot gets camera directions right but not
    depths or sizes, so only its directions are used:
      - Direction of each camera: the photo reconstruction's, turned into the lidar's world. The turn
        between the two worlds comes from matching the axes the rig rotated about in both.
      - Position of each camera: the lidar's measured position at that moment, plus the camera's fixed
        offset on the rig (solved here; it only needs to be right to a few cm).
    Returns (names, rotations, centres, report dict).
    """
    from scipy.spatial.transform import Rotation, Slerp
    poses = np.loadtxt(poses_file, delimiter=",", skiprows=1)
    turn_at = Slerp(poses[:, 0], Rotation.from_matrix(poses[:, 4:13].reshape(-1, 3, 3)))
    images = [im for im in sorted(rec.images.values(), key=lambda im: im.name)
              if im.name in frame_times and poses[0, 0] < video_start + frame_times[im.name] < poses[-1, 0]]
    n = len(images)
    if n < 10:
        sys.exit("Too few video frames overlap the lidar recording in time.")
    times = np.array([video_start + frame_times[im.name] for im in images])
    cam_pos = np.array([im.projection_center() for im in images])
    cam_rot = np.array([im.cam_from_world().rotation.matrix().T for im in images])
    lidar_rot = turn_at(times).as_matrix()
    lidar_pos = np.column_stack([np.interp(times, poses[:, 0], poses[:, k]) for k in (1, 2, 3)])

    axes_cam, axes_lidar = [], []
    for gap in (4, 8, 16, 32):
        for i in range(n - gap):
            a = rotation_axis(cam_rot[i + gap] @ cam_rot[i].T)
            b = rotation_axis(lidar_rot[i + gap] @ lidar_rot[i].T)
            if min(np.linalg.norm(a), np.linalg.norm(b)) > np.radians(8):
                axes_cam.append(a)
                axes_lidar.append(b)
    if len(axes_cam) < 10:
        sys.exit("The rig did not turn enough during the recording to line the video up with the lidar.")
    world = nearest_rotation(np.array(axes_lidar).T @ np.array(axes_cam))
    rotations = np.array([world @ r for r in cam_rot])
    mount = nearest_rotation(np.mean([lidar_rot[i].T @ rotations[i] for i in range(n)], axis=0))
    wobble = np.degrees([np.linalg.norm(rotation_axis(mount.T @ lidar_rot[i].T @ rotations[i])) for i in range(n)])

    # camera offset on the rig:  scale * world @ cam_i + shift - lidar_rot_i @ offset = lidar_pos_i
    rows = np.zeros((3 * n, 7))
    rows[:, 0] = (cam_pos @ world.T).ravel()
    rows[:, 1:4] = np.tile(np.eye(3), (n, 1))
    rows[:, 4:7] = -lidar_rot.reshape(3 * n, 3)
    solution, *_ = np.linalg.lstsq(rows, lidar_pos.ravel(), rcond=None)
    offset = solution[4:7]
    if np.linalg.norm(offset) > 0.5:      # implausible for a handheld rig: fall back to "at the lidar"
        offset = np.zeros(3)
    centres = lidar_pos + np.einsum("nij,j->ni", lidar_rot, offset)
    report = {"pairs": len(axes_cam), "wobble_median_deg": float(np.median(wobble)),
              "wobble_p90_deg": float(np.percentile(wobble, 90)), "offset_m": offset.tolist()}
    return [im.name for im in images], rotations, centres, report


def place_by_path(args, rec, root, video, frame_times):
    import json
    sidecar = os.path.splitext(video)[0] + ".phone.json"
    if not os.path.exists(sidecar):
        sys.exit("Missing %s (written by livox_record.py --phone); it holds the video's start time." % sidecar)
    info = json.load(open(sidecar))
    video_start = info["first_frame_time"] + info["clock_offset_s"]
    names, rotations, centres, report = cameras_from_rig(rec, frame_times, video_start, args.poses)
    print("Placed %d frames along the lidar's path (%d turn pairs matched)" % (len(names), report["pairs"]))
    print("Camera sits %.0f cm from the lidar's centre: (%+.2f, %+.2f, %+.2f) m in the lidar's own axes" % (
        100 * np.linalg.norm(report["offset_m"]), *report["offset_m"]))
    print("Camera direction vs lidar direction disagree by %.1f degrees typically, %.1f at worst-tenth" % (
        report["wobble_median_deg"], report["wobble_p90_deg"]))
    if report["wobble_median_deg"] > 2:
        print("That is a lot: either the phone moves relative to the lidar on the rig, or one of the two "
              "motion estimates is off. Expect colours to be offset by several cm per metre of distance.")
    save_cameras(root, rec, names, rotations, centres)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("photo_folder")
    ap.add_argument("cloud", help="the stitched lidar cloud (.ply)")
    ap.add_argument("--name", default="photos", help="working folder name under photos/")
    ap.add_argument("--fps", type=float, default=2.0, help="frames per second to take from a video")
    ap.add_argument("--poses", help="for a video recorded with the lidar (livox_record.py --phone): the "
                    "stitcher's -poses.csv. The cameras are then placed along the lidar's own path.")
    args = ap.parse_args()

    root = os.path.join(HERE, "photos", args.name)
    source = os.path.expanduser(args.photo_folder)
    is_video = os.path.isfile(source)
    if is_video:
        frame_times = prepare_video(source, os.path.join(root, "images"), args.fps)
        count = len(frame_times)
    else:
        frame_times = {}
        count = prepare_images(source, os.path.join(root, "images"))
    print("%d photos ready in %s" % (count, os.path.join(root, "images")))
    rec = reconstruct(root, sequential=is_video)
    print("Photo reconstruction: %d of %d photos located, %d 3D points" % (
        rec.num_reg_images(), count, rec.num_points3D()))

    if args.poses:
        return place_by_path(args, rec, root, source, frame_times)

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
    save_cameras_from_fit(root, rec, to_lidar)
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
