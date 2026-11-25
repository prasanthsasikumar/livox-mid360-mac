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


def refine_directions(root, cloud, passes=3, samples=60000, scale=0.25):
    """Nudge each camera's direction so its picture agrees with what the other cameras see.

    Lidar points are projected into every frame (hidden ones skipped); each point's reference brightness
    is the median over the frames that see it, and each frame is turned by a fraction of a degree to
    best match those references. Rewrites cameras.npz. Returns (before, after) brightness spread.
    """
    import cv2
    from scipy.optimize import minimize
    from scipy.spatial.transform import Rotation
    path = os.path.join(root, "cameras.npz")
    cams = dict(np.load(path))
    f, cx, cy, k = cams["intrinsics"]
    width, height = (int(v) for v in cams["size"])
    w, h = int(width * scale), int(height * scale)
    names, rot, centre = [str(n) for n in cams["names"]], cams["rotations"].copy(), cams["centres"]
    grey = [cv2.GaussianBlur(cv2.cvtColor(cv2.resize(cv2.imread(os.path.join(root, "images", n)), (w, h),
            interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY).astype(np.float32), (7, 7), 2.0) for n in names]
    xyz, _ = livox_room.read_ply(cloud)
    rng = np.random.default_rng(1)
    dist = np.linalg.norm(xyz - centre.mean(0), axis=1)
    pool = xyz[(dist > 0.8) & (dist < 10)]
    pts = pool[rng.choice(len(pool), min(samples, len(pool)), replace=False)]
    full = xyz[rng.choice(len(xyz), min(len(xyz), 800000), replace=False)]

    def project(r, c, p):
        q = (p - c) @ r
        z = q[:, 2]
        front = z > 0.3
        zz = np.where(front, z, 1)
        x, y = q[:, 0] / zz, q[:, 1] / zz
        r2 = x * x + y * y
        u, v = (f * x * (1 + k * r2) + cx) * scale, (f * y * (1 + k * r2) + cy) * scale
        return u, v, z, front & (u >= 1) & (u < w - 2) & (v >= 1) & (v < h - 2) & (r2 < 3)

    def sample(img, u, v):
        x0, y0 = u.astype(int), v.astype(int)
        ax, ay = u - x0, v - y0
        return (img[y0, x0] * (1 - ax) * (1 - ay) + img[y0, x0 + 1] * ax * (1 - ay)
                + img[y0 + 1, x0] * (1 - ax) * ay + img[y0 + 1, x0 + 1] * ax * ay)

    seen = []
    for i in range(len(names)):
        u, v, z, ok = project(rot[i], centre[i], full)
        buf = np.full((h, w), np.inf, np.float32)
        np.minimum.at(buf, (v[ok].astype(int), u[ok].astype(int)), z[ok].astype(np.float32))
        buf = cv2.erode(buf, np.ones((3, 3), np.uint8))
        u, v, z, ok = project(rot[i], centre[i], pts)
        vis = np.zeros(len(pts), bool)
        vis[ok] = z[ok] <= buf[v[ok].astype(int), u[ok].astype(int)] * 1.05 + 0.05
        seen.append(vis)

    def table():
        vals = np.full((len(names), len(pts)), np.nan, np.float32)
        for i in range(len(names)):
            u, v, z, ok = project(rot[i], centre[i], pts)
            ok &= seen[i]
            if ok.any():
                vals[i, ok] = sample(grey[i], u[ok], v[ok])
        return vals

    def spread(vals):
        keep = np.sum(~np.isnan(vals), 0) >= 5
        return float(np.mean(np.nanstd(vals[:, keep], 0)))

    vals = table()
    before = spread(vals)
    for _ in range(passes):
        counts = np.sum(~np.isnan(vals), 0)
        ref = np.full(len(pts), np.nan, np.float32)
        ref[counts >= 4] = np.nanmedian(vals[:, counts >= 4], 0)
        for i in range(len(names)):
            sel = seen[i] & ~np.isnan(ref)
            if sel.sum() < 300:
                continue
            p, r = pts[sel], ref[sel]

            def cost(q):
                u, v, z, ok = project(Rotation.from_rotvec(q).as_matrix() @ rot[i], centre[i], p)
                if ok.sum() < 200:
                    return 1e3
                e = sample(grey[i], u[ok], v[ok]) - r[ok]
                return float(np.mean(np.minimum(np.abs(e - np.median(e)), 40)))   # ignore brightness offsets

            q = minimize(cost, np.zeros(3), method="Powell",
                         options={"xtol": 2e-4, "ftol": 1e-3, "maxfev": 200}).x
            if np.degrees(np.linalg.norm(q)) < 3:
                rot[i] = Rotation.from_rotvec(q).as_matrix() @ rot[i]
        vals = table()
    cams["rotations"] = rot
    np.savez(path, **cams)
    return before, spread(vals)


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
    print("Lidar and photo estimates of each turn differ by %.1f degrees typically (the camera directions "
          "used come from the photos, which are the steadier of the two)" % report["wobble_median_deg"])
    save_cameras(root, rec, names, rotations, centres)
    before, after = refine_directions(root, args.cloud)
    print("Refined each frame's direction: brightness disagreement between frames %.1f -> %.1f grey levels" % (
        before, after))


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
