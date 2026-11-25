#!/usr/bin/env python3
"""Paint a lidar cloud with colours from photos that have been located in it.

    .venv/bin/python livox_colour.py scans/<name>-stitched.ply --photos photos/office

`--photos` is a folder prepared by livox_photos.py: images/, sfm/0 (the photo
reconstruction) and sfm_to_lidar.npy (where that reconstruction sits in the scan).

Each lidar point is projected into every photo. A photo only colours a point
it can actually see: points hidden behind nearer lidar points are skipped.
Where several photos see a point, they are blended, favouring the photo that
has it nearest its centre. Points no photo sees keep a grey from reflectivity.
Writes <name>-colour.ply and <name>-colour.html.
"""
import argparse
import os
import subprocess
import sys

import cv2
import numpy as np
import pycolmap

import livox_room

HERE = os.path.dirname(os.path.abspath(__file__))


def colour_points(xyz, photos, room_turn=None, buffer_scale=0.25, status=True):
    """Return (rgb uint8 N x 3, seen bool N) for points given in the lidar frame.

    room_turn: if the points are in the wall-aligned frame written by livox_room.py,
    pass its rotation so they can be turned back first.
    """
    rec = pycolmap.Reconstruction(os.path.join(photos, "sfm", "0"))
    to_lidar = np.load(os.path.join(photos, "sfm_to_lidar.npy"))
    scale = np.cbrt(np.linalg.det(to_lidar[:3, :3]))
    to_sfm = np.linalg.inv(to_lidar)
    pts = np.asarray(xyz, np.float64)
    if room_turn is not None:
        pts = pts @ room_turn            # rows: inverse of (x @ turn.T)
    pts = pts @ to_sfm[:3, :3].T + to_sfm[:3, 3]

    total = np.zeros((len(pts), 3))
    weight = np.zeros(len(pts))
    for image in sorted(rec.images.values(), key=lambda im: im.name):
        cam = rec.cameras[image.camera_id]
        f, cx, cy, k = cam.params                      # SIMPLE_RADIAL
        pose = image.cam_from_world()
        c = pts @ pose.rotation.matrix().T + pose.translation
        z = c[:, 2]
        front = z > 0.05 / scale
        x, y = c[front, 0] / z[front], c[front, 1] / z[front]
        r2 = x * x + y * y
        u, v = f * x * (1 + k * r2) + cx, f * y * (1 + k * r2) + cy
        inside = (u >= 0) & (u < cam.width - 1) & (v >= 0) & (v < cam.height - 1) & (r2 < 4)
        idx = np.flatnonzero(front)[inside]
        u, v, depth, r2 = u[inside], v[inside], z[idx], r2[inside]

        # Depth buffer from the lidar points themselves, at reduced resolution, to find what is hidden.
        bw, bh = int(cam.width * buffer_scale), int(cam.height * buffer_scale)
        bu, bv = (u * buffer_scale).astype(int), (v * buffer_scale).astype(int)
        buf = np.full(bh * bw, np.inf)
        np.minimum.at(buf, bv * bw + bu, depth)
        buf = buf.reshape(bh, bw)
        buf = cv2.erode(buf, np.ones((5, 5), np.uint8))    # nearest depth in a 5x5 patch closes gaps between points
        visible = depth <= buf[bv, bu] * 1.04 + 0.08 / scale

        picture = cv2.imread(os.path.join(photos, "images", image.name))
        if picture is None:
            continue
        if picture.shape[1] != cam.width:
            picture = cv2.resize(picture, (cam.width, cam.height), interpolation=cv2.INTER_AREA)
        sample = picture[v[visible].round().astype(int), u[visible].round().astype(int), ::-1]
        w = 1.0 / (1.0 + r2[visible]) ** 2                  # favour the photo's centre
        total[idx[visible]] += sample * w[:, None]
        weight[idx[visible]] += w
        if status:
            sys.stdout.write("\r%s: coloured %d points   " % (image.name, visible.sum()))
            sys.stdout.flush()
    seen = weight > 0
    rgb = np.zeros((len(pts), 3), np.uint8)
    rgb[seen] = np.clip(total[seen] / weight[seen, None], 0, 255).astype(np.uint8)
    if status:
        print("\r%.0f%% of points were seen by at least one photo        " % (100 * seen.mean()))
    return rgb, seen


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cloud")
    ap.add_argument("--photos", required=True)
    ap.add_argument("--room-of", help="for a -room.ply: the stitched cloud it was cut from, to recover its rotation")
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()

    xyz, intensity = livox_room.read_ply(args.cloud)
    turn = livox_room.analyse(livox_room.read_ply(args.room_of)[0])[0] if args.room_of else None
    rgb, seen = colour_points(xyz, args.photos, turn)
    grey = (40 + 215 * (np.minimum(intensity, 160) / 160.0) ** 0.6).astype(np.uint8)
    rgb[~seen] = grey[~seen, None]
    base = os.path.splitext(args.cloud)[0] + "-colour"
    livox_room.write_ply(base + ".ply", xyz, intensity, rgb)
    print("Wrote %s.ply" % base)
    cmd = [sys.executable, os.path.join(HERE, "livox_view.py"), base + ".ply"] + (["--no-open"] if args.no_open else [])
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
