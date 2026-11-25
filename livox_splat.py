#!/usr/bin/env python3
"""Turn a photo-coloured lidar cloud into a Gaussian splat file (.ply) that splat viewers open.

    .venv/bin/python livox_splat.py scans/live-<time>-map-colour.ply     # -> scans/live-<time>-map-colour-splat.ply

Each lidar point becomes one small flat Gaussian: centred on the point, coloured from the photos,
lying flat on the surface around it (its normal from the nearest points), and as wide as the gap to
its neighbours, so neighbouring splats just overlap and close the holes between points.

This is a starting splat, not a trained one: nothing here is optimised against the photos, so it is
as sharp as the point cloud, no sharper. It is in the standard 3D Gaussian Splatting layout, so it
opens in SuperSplat (https://superspl.at/editor, drag the file in) and other viewers, and it can be
the starting point for training (gsplat, nerfstudio) when there are posed photos to train against.
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

import livox_room

SH_C0 = 0.28209479177387814   # colour = 0.5 + SH_C0 * f_dc
NEIGHBOURS = 8
MIN_SIZE, MAX_SIZE = 0.004, 0.10   # metres: splat radius limits
FLAT = 0.15                        # thickness relative to width


def normals_and_spacing(xyz):
    """Surface normal (smallest spread direction of the nearest points) and gap to neighbours."""
    tree = cKDTree(xyz)
    dist, idx = tree.query(xyz, k=NEIGHBOURS + 1, workers=-1)
    spacing = dist[:, 1:].mean(1)
    nb = xyz[idx] - xyz[idx].mean(1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / NEIGHBOURS
    _, vecs = np.linalg.eigh(cov)            # ascending: column 0 is the normal
    return vecs, spacing


def quaternions(rot):
    """(w, x, y, z) from rotation matrices whose columns are the splat's axes."""
    rot = rot * np.sign(np.linalg.det(rot))[:, None, None]   # proper rotations only
    m = rot
    w = np.sqrt(np.maximum(0, 1 + m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2])) / 2
    x = np.sqrt(np.maximum(0, 1 + m[:, 0, 0] - m[:, 1, 1] - m[:, 2, 2])) / 2
    y = np.sqrt(np.maximum(0, 1 - m[:, 0, 0] + m[:, 1, 1] - m[:, 2, 2])) / 2
    z = np.sqrt(np.maximum(0, 1 - m[:, 0, 0] - m[:, 1, 1] + m[:, 2, 2])) / 2
    x = np.copysign(x, m[:, 2, 1] - m[:, 1, 2])
    y = np.copysign(y, m[:, 0, 2] - m[:, 2, 0])
    z = np.copysign(z, m[:, 1, 0] - m[:, 0, 1])
    q = np.stack([w, x, y, z], 1)
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def write_splat(path, xyz, rgb, status=True):
    vecs, spacing = normals_and_spacing(xyz)
    # splat axes: two in the surface (columns 2 and 1), the normal last, so scale_2 is the thin one
    axes = np.stack([vecs[:, :, 2], vecs[:, :, 1], vecs[:, :, 0]], axis=2)
    size = np.clip(spacing * 0.7, MIN_SIZE, MAX_SIZE)
    scale = np.log(np.stack([size, size, size * FLAT], 1))
    rot = quaternions(axes)
    f_dc = (rgb.astype(np.float32) / 255.0 - 0.5) / SH_C0
    opacity = np.full(len(xyz), 3.0)          # sigmoid(3) = 0.95

    names = ["x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2", "opacity",
             "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3"]
    data = np.column_stack([xyz, np.zeros((len(xyz), 3)), f_dc, opacity, scale, rot]).astype("<f4")
    header = "ply\nformat binary_little_endian 1.0\nelement vertex %d\n%send_header\n" % (
        len(xyz), "".join("property float %s\n" % n for n in names))
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(data.tobytes())
    if status:
        print("Wrote %s (%d splats, median radius %.1f cm, %.0f MB)" % (
            path, len(xyz), 100 * np.median(size), os.path.getsize(path) / 1e6))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cloud", help="a coloured .ply from livox_live.py Save or livox_pano.py")
    ap.add_argument("-o", "--out", help="output path (default: <cloud>-splat.ply)")
    args = ap.parse_args()
    xyz, _ = livox_room.read_ply(args.cloud)
    rgb = livox_room.read_ply_rgb(args.cloud)
    if rgb is None:
        sys.exit("%s has no colours; use a photo-coloured cloud" % args.cloud)
    write_splat(args.out or os.path.splitext(args.cloud)[0] + "-splat.ply", xyz, rgb)


if __name__ == "__main__":
    main()
