#!/usr/bin/env python3
"""Make an interactive point-cloud viewer (.html) from any cloud .ply written by these scripts.

    .venv/bin/python livox_view.py scans/walk-...-stitched-room.ply
    .venv/bin/python livox_view.py cloud.ply --max-points 600000 -o docs/cloud.html
"""
import argparse
import base64
import os
import subprocess

import numpy as np

import livox_cloud
import livox_room


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cloud")
    ap.add_argument("-o", "--output")
    ap.add_argument("--title")
    ap.add_argument("--max-points", type=int, default=1500000, help="randomly thin larger clouds to this many")
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()

    xyz, refl = livox_room.read_ply(args.cloud)
    if len(xyz) > args.max_points:
        pick = np.random.default_rng(0).choice(len(xyz), args.max_points, replace=False)
        xyz, refl = xyz[pick], refl[pick]
    out = args.output or os.path.splitext(args.cloud)[0] + ".html"
    title = args.title or os.path.splitext(os.path.basename(out))[0]
    z_lo, z_hi = np.percentile(xyz[:, 2], [1, 99])
    radius = float(np.percentile(np.abs(xyz[:, :2]), 99)) * 1.1
    html = (livox_cloud.VIEWER.replace("__TITLE__", title).replace("__COUNT__", "{:,}".format(len(xyz)))
            .replace("__ZLO__", "%.3f" % z_lo).replace("__ZHI__", "%.3f" % z_hi).replace("__RADIUS__", "%.3f" % radius)
            .replace("__XYZ__", base64.b64encode(np.ascontiguousarray(xyz, np.float32).tobytes()).decode())
            .replace("__REFL__", base64.b64encode(np.ascontiguousarray(refl, np.uint8).tobytes()).decode()))
    with open(out, "w") as f:
        f.write(html)
    print("Wrote %s (%s points)" % (out, "{:,}".format(len(xyz))))
    if not args.no_open:
        subprocess.run(["open", out])


if __name__ == "__main__":
    main()
