#!/usr/bin/env python3
"""Capture a few seconds of Mid-360 points and write a 3D viewer for them.

    python3 livox_cloud.py                # 3 second capture, opens the viewer
    python3 livox_cloud.py --seconds 6    # denser cloud

Writes three files into scans/: an interactive .html viewer, a .ply for other
tools (CloudCompare, MeshLab), and a top-down .png preview. Keep the lidar
still while it captures; nothing here corrects for movement.
"""
import argparse
import base64
import math
import os
import select
import struct
import subprocess
import sys
import time
import zlib
from array import array

import livox_probe as lp

VIEWER = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
  html, body { margin: 0; height: 100%; background: #0b0d12; color: #cfd6e4;
    font: 13px/1.4 -apple-system, system-ui, sans-serif; overflow: hidden; }
  #hud { position: fixed; left: 16px; top: 14px; max-width: calc(100% - 32px); }
  #hud b { color: #fff; }
  #hud div { margin-top: 6px; display: flex; gap: 6px; flex-wrap: wrap; }
  button { background: #1b2030; color: #cfd6e4; border: 1px solid #343b50;
    border-radius: 6px; padding: 5px 10px; font: inherit; cursor: pointer; }
  button.on { background: #2f6fed; border-color: #2f6fed; color: #fff; }
  #err { color: #ff9a8a; }
</style>
<script type="importmap">{"imports": {
  "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
  "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<div id="hud"><b>__TITLE__</b> &middot; __COUNT__ points &middot; drag to orbit, scroll to zoom, right-drag to pan
  <div>
    <button id="c-height" class="on">Colour: height</button>
    <button id="c-refl">Colour: reflectivity</button>
    <button id="v-top">Top view</button>
    <button id="v-in">From the lidar</button>
    <button id="s-less">Smaller dots</button>
    <button id="s-more">Bigger dots</button>
  </div>
  <div id="err"></div>
</div>
<script id="xyz" type="application/octet-stream">__XYZ__</script>
<script id="refl" type="application/octet-stream">__REFL__</script>
<script type="module">
window.addEventListener('error', e => document.getElementById('err').textContent = e.message);
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const bytes = id => Uint8Array.from(atob(document.getElementById(id).textContent), c => c.charCodeAt(0));
const pos = new Float32Array(bytes('xyz').buffer);
const refl = bytes('refl');
const n = refl.length, zLo = __ZLO__, zHi = __ZHI__;

const ramp = [[48,18,59],[70,107,227],[27,207,212],[98,252,108],[210,233,53],[254,155,45],[219,56,7],[122,4,3]];
function turbo(t, out, i) {
  t = Math.min(0.999, Math.max(0, t)) * (ramp.length - 1);
  const k = Math.floor(t), f = t - k, a = ramp[k], b = ramp[k + 1];
  for (let c = 0; c < 3; c++) out[i + c] = (a[c] + (b[c] - a[c]) * f) / 255;
}
const col = new Float32Array(n * 3);
function colour(mode) {
  for (let i = 0; i < n; i++) {
    if (mode === 'height') turbo((pos[i * 3 + 2] - zLo) / (zHi - zLo), col, i * 3);
    else { const v = 0.12 + 0.88 * Math.pow(Math.min(refl[i], 160) / 160, 0.6); col[i*3] = col[i*3+1] = col[i*3+2] = v; }
  }
  geo.attributes.color.needsUpdate = true;
  document.getElementById('c-height').classList.toggle('on', mode === 'height');
  document.getElementById('c-refl').classList.toggle('on', mode !== 'height');
}

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, 1, 0.02, 500);
camera.up.set(0, 0, 1);
const geo = new THREE.BufferGeometry();
geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
const mat = new THREE.PointsMaterial({ size: 0.02, vertexColors: true, sizeAttenuation: true });
scene.add(new THREE.Points(geo, mat));
scene.add(new THREE.AxesHelper(0.3));  // the lidar: red = x (forward), green = y, blue = z (up)
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const R = __RADIUS__;
function view(which) {
  if (which === 'top') { camera.position.set(0, -0.01, R * 2.2); controls.target.set(0, 0, 0); }
  else if (which === 'in') { camera.position.set(-0.05, 0, 0.05); controls.target.set(1, 0, 0); }
  else { camera.position.set(-R * 1.1, -R * 1.1, R * 1.0); controls.target.set(0, 0, 0); }
  controls.update();
}
function resize() {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
}
const params = new URLSearchParams(location.search);   // ?view=top|in|orbit  &colour=height|refl
addEventListener('resize', resize); resize();
view(params.get('view') || 'orbit'); colour(params.get('colour') || 'height');
document.getElementById('c-height').onclick = () => colour('height');
document.getElementById('c-refl').onclick = () => colour('refl');
document.getElementById('v-top').onclick = () => view('top');
document.getElementById('v-in').onclick = () => view('in');
document.getElementById('s-less').onclick = () => mat.size = Math.max(0.004, mat.size / 1.4);
document.getElementById('s-more').onclick = () => mat.size = Math.min(0.2, mat.size * 1.4);
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"""


def capture(lidar, host, seconds):
    """Return the raw point packets received in `seconds`."""
    cmd = lp.bind(host, lp.HOST_CMD_PORT)
    points = lp.bind(host, lp.HOST_POINT_PORT)
    targets = lp.key_values([
        (lp.KEY_POINT_HOST, lp.host_target(host, lp.HOST_POINT_PORT, lp.LIDAR_POINT_PORT)),
    ])
    ok = lp.send_command(cmd, lidar, 1, targets, "set point destination")
    ok = ok and lp.send_command(cmd, lidar, 2, lp.key_values([(lp.KEY_WORK_MODE, bytes([lp.MODE_NORMAL]))]),
                                "start scanning")
    if not ok:
        sys.exit(1)

    # If the motor was in standby it needs a moment to spin up: wait for the first packet.
    if not select.select([points], [], [], 15)[0]:
        sys.exit("No points arrived within 15 s.")
    packets = []
    end = time.time() + seconds
    while time.time() < end:
        if select.select([points], [], [], 0.2)[0]:
            packets.append(points.recv(2048))
    return packets


def decode(packets):
    """Raw packets -> (xyz float32 metres, reflectivity uint8), dropping empty returns."""
    xyz, refl = array("f"), array("B")
    for data in packets:
        if len(data) < lp.POINT_HEADER.size:
            continue
        data_type = data[10]
        if data_type == 1:
            fmt, scale = "<iiiBB", 0.001
        elif data_type == 2:
            fmt, scale = "<hhhBB", 0.01
        else:
            continue
        size = struct.calcsize(fmt)
        body = data[lp.POINT_HEADER.size:]
        body = body[:len(body) - len(body) % size]
        for x, y, z, r, _tag in struct.iter_unpack(fmt, body):
            if x or y or z:
                xyz.extend((x * scale, y * scale, z * scale))
                refl.append(r)
    return xyz, refl


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def write_ply(path, xyz, refl):
    n = len(refl)
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\nelement vertex %d\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar intensity\nend_header\n" % n).encode())
        row = struct.Struct("<fffB")
        f.write(b"".join(row.pack(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2], refl[i]) for i in range(n)))


def write_top_png(path, xyz, half, size=900):
    """Top-down density image, lidar at the centre, x (lidar forward) pointing up."""
    grid = [0] * (size * size)
    scale = size / (2 * half)
    for i in range(0, len(xyz), 3):
        col = int((half - xyz[i + 1]) * scale)   # +y is left
        row = int((half - xyz[i]) * scale)       # +x is up
        if 0 <= col < size and 0 <= row < size:
            grid[row * size + col] += 1
    peak = math.log1p(percentile([g for g in grid if g] or [1], 0.995))
    raw = bytearray()
    for row in range(size):
        raw.append(0)
        raw.extend(min(255, int(255 * math.log1p(g) / peak)) for g in grid[row * size:(row + 1) * size])

    def chunk(kind, payload):
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 0, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lidar", default="192.168.1.119")
    ap.add_argument("--host", default="192.168.1.50")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--no-open", action="store_true", help="don't open the viewer in the browser")
    args = ap.parse_args()

    packets = capture(args.lidar, args.host, args.seconds)
    xyz, refl = decode(packets)
    n = len(refl)
    if not n:
        sys.exit("Packets arrived but none contained a usable point.")

    xs, ys, zs = xyz[0::3], xyz[1::3], xyz[2::3]
    lo, hi = 0.01, 0.99
    box = [(percentile(v, lo), percentile(v, hi)) for v in (xs, ys, zs)]
    half = max(abs(b) for pair in box[:2] for b in pair) * 1.1

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")
    os.makedirs(out_dir, exist_ok=True)
    name = "room-" + time.strftime("%Y%m%d-%H%M%S")
    base = os.path.join(out_dir, name)

    write_ply(base + ".ply", xyz, refl)
    write_top_png(base + ".png", xyz, half)
    html = (VIEWER.replace("__TITLE__", name).replace("__COUNT__", "{:,}".format(n))
            .replace("__ZLO__", "%.3f" % box[2][0]).replace("__ZHI__", "%.3f" % box[2][1])
            .replace("__RADIUS__", "%.3f" % half)
            .replace("__XYZ__", base64.b64encode(xyz.tobytes()).decode())
            .replace("__REFL__", base64.b64encode(refl.tobytes()).decode()))
    with open(base + ".html", "w") as f:
        f.write(html)

    print("%d points from %d packets" % (n, len(packets)))
    print("Extent (middle 98%% of points, metres from the lidar):")
    for axis, (a, b) in zip("xyz", box):
        print("  %s: %+.2f to %+.2f  (%.2f m)" % (axis, a, b, b - a))
    print("Wrote %s.html, .ply, .png" % base)
    if not args.no_open:
        subprocess.run(["open", base + ".html"])


if __name__ == "__main__":
    main()
