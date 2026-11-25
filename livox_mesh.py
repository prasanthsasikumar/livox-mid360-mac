#!/usr/bin/env python3
"""Turn a stitched point cloud into a surface mesh.

    .venv/bin/python livox_mesh.py                         # newest stitched cloud
    .venv/bin/python livox_mesh.py scans/walk-...-stitched.ply

Cleans the cloud, estimates which way each surface faces (towards where the
lidar was), runs Poisson surface reconstruction and trims the parts the lidar
never saw. Writes <name>-mesh.ply and a <name>-mesh.html viewer.
"""
import argparse
import base64
import glob
import os
import subprocess
import sys

import numpy as np
import open3d as o3d

HERE = os.path.dirname(os.path.abspath(__file__))

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
<div id="hud"><b>__TITLE__</b> &middot; __COUNT__ triangles &middot; drag to orbit, scroll to zoom, right-drag to pan
  <div>
    <button id="m-solid" class="on">Solid</button>
    <button id="m-height">Colour by height</button>
    <button id="m-wire">Wireframe</button>
    <button id="m-ceil">Hide ceiling</button>
    <button id="v-top">Top view</button>
    <button id="v-in">From the lidar</button>
  </div>
  <div id="err"></div>
</div>
<script id="pos" type="application/octet-stream">__POS__</script>
<script id="idx" type="application/octet-stream">__IDX__</script>
<script type="module">
window.addEventListener('error', e => document.getElementById('err').textContent = e.message);
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const bytes = id => Uint8Array.from(atob(document.getElementById(id).textContent), c => c.charCodeAt(0));
const pos = new Float32Array(bytes('pos').buffer);
const idx = new Uint32Array(bytes('idx').buffer);
const zLo = __ZLO__, zHi = __ZHI__, R = __RADIUS__;

const ramp = [[48,18,59],[70,107,227],[27,207,212],[98,252,108],[210,233,53],[254,155,45],[219,56,7],[122,4,3]];
const col = new Float32Array(pos.length);
for (let i = 0; i < pos.length; i += 3) {
  const t = Math.min(0.999, Math.max(0, (pos[i + 2] - zLo) / (zHi - zLo))) * (ramp.length - 1);
  const k = Math.floor(t), f = t - k;
  for (let c = 0; c < 3; c++) col[i + c] = (ramp[k][c] + (ramp[k + 1][c] - ramp[k][c]) * f) / 255;
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
geo.setIndex(new THREE.BufferAttribute(idx, 1));
geo.computeVertexNormals();
const mat = new THREE.MeshStandardMaterial({ color: 0xc9ccd4, roughness: 0.85, metalness: 0, side: THREE.DoubleSide });
scene.add(new THREE.Mesh(geo, mat));
scene.add(new THREE.AxesHelper(0.3));
scene.add(new THREE.HemisphereLight(0xffffff, 0x30343f, 1.1));
const lamp = new THREE.DirectionalLight(0xffffff, 1.6);
camera.add(lamp); lamp.position.set(0.4, 0.6, 1); scene.add(camera);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

function mode(m) {
  mat.wireframe = m === 'wire';
  mat.vertexColors = m === 'height';
  mat.color.set(m === 'height' ? 0xffffff : 0xc9ccd4);
  mat.needsUpdate = true;
  for (const b of ['solid', 'height', 'wire']) document.getElementById('m-' + b).classList.toggle('on', b === m);
}
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
addEventListener('resize', resize); resize();
view(new URLSearchParams(location.search).get('view') || 'orbit');
for (const b of ['solid', 'height', 'wire']) document.getElementById('m-' + b).onclick = () => mode(b);
renderer.localClippingEnabled = true;
const cut = [new THREE.Plane(new THREE.Vector3(0, 0, -1), zHi - 0.35)];
function ceiling(hide) {
  mat.clippingPlanes = hide ? cut : []; mat.needsUpdate = true;
  document.getElementById('m-ceil').classList.toggle('on', hide);
}
document.getElementById('m-ceil').onclick = () => ceiling(!mat.clippingPlanes.length);
if (new URLSearchParams(location.search).get('ceiling') === 'off') ceiling(true);
document.getElementById('v-top').onclick = () => view('top');
document.getElementById('v-in').onclick = () => view('in');
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cloud", nargs="?")
    ap.add_argument("--voxel", type=float, default=0.03, help="working resolution in metres")
    ap.add_argument("--depth", type=int, default=10, help="Poisson octree depth; higher = finer and slower")
    ap.add_argument("--trim", type=float, default=0.06, help="fraction of least-supported surface to cut away")
    ap.add_argument("--min-piece", type=int, default=3000, help="delete loose pieces smaller than this many triangles")
    ap.add_argument("--smooth", type=int, default=0, help="smoothing passes (Taubin); 0 = none")
    ap.add_argument("--viewer-triangles", type=int, default=700000)
    ap.add_argument("--no-open", action="store_true")
    args = ap.parse_args()

    path = args.cloud or (sorted(glob.glob(os.path.join(HERE, "scans", "*-stitched*.ply")), key=os.path.getmtime)
                          or [None])[-1]
    if not path or path.endswith("-mesh.ply"):
        sys.exit("No stitched cloud found. Run livox_stitch.py first, or pass a cloud .ply.")
    base = os.path.splitext(path)[0]

    cloud = o3d.io.read_point_cloud(path)
    print("Loaded {:,} points".format(len(cloud.points)))
    cloud = cloud.voxel_down_sample(args.voxel)
    cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print("After cleaning: {:,} points at {:.0f} cm spacing".format(len(cloud.points), args.voxel * 100))

    # Surfaces face the lidar: orient every normal towards the nearest point on its path.
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 4, max_nn=40))
    track_file = base + "-path.csv"
    track = np.loadtxt(track_file, delimiter=",", skiprows=1).reshape(-1, 3) if os.path.exists(track_file) else np.zeros((1, 3))
    track_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(track))
    tree = o3d.geometry.KDTreeFlann(track_cloud)
    points, normals = np.asarray(cloud.points), np.asarray(cloud.normals)
    nearest = np.array([track[tree.search_knn_vector_3d(p, 1)[1][0]] for p in points])
    flip = np.einsum("ij,ij->i", normals, nearest - points) < 0
    normals[flip] *= -1
    cloud.normals = o3d.utility.Vector3dVector(normals)

    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        cloud, depth=args.depth, n_threads=1)  # multi-threaded Poisson crashes ("Failed to close loop")
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, args.trim))
    # Poisson invents surface far from any data; keep only what is near real points.
    dist = np.asarray(o3d.geometry.PointCloud(mesh.vertices).compute_point_cloud_distance(cloud))
    mesh.remove_vertices_by_mask(dist > args.voxel * 3)
    mesh.remove_degenerate_triangles()
    labels, sizes, _ = mesh.cluster_connected_triangles()
    mesh.remove_triangles_by_mask(np.asarray(sizes)[np.asarray(labels)] < args.min_piece)
    mesh.remove_unreferenced_vertices()
    if args.smooth:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=args.smooth)
    mesh.compute_vertex_normals()
    print("Mesh: {:,} vertices, {:,} triangles".format(len(mesh.vertices), len(mesh.triangles)))
    o3d.io.write_triangle_mesh(base + "-mesh.ply", mesh)

    small = mesh
    if len(mesh.triangles) > args.viewer_triangles:
        small = mesh.simplify_quadric_decimation(args.viewer_triangles)
    verts = np.asarray(small.vertices, dtype=np.float32)
    tris = np.asarray(small.triangles, dtype=np.uint32)
    z_lo, z_hi = np.percentile(verts[:, 2], [1, 99])
    radius = float(np.percentile(np.abs(verts[:, :2]), 99)) * 1.1
    name = os.path.basename(base) + "-mesh"
    html = (VIEWER.replace("__TITLE__", name).replace("__COUNT__", "{:,}".format(len(tris)))
            .replace("__ZLO__", "%.3f" % z_lo).replace("__ZHI__", "%.3f" % z_hi).replace("__RADIUS__", "%.3f" % radius)
            .replace("__POS__", base64.b64encode(np.ascontiguousarray(verts).tobytes()).decode())
            .replace("__IDX__", base64.b64encode(np.ascontiguousarray(tris).tobytes()).decode()))
    with open(base + "-mesh.html", "w") as f:
        f.write(html)
    print("Wrote %s-mesh.ply and -mesh.html" % base)
    if not args.no_open:
        subprocess.run(["open", base + "-mesh.html"])


if __name__ == "__main__":
    main()
