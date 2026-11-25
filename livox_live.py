#!/usr/bin/env python3
"""Live view: the Mid-360 point cloud and the THETA 360 video side by side in a browser.

    .venv/bin/python livox_live.py               # opens http://localhost:8360
    .venv/bin/python livox_live.py --no-theta    # lidar only

The lidar is started and its points are kept for a short rolling window (default 0.5 s, adjustable in the
page). The THETA is put into its USB live-streaming mode, where macOS sees it as a webcam; ffmpeg reads it and
the page shows it as MJPEG (Chrome opens the THETA's webcam stream but never gets a frame). Press the
camera's Mode button to get back to photo mode for `livox_record.py --theta`.

Photo colour paints the live points with the THETA's video, and Map builds a coloured map while you walk
(see livox_livemap.py; both need theta-calibration.json from livox_pano.py).

Only one program can receive the lidar's points at a time: stop this before running livox_record.py.
"""
import argparse
import collections
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import livox_livemap
import livox_probe as lp

POINT_DTYPE = {1: (np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1"), ("tag", "u1")]), 0.1),
               2: (np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("r", "u1"), ("tag", "u1")]), 1.0)}
MAX_WINDOW = 3.0  # seconds of points kept on the server


class Lidar:
    def __init__(self, host, lidar):
        self.host, self.lidar = host, lidar
        self.packets = collections.deque()  # (time, xyz int16 cm (n,3), reflectivity (n,))
        self.lock = threading.Lock()
        self.rate = 0.0
        self.imu = None
        self.up = None        # low-passed acceleration direction: 'up' in lidar coordinates while still
        self.error = None
        self.on_packet = None  # callbacks for the mapper
        self.on_imu = None

    def start(self):
        socks = []
        try:
            for port in (lp.HOST_CMD_PORT, lp.HOST_POINT_PORT, lp.HOST_IMU_PORT, lp.HOST_PUSH_PORT):
                socks.append(lp.bind(self.host, port))
        except OSError as e:
            for sk in socks:
                sk.close()
            self.error = ("no network address %s: is the lidar's Ethernet adapter plugged in?" % self.host
                          if e.errno == 49 else "cannot listen on %s (%s); is livox_record.py running?" % (self.host, e))
            return False
        cmd, self.points, self.imu_sock, self._push = socks
        targets = lp.key_values([
            (lp.KEY_STATE_HOST, lp.host_target(self.host, lp.HOST_PUSH_PORT, lp.LIDAR_PUSH_PORT)),
            (lp.KEY_POINT_HOST, lp.host_target(self.host, lp.HOST_POINT_PORT, lp.LIDAR_POINT_PORT)),
            (lp.KEY_IMU_HOST, lp.host_target(self.host, lp.HOST_IMU_PORT, lp.LIDAR_IMU_PORT)),
        ])
        if not (lp.send_command(cmd, self.lidar, 1, targets, "set data destinations") and
                lp.send_command(cmd, self.lidar, 2, lp.key_values([(lp.KEY_WORK_MODE, bytes([lp.MODE_NORMAL]))]),
                                "start scanning")):
            for sk in socks:
                sk.close()
            self.error = "lidar at %s did not answer; is it powered?" % self.lidar
            return False
        threading.Thread(target=self._points_loop, daemon=True).start()
        threading.Thread(target=self._imu_loop, daemon=True).start()
        return True

    def _points_loop(self):
        count, since = 0, time.time()
        while True:
            data = self.points.recv(2048)
            if len(data) < 36 or data[10] not in POINT_DTYPE:
                continue
            dtype, to_cm = POINT_DTYPE[data[10]]
            body = data[36:]
            p = np.frombuffer(body, dtype=dtype, count=len(body) // dtype.itemsize)
            p = p[(p["x"] != 0) | (p["y"] != 0) | (p["z"] != 0)]
            xyz = (np.stack([p["x"], p["y"], p["z"]], axis=1) * to_cm).astype(np.int16)
            now = time.time()
            if self.on_packet:
                self.on_packet(now, data)
            with self.lock:
                self.packets.append((now, xyz, p["r"].copy()))
                while self.packets and self.packets[0][0] < now - MAX_WINDOW:
                    self.packets.popleft()
            count += len(p)
            if now - since >= 1.0:
                self.rate, count, since = count / (now - since), 0, now

    def _imu_loop(self):
        while True:
            data = self.imu_sock.recv(2048)
            if len(data) >= 60:
                values = np.frombuffer(data[36:60], "<f4").astype(float)
                self.imu = [round(v, 4) for v in values.tolist()]
                acc = values[3:6]
                self.up = acc if self.up is None else 0.98 * self.up + 0.02 * acc
                if self.on_imu:
                    self.on_imu(acc)

    def snapshot(self, seconds):
        cutoff = time.time() - seconds
        with self.lock:
            chunks = [(x, r) for t, x, r in self.packets if t >= cutoff]
        if not chunks:
            return np.zeros((0, 3), np.int16), np.zeros(0, np.uint8)
        return np.concatenate([c[0] for c in chunks]), np.concatenate([c[1] for c in chunks])


def theta_webcam_present():
    out = subprocess.run(["system_profiler", "SPCameraDataType"], capture_output=True, text=True).stdout
    return "THETA" in out


def theta_to_live():
    """Put a USB THETA into live-streaming mode (it then shows up as a webcam). Returns an error or None."""
    try:
        import livox_theta
        livox_theta.Theta()._run(["--set-config-value", "/main/other/5013=32773"])
    except Exception as e:  # noqa: BLE001 - camera off, unplugged, busy...
        return "THETA not found on USB (%s); switch it on and plug it in" % e
    for _ in range(10):
        time.sleep(1)
        if theta_webcam_present():
            return None
    return "THETA did not come back as a webcam after switching to live mode"


class ThetaFeed:
    """Reads the THETA webcam with ffmpeg and keeps the latest frame as a JPEG for any number of viewers."""

    def __init__(self, name="RICOH THETA Z1", size="1920x960", fps=15, quality=5):
        self.cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "avfoundation", "-framerate", "29.97",
                    "-video_size", size, "-i", name, "-r", str(fps), "-q:v", str(quality),
                    "-f", "image2pipe", "-c:v", "mjpeg", "pipe:1"]
        self.size = size
        self.jpeg, self.seq, self.fps, self.error = None, 0, 0.0, None
        self.frames = livox_livemap.Frames()   # recent frames with arrival times, for colouring
        self.cond = threading.Condition()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            if not theta_webcam_present():
                self.error = "THETA is not streaming; switching it to live mode…"
                err = theta_to_live()
                if err:
                    self.error = err
                    time.sleep(3)
                    continue
            try:
                proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except FileNotFoundError:
                self.error = "ffmpeg not found (brew install ffmpeg)"
                return
            buf, count, since = b"", 0, time.time()
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:  # split the stream on JPEG start/end markers
                    a = buf.find(b"\xff\xd8")
                    b = buf.find(b"\xff\xd9", a + 2)
                    if a < 0 or b < 0:
                        break
                    with self.cond:
                        self.jpeg, self.seq, self.error = buf[a:b + 2], self.seq + 1, None
                        self.cond.notify_all()
                    self.frames.add(time.time(), self.jpeg)
                    buf = buf[b + 2:]
                    count += 1
                now = time.time()
                if now - since >= 1.0:
                    self.fps, count, since = count / (now - since), 0, now
            err = proc.stderr.read().decode(errors="replace").strip().splitlines()
            proc.wait()
            self.fps = 0.0
            self.error = "THETA feed stopped: %s; retrying (is it in LIVE mode?)" % (err[-1] if err else "no frames")
            time.sleep(2)

    def wait_frame(self, last_seq, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.jpeg

    def status(self):
        w, h = self.size.split("x")
        return {"fps": self.fps, "w": int(w), "h": int(h), "error": self.error}


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Livox Live</title>
<style>
  :root { --bg:#0b0d12; --panel:#121620; --fg:#cfd6e4; --muted:#7d879c; --line:#262c3c; --accent:#2f6fed; }
  html, body { margin:0; height:100%; background:var(--bg); color:var(--fg);
    font:13px/1.4 -apple-system, system-ui, sans-serif; overflow:hidden; }
  main { display:grid; grid-template-columns:1fr 1fr; height:100%; gap:1px; background:var(--line); }
  @media (max-width: 900px) { main { grid-template-columns:1fr; grid-template-rows:1fr 1fr; } }
  section { position:relative; background:var(--bg); overflow:hidden; min-height:0; }
  canvas, #video { display:block; width:100%; height:100%; }
  #video { object-fit:contain; background:#000; }
  .hud { position:absolute; left:12px; top:10px; right:12px; pointer-events:none; }
  .hud > * { pointer-events:auto; }
  .hud b { color:#fff; }
  .stat { color:var(--muted); font-variant-numeric:tabular-nums; }
  .row { margin-top:6px; display:flex; gap:6px; flex-wrap:wrap; align-items:center; }
  button, select { background:#1b2030; color:var(--fg); border:1px solid #343b50; border-radius:6px;
    padding:4px 9px; font:inherit; cursor:pointer; }
  button.on { background:var(--accent); border-color:var(--accent); color:#fff; }
  input[type=range] { width:110px; }
  .err { color:#ff9a8a; margin-top:6px; }
</style>
<script type="importmap">{"imports": {
  "three": "https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
  "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
</head><body>
<main>
  <section id="lidar-panel">
    <canvas id="lidar"></canvas>
    <div class="hud">
      <b>Mid-360</b> <span class="stat" id="lstat">connecting…</span>
      <div class="row">
        <button id="c-height" class="on">Height</button>
        <button id="c-refl">Reflectivity</button>
        <button id="c-dist">Distance</button>
        <button id="c-photo" title="Colour the points from the THETA's live video">Photo</button>
        <button id="v-top">Top</button>
        <button id="v-side">Orbit</button>
        <button id="level" class="on" title="Rotate the cloud so gravity (from the lidar's IMU) points down">Level</button>
        <label class="stat">window <input id="win" type="range" min="0.1" max="3" step="0.1" value="0.5">
          <span id="winv">0.5 s</span></label>
        <label class="stat">dot <input id="dot" type="range" min="1" max="6" step="0.5" value="2"></label>
      </div>
      <div class="row">
        <button id="m-map" title="Track the rig and build a coloured map as you walk. Hold still for a second when starting.">Map</button>
        <button id="m-reset">Reset map</button>
        <button id="m-save" title="Write the coloured map to scans/ as .ply and a viewer page">Save</button>
        <span class="stat" id="mstat"></span>
      </div>
      <div class="err" id="lerr"></div>
    </div>
  </section>
  <section id="theta-panel">
    <img id="video" alt="">
    <canvas id="sphere" hidden></canvas>
    <div class="hud">
      <b>THETA Z1</b> <span class="stat" id="tstat">looking for camera…</span>
      <div class="row">
        <button id="m-flat" class="on">Equirectangular</button>
        <button id="m-look">Look around</button>
      </div>
      <div class="err" id="terr"></div>
    </div>
  </section>
</main>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const $ = id => document.getElementById(id);

// ---------- lidar ----------
const lc = $('lidar');
const lr = new THREE.WebGLRenderer({ canvas: lc, antialias: false });
lr.setPixelRatio(devicePixelRatio);
const ls = new THREE.Scene(); ls.background = new THREE.Color(0x0b0d12);
const lcam = new THREE.PerspectiveCamera(60, 1, 0.05, 500);
lcam.up.set(0, 0, 1); lcam.position.set(-8, -8, 7);
const lctl = new OrbitControls(lcam, lc); lctl.enableDamping = true;
ls.add(new THREE.AxesHelper(0.5));
const grid = new THREE.GridHelper(20, 20, 0x2a3142, 0x1a1f2b); grid.rotation.x = Math.PI / 2; grid.position.z = -1; ls.add(grid);
const MAXP = 800000;
const geo = new THREE.BufferGeometry();
const pos = new Float32Array(MAXP * 3), col = new Float32Array(MAXP * 3);
geo.setAttribute('position', new THREE.BufferAttribute(pos, 3).setUsage(THREE.DynamicDrawUsage));
geo.setAttribute('color', new THREE.BufferAttribute(col, 3).setUsage(THREE.DynamicDrawUsage));
const mat = new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true });
const cloud = new THREE.Points(geo, mat); cloud.frustumCulled = false; ls.add(cloud);

let colourBy = 'height';
const turbo = t => [ // polynomial fit of the Turbo colour map
  0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943)))),
  0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604)))),
  0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))))
].map(v => Math.min(1, Math.max(0, v)));
const LUT = Array.from({ length: 256 }, (_, i) => turbo(i / 255));
const MODES = ['height', 'refl', 'dist', 'photo'];
for (const k of MODES) $('c-' + k).onclick = () => {
  colourBy = k; for (const j of MODES) $('c-' + j).classList.toggle('on', j === k);
};

// ---------- coloured map (built on the server while the rig moves) ----------
const MAXM = 3000000;
const mgeo = new THREE.BufferGeometry();
const mpos = new Float32Array(MAXM * 3), mcol = new Float32Array(MAXM * 3);
mgeo.setAttribute('position', new THREE.BufferAttribute(mpos, 3).setUsage(THREE.DynamicDrawUsage));
mgeo.setAttribute('color', new THREE.BufferAttribute(mcol, 3).setUsage(THREE.DynamicDrawUsage));
mgeo.setDrawRange(0, 0);
const mapCloud = new THREE.Points(mgeo, mat); mapCloud.frustumCulled = false; mapCloud.visible = false; ls.add(mapCloud);
const rig = new THREE.Mesh(new THREE.SphereGeometry(0.08, 16, 8), new THREE.MeshBasicMaterial({ color: 0xff5533 }));
rig.visible = false; ls.add(rig);
let mapOn = false, mChunk = 0, mGen = 0, mCount = 0;
$('m-map').onclick = async () => {
  mapOn = !mapOn; $('m-map').classList.toggle('on', mapOn);
  mapCloud.visible = rig.visible = mapOn; cloud.visible = !mapOn;
  await fetch(mapOn ? '/map/start' : '/map/stop', { cache: 'no-store' });
};
$('m-reset').onclick = () => fetch('/map/reset', { cache: 'no-store' });
$('m-save').onclick = async () => {
  const r = await (await fetch('/map/save', { cache: 'no-store' })).json();
  $('mstat').textContent = r.file ? 'saved ' + r.file.split('/').pop() : 'nothing to save yet';
};
async function pullMap() {
  if (mapOn) try {
    const buf = await (await fetch(`/map?c=${mChunk}&g=${mGen}`, { cache: 'no-store' })).arrayBuffer();
    const [n, chunk, gen] = new Uint32Array(buf, 0, 3);
    const p = new Float32Array(buf, 12, 3);
    if (gen !== mGen) { mCount = 0; mGen = gen; }
    mChunk = chunk;
    const xyz = new Float32Array(buf, 24, n * 3), rgb = new Uint8Array(buf, 24 + n * 12, n * 3);
    const m = Math.min(n, MAXM - mCount);
    mpos.set(xyz.subarray(0, m * 3), mCount * 3);
    for (let i = 0; i < m * 3; i++) mcol[mCount * 3 + i] = rgb[i] / 255;
    if (m > 0) {
      const a = mgeo.attributes.position, c = mgeo.attributes.color;
      a.addUpdateRange ? a.addUpdateRange(mCount * 3, m * 3) : 0; c.addUpdateRange ? c.addUpdateRange(mCount * 3, m * 3) : 0;
      a.needsUpdate = c.needsUpdate = true;
    }
    mCount += m; mgeo.setDrawRange(0, mCount);
    rig.position.set(p[0], p[1], p[2]);
  } catch (e) {}
  setTimeout(pullMap, 300);
}
pullMap();
$('v-top').onclick = () => { lcam.position.set(0, -0.01, 18); lctl.target.set(0, 0, 0); };
$('v-side').onclick = () => { lcam.position.set(-8, -8, 7); lctl.target.set(0, 0, 0); };
$('win').oninput = e => $('winv').textContent = (+e.target.value).toFixed(1) + ' s';
$('dot').oninput = e => mat.size = +e.target.value;
let level = true, up = null;
$('level').onclick = () => { level = !level; $('level').classList.toggle('on', level); if (!level) cloud.quaternion.identity(); };
function applyLevel(imu) {
  if (!imu) return;
  const a = new THREE.Vector3(imu[3], imu[4], imu[5]);
  if (a.length() < 0.5) return;
  a.normalize();
  up = up ? up.lerp(a, 0.2).normalize() : a;  // low-pass so walking jitter does not shake the view
  if (level) cloud.quaternion.setFromUnitVectors(up, new THREE.Vector3(0, 0, 1));
}

async function pull() {
  try {
    if (mapOn) { setTimeout(pull, 200); return; }
    const photo = colourBy === 'photo';
    const r = await fetch('/points?s=' + $('win').value + (photo ? '&rgb=1' : ''), { cache: 'no-store' });
    const buf = await r.arrayBuffer();
    const total = new Uint32Array(buf, 0, 1)[0], n = Math.min(total, MAXP);
    const xyz = new Int16Array(buf, 4, n * 3);
    const refl = new Uint8Array(buf, 4 + total * 6, n);
    const rgb = photo ? new Uint8Array(buf, 4 + total * 7, n * 3) : null;
    for (let i = 0; i < n; i++) {
      const x = xyz[3 * i] / 100, y = xyz[3 * i + 1] / 100, z = xyz[3 * i + 2] / 100;
      pos[3 * i] = x; pos[3 * i + 1] = y; pos[3 * i + 2] = z;
      if (rgb) { col[3 * i] = rgb[3 * i] / 255; col[3 * i + 1] = rgb[3 * i + 1] / 255; col[3 * i + 2] = rgb[3 * i + 2] / 255; continue; }
      let t;
      if (colourBy === 'height') t = (z + 1.5) / 4;
      else if (colourBy === 'refl') t = Math.min(1, refl[i] / 150);
      else t = Math.sqrt(x * x + y * y + z * z) / 15;
      const c = LUT[Math.max(0, Math.min(255, (t * 255) | 0))];
      col[3 * i] = c[0]; col[3 * i + 1] = c[1]; col[3 * i + 2] = c[2];
    }
    geo.setDrawRange(0, n);
    geo.attributes.position.needsUpdate = true; geo.attributes.color.needsUpdate = true;
    $('lerr').textContent = '';
  } catch (e) { $('lerr').textContent = 'lost the server: ' + e.message; await new Promise(r => setTimeout(r, 1000)); }
  setTimeout(pull, 50);
}
async function stats() {
  try {
    const s = await (await fetch('/stats', { cache: 'no-store' })).json();
    if (s.error) { $('lerr').textContent = s.error; $('lstat').textContent = ''; }
    else {
      let t = (s.rate / 1000).toFixed(0) + 'k pts/s';
      if (s.imu) { const [, , , ax, ay, az] = s.imu; t += ' · |a| ' + Math.hypot(ax, ay, az).toFixed(2) + ' g'; }
      $('lstat').textContent = t;
      applyLevel(s.imu);
      $('mstat').textContent = s.map === 'off' ? '' : s.map;
      $('c-photo').disabled = !s.calibrated;
      if (!s.calibrated) $('c-photo').title = 'needs theta-calibration.json (run livox_pano.py once)';
    }
  } catch (e) {}
  setTimeout(stats, 250);
}
// ?colour=photo|height|refl|dist  ?map=1  ?view=top|orbit
const opts = new URLSearchParams(location.search);
if (opts.get('colour') && $('c-' + opts.get('colour'))) $('c-' + opts.get('colour')).onclick();
if (opts.get('view') === 'top') $('v-top').onclick();
if (opts.get('map') === '1') $('m-map').onclick();
pull(); stats();

// ---------- THETA ----------
const video = $('video'), sc = $('sphere');
const sr = new THREE.WebGLRenderer({ canvas: sc });
sr.setPixelRatio(devicePixelRatio);
const ss = new THREE.Scene();
const scam = new THREE.PerspectiveCamera(80, 1, 0.1, 100);
scam.position.set(0, 0, 0.01);
const sctl = new OrbitControls(scam, sc);
sctl.enableZoom = false; sctl.enablePan = false; sctl.rotateSpeed = -0.35;
const tc = document.createElement('canvas'); tc.width = 2048; tc.height = 1024;
const tx = tc.getContext('2d');
const tex = new THREE.CanvasTexture(tc); tex.colorSpace = THREE.SRGBColorSpace;
const sphereGeo = new THREE.SphereGeometry(10, 64, 32); sphereGeo.scale(-1, 1, 1);
ss.add(new THREE.Mesh(sphereGeo, new THREE.MeshBasicMaterial({ map: tex })));
sc.addEventListener('wheel', e => { e.preventDefault();
  scam.fov = Math.min(110, Math.max(25, scam.fov + e.deltaY * 0.05)); scam.updateProjectionMatrix(); }, { passive: false });
let look = false;
$('m-flat').onclick = () => { look = false; video.hidden = false; sc.hidden = true;
  $('m-flat').classList.add('on'); $('m-look').classList.remove('on'); };
$('m-look').onclick = () => { look = true; video.hidden = true; sc.hidden = false;
  $('m-look').classList.add('on'); $('m-flat').classList.remove('on'); resize(); };


// The server reads the THETA with ffmpeg and serves it as MJPEG (Chrome cannot decode the THETA's webcam
// stream directly). Reconnect whenever the stream drops.
function connect() { video.src = '/theta.mjpg?' + Date.now(); }
video.onerror = () => setTimeout(connect, 1500);
connect();
async function thetaStats() {
  try {
    const t = (await (await fetch('/stats', { cache: 'no-store' })).json()).theta_feed || {};
    $('tstat').textContent = t.fps ? `${t.w}×${t.h} · ${t.fps.toFixed(0)} fps` : '';
    $('terr').textContent = t.error || '';
  } catch (e) {}
  setTimeout(thetaStats, 1000);
}
thetaStats();

// ---------- render ----------
function resize() {
  for (const [r, c, cam] of [[lr, lc, lcam], [sr, sc, scam]]) {
    const p = c.parentElement, w = p.clientWidth, h = p.clientHeight;
    r.setSize(w, h, false); cam.aspect = w / h; cam.updateProjectionMatrix();
  }
}
addEventListener('resize', resize); resize();
(function loop() {
  lctl.update(); lr.render(ls, lcam);
  if (look) { if (video.naturalWidth) { tx.drawImage(video, 0, 0, tc.width, tc.height); tex.needsUpdate = true; }
    sctl.update(); sr.render(ss, scam); }
  requestAnimationFrame(loop);
})();
</script>
</body></html>
"""


def photo_colours(lidar, feed, mapper, cal, xyz_cm, refl):
    """RGB for the live points from the newest video frame; points the camera cannot see stay grey."""
    grey = (40 + 215 * (np.minimum(refl, 160) / 160.0) ** 0.6).astype(np.uint8)
    rgb = np.repeat(grey[:, None], 3, axis=1)
    up = lidar.up if lidar is not None else None
    if mapper is not None and mapper.status.startswith("mapping"):
        up = mapper.pose[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    if cal is None or feed is None or up is None or not len(refl):
        return rgb
    image = feed.frames.at(time.time())
    colours, seen = livox_livemap.colour_points(xyz_cm.astype(np.float64) / 100.0, image, cal, up)
    rgb[seen] = colours[seen]
    return rgb


def make_handler(lidar, feed, mapper=None, cal=None):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path, _, query = self.path.partition("?")
            if path == "/":
                self._send(PAGE.encode(), "text/html; charset=utf-8")
            elif path == "/points":
                try:
                    secs = min(MAX_WINDOW, max(0.05, float(dict(q.split("=") for q in query.split("&") if "=" in q)
                                                             .get("s", 0.5))))
                except ValueError:
                    secs = 0.5
                xyz, refl = lidar.snapshot(secs) if lidar else (np.zeros((0, 3), np.int16), np.zeros(0, np.uint8))
                body = np.uint32(len(refl)).tobytes() + xyz.tobytes() + refl.tobytes()
                if "rgb=1" in query:
                    body += photo_colours(lidar, feed, mapper, cal, xyz, refl).tobytes()
                self._send(body, "application/octet-stream")
            elif path == "/map":
                q = dict(kv.split("=") for kv in query.split("&") if "=" in kv)
                chunk, gen, pose, xyz, rgb = mapper.since(int(q.get("c", 0)), int(q.get("g", 0)))
                head = np.array([len(rgb), chunk, gen], np.uint32).tobytes() + pose.astype(np.float32).tobytes()
                self._send(head + xyz.astype(np.float32).tobytes() + rgb.tobytes(), "application/octet-stream")
            elif path == "/map/save":
                saved = mapper.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "scans")) if mapper else None
                self._send(json.dumps({"file": saved}).encode(), "application/json")
            elif path in ("/map/start", "/map/stop", "/map/reset"):
                if mapper is None or lidar is None:
                    self.send_error(404)
                    return
                if path == "/map/start":
                    mapper.start()
                elif path == "/map/stop":
                    mapper.stop()
                else:
                    was = mapper.running
                    mapper.stop()
                    mapper.reset()
                    if was:
                        mapper.start()
                self._send(b"{}", "application/json")
            elif path == "/theta.mjpg":
                if feed is None:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        seq, jpeg = feed.wait_frame(seq)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(jpeg) + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return
            elif path == "/stats":
                s = {"theta_feed": feed.status() if feed else {"error": "THETA disabled (--no-theta)"}}
                if lidar is None:
                    s["error"] = "lidar disabled (--no-lidar)"
                elif lidar.error:
                    s["error"] = lidar.error
                else:
                    s.update(rate=lidar.rate, imu=lidar.imu)
                s["map"] = mapper.status if mapper else "off"
                s["calibrated"] = cal is not None
                self._send(json.dumps(s).encode(), "application/json")
            else:
                self.send_error(404)
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lidar", default="192.168.1.119")
    ap.add_argument("--host", default="192.168.1.50", help="this computer's address on the lidar cable")
    ap.add_argument("--port", type=int, default=8360, help="web page port")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-theta", action="store_true", help="lidar only")
    ap.add_argument("--no-open", action="store_true")
    ap.add_argument("--video-delay", type=float, default=0.25,
                    help="seconds the THETA's live video lags behind real time (lines up map colours while moving)")
    args = ap.parse_args()
    cal = livox_livemap.load_calibration()
    if cal is None:
        print("No theta-calibration.json: run livox_pano.py once on a --theta recording to enable photo colour.")

    lidar = None
    if not args.no_lidar:
        lidar = Lidar(args.host, args.lidar)

        def keep_trying():  # the adapter may not be plugged in yet
            while not lidar.start():
                time.sleep(3)
            lidar.error = None
        threading.Thread(target=keep_trying, daemon=True).start()

    feed = None
    if not args.no_theta:
        feed = ThetaFeed()
        feed.start()

    mapper = None
    if lidar is not None:
        mapper = livox_livemap.Mapper(feed.frames if feed else livox_livemap.Frames(), cal, args.video_delay)
        lidar.on_packet, lidar.on_imu = mapper.feed_packet, mapper.feed_imu

    ThreadingHTTPServer.daemon_threads = True
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(lidar, feed, mapper, cal))
    url = "http://localhost:%d" % args.port
    print("Live view at %s  (Ctrl-C to stop)" % url)
    if not args.no_open:
        subprocess.Popen(["open", url])
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
