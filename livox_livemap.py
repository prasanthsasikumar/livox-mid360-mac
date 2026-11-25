"""Live colouring for livox_live.py: paint lidar points with the THETA's live 360 video, and build a
coloured map while the rig moves.

Colouring needs theta-calibration.json (written by livox_pano.py): where the THETA sits on the rig and
which way it faces, in the lidar's own axes. The THETA levels its video by gravity, so the camera's
orientation in the lidar's axes comes from that calibration plus which way is up.

The map tracks the rig with KISS-ICP (the same points-only method livox_stitch.py uses by default) on
0.1 s frames as they arrive, colours each frame from the video frame taken at the same moment, and keeps
one coloured point per 3 cm cube. Start it with the rig standing still for a second: that sets 'up'.
"""
import collections
import json
import os
import struct
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NEAR = 0.45          # metres: nearer points are the rig or the person holding it
FLOOR_CUT = -62.0    # degrees below the camera's horizon where the rig itself fills the picture
FRAME_SECONDS = 0.1
MIN_RANGE, MAX_RANGE = 0.3, 60.0
MAP_VOXEL = 0.03
MAP_MAX_POINTS = 3_000_000
POINT_DTYPE = {1: (np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1"), ("tag", "u1")]), 0.001),
               2: (np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("r", "u1"), ("tag", "u1")]), 0.01)}


def load_calibration(path=os.path.join(HERE, "theta-calibration.json")):
    if not os.path.exists(path):
        return None
    cal = json.load(open(path))
    return {"offset": np.array(cal["offset_lidar"], float), "forward": np.array(cal["forward_lidar"], float),
            "mirror": bool(cal.get("mirror", False))}


def camera_axes(cal, up):
    """Rows: the camera's forward, left and up directions in lidar coordinates."""
    up = up / np.linalg.norm(up)
    f = cal["forward"] - up * (cal["forward"] @ up)
    f /= np.linalg.norm(f)
    return np.stack([f, np.cross(up, f), up])


def colour_points(xyz, image, cal, up, occlusion=True):
    """Colours (n x 3 uint8 RGB) and a seen mask for lidar-frame points `xyz` (metres) from an equirectangular
    BGR `image`, with `up` the up direction in lidar coordinates."""
    n = len(xyz)
    rgb = np.zeros((n, 3), np.uint8)
    seen = np.zeros(n, bool)
    if n == 0 or image is None:
        return rgb, seen
    height, width = image.shape[:2]
    d = (xyz - cal["offset"]) @ camera_axes(cal, up).T
    rng = np.linalg.norm(d, axis=1)
    lon = np.arctan2(d[:, 1], d[:, 0])
    if cal["mirror"]:
        lon = -lon
    lat = np.arcsin(np.clip(d[:, 2] / np.maximum(rng, 1e-9), -1, 1))
    keep = (rng > NEAR) & (lat > np.radians(FLOOR_CUT))
    idx = np.flatnonzero(keep)
    fu = (0.5 - lon[keep] / (2 * np.pi)) % 1.0
    fv = np.clip(0.5 - lat[keep] / np.pi, 0, 0.9999)
    if occlusion and len(idx):
        bw, bh = 512, 256      # coarse depth buffer: a point hidden behind a nearer one takes no colour
        bu, bv = (fu * bw).astype(int) % bw, (fv * bh).astype(int)
        buf = np.full(bw * bh, np.inf)
        np.minimum.at(buf, bv * bw + bu, rng[keep])
        buf = cv2.erode(buf.reshape(bh, bw), np.ones((3, 3), np.uint8))
        vis = rng[keep] <= buf[bv, bu] * 1.05 + 0.1
        idx, fu, fv = idx[vis], fu[vis], fv[vis]
    u = (fu * width).astype(int) % width
    v = (fv * height).astype(int)
    rgb[idx] = image[v, u, ::-1]
    seen[idx] = True
    return rgb, seen


def even_lenses(image, strip=6):
    """The THETA's live video exposes its two lenses separately, leaving a step in brightness at the seams
    (a quarter and three quarters of the way across). Scale the middle half to match the outer half."""
    height, width = image.shape[:2]
    rows = slice(int(height * 0.1), int(height * 0.8))
    img = image.astype(np.float32)
    q = width // 4
    ratios = []
    for outer, inner in ((img[rows, q - 2 * strip:q - strip], img[rows, q + strip:q + 2 * strip]),
                         (img[rows, 3 * q + strip:3 * q + 2 * strip], img[rows, 3 * q - 2 * strip:3 * q - strip])):
        o, i = outer.mean(2), inner.mean(2)
        ok = (o > 15) & (o < 245) & (i > 10)
        if ok.sum() > 50:
            ratios.append(np.median(o[ok] / i[ok]))
    # only when both seams agree: a real exposure step, not just different things either side
    if len(ratios) < 2 or not (min(ratios) > 1.05 or max(ratios) < 0.95):
        return image
    gain = float(np.clip(np.mean(ratios), 0.5, 2.5))
    img[:, q:3 * q] *= gain
    return np.clip(img, 0, 255).astype(np.uint8)


class Frames:
    """Recent video frames with the time each arrived, decoded on demand."""

    def __init__(self, seconds=4.0):
        self.items = collections.deque()
        self.seconds = seconds
        self.lock = threading.Lock()
        self._cache = (None, None)

    def add(self, t, jpeg):
        with self.lock:
            self.items.append((t, jpeg))
            while self.items and self.items[0][0] < t - self.seconds:
                self.items.popleft()

    def latest_time(self):
        with self.lock:
            return self.items[-1][0] if self.items else 0.0

    def at(self, t):
        """The decoded frame that arrived closest to time t (or None)."""
        with self.lock:
            if not self.items:
                return None
            best = min(self.items, key=lambda item: abs(item[0] - t))
        if self._cache[0] is not best[1]:
            image = cv2.imdecode(np.frombuffer(best[1], np.uint8), cv2.IMREAD_COLOR)
            self._cache = (best[1], even_lenses(image) if image is not None else None)
        return self._cache[1]


def level_rotation(up):
    """Rotation taking `up` to +z."""
    a = up / np.linalg.norm(up)
    v, c = np.cross(a, [0, 0, 1]), a[2]
    if c < -0.999:
        return np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


class Mapper:
    """Tracks the rig with KISS-ICP and builds a coloured map, live."""

    def __init__(self, frames, cal, video_delay=0.25):
        self.frames, self.cal, self.video_delay = frames, cal, video_delay
        self.packets = collections.deque()
        self.lock = threading.Lock()
        self.running = False
        self.status = "off"
        self.reset()

    def reset(self):
        with self.lock:
            self.chunks_xyz, self.chunks_rgb, self.count = [], [], 0
            self.voxels = set()
            self.pose = np.eye(4)
            self.track = []
            self.generation = getattr(self, "generation", 0) + 1
        self._odometry = None
        self._acc = []
        self.packets.clear()

    def feed_packet(self, received, data):
        if self.running:
            self.packets.append((received, data))

    def feed_imu(self, acc):
        if self.running and self._odometry is None:
            self._acc.append(acc)

    def start(self):
        if not self.running:
            self.reset()
            self.running = True
            self.status = "hold still…"
            threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False
        self.status = "paused"

    def _new_odometry(self):
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        config = KISSConfig()
        config.data.min_range, config.data.max_range, config.data.deskew = 0.0, 100.0, True
        config.mapping.voxel_size = 0.15
        return KissICP(config)

    def _frames(self):
        """Group packets into 0.1 s frames: (xyz metres, per-point lidar seconds, mean arrival time)."""
        xyz, stamps, arrivals, start = [], [], [], None
        while self.running:
            if not self.packets:
                time.sleep(0.005)
                continue
            received, data = self.packets.popleft()
            if len(data) <= 36 or data[10] not in POINT_DTYPE:
                continue
            dtype, scale = POINT_DTYPE[data[10]]
            interval, dot_num = struct.unpack_from("<HH", data, 3)
            t0 = struct.unpack_from("<Q", data, 28)[0] * 1e-9
            if start is None:
                start = t0
            if t0 - start >= FRAME_SECONDS and xyz:
                yield np.concatenate(xyz), np.concatenate(stamps), float(np.mean(arrivals))
                xyz, stamps, arrivals, start = [], [], [], t0
            body = data[36:]
            pts = np.frombuffer(body, dtype=dtype, count=len(body) // dtype.itemsize)
            p = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64) * scale
            times = t0 + np.arange(len(pts)) * (interval * 1e-7 / max(dot_num, 1))
            dist = np.linalg.norm(p, axis=1)
            keep = (dist > MIN_RANGE) & (dist < MAX_RANGE)
            xyz.append(p[keep]); stamps.append(times[keep]); arrivals.append(received)

    def _loop(self):
        generation = self.generation
        # 'up' from the motion sensor while the rig stands still
        began = time.time()
        while self.running and (len(self._acc) < 100 or time.time() - began < 1.0):
            time.sleep(0.05)
        if not self.running:
            return
        level = np.eye(4)
        level[:3, :3] = level_rotation(np.mean(self._acc, axis=0))
        self._odometry = self._new_odometry()
        self.packets.clear()
        self.status = "mapping"
        for xyz, stamps, arrived in self._frames():
            if generation != self.generation:
                return
            if len(xyz) < 500:
                continue
            # skip ahead if tracking falls behind (frames older than 1 s)
            if time.time() - arrived > 1.0 and len(self.packets) > 500:
                continue
            relative = (stamps - stamps[0]) / FRAME_SECONDS
            deskewed, _ = self._odometry.register_frame(xyz, relative)
            deskewed = np.asarray(deskewed)
            pose = level @ self._odometry.last_pose
            if self.cal is None or len(deskewed) != len(xyz):
                continue
            # wait for the video frame taken when these points were (frames arrive `video_delay` late)
            want = arrived + self.video_delay
            while self.running and self.frames.latest_time() < want and time.time() - want < 1.0:
                time.sleep(0.01)
            image = self.frames.at(want)
            up_lidar = pose[:3, :3].T @ np.array([0.0, 0.0, 1.0])
            rgb, seen = colour_points(deskewed, image, self.cal, up_lidar)
            world = deskewed[seen] @ pose[:3, :3].T + pose[:3, 3]
            rgb = rgb[seen]
            keys = np.floor(world / MAP_VOXEL).astype(np.int64)
            keys = (keys[:, 0] * 73856093) ^ (keys[:, 1] * 19349663) ^ (keys[:, 2] * 83492791)
            _, first = np.unique(keys, return_index=True)
            with self.lock:
                if generation != self.generation:
                    return
                fresh = [i for i in first if keys[i] not in self.voxels]
                if fresh and self.count < MAP_MAX_POINTS:
                    fresh = np.array(fresh)
                    self.voxels.update(keys[fresh].tolist())
                    self.chunks_xyz.append(world[fresh].astype(np.float32))
                    self.chunks_rgb.append(rgb[fresh])
                    self.count += len(fresh)
                self.pose = pose
                self.track.append(pose[:3, 3].astype(np.float32))
            behind = time.time() - arrived
            self.status = "mapping · %.0fk points · %.1f m from start%s" % (
                self.count / 1000, float(np.linalg.norm(pose[:3, 3])),
                " · %.1f s behind" % behind if behind > 0.6 else "")

    def save(self, out_dir):
        """Write the map as a coloured .ply (and a viewer page next to it). Returns the .ply path or None."""
        import subprocess
        import sys
        import livox_room
        with self.lock:
            if not self.chunks_xyz:
                return None
            xyz, rgb = np.concatenate(self.chunks_xyz), np.concatenate(self.chunks_rgb)
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "live-%s-map-colour.ply" % time.strftime("%Y%m%d-%H%M%S"))
        livox_room.write_ply(path, xyz, np.zeros(len(xyz), np.uint8), rgb)
        subprocess.Popen([sys.executable, os.path.join(HERE, "livox_view.py"), path, "--no-open"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.Popen([sys.executable, os.path.join(HERE, "livox_splat.py"), path],   # -> <map>-splat.ply
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return path

    def since(self, chunk, generation):
        """New map points after `chunk` (or all, if the map was reset since `generation`)."""
        with self.lock:
            if generation != self.generation:
                chunk = 0
            xyz = self.chunks_xyz[chunk:]
            rgb = self.chunks_rgb[chunk:]
            return (len(self.chunks_xyz), self.generation, self.pose[:3, 3].copy(),
                    np.concatenate(xyz) if xyz else np.zeros((0, 3), np.float32),
                    np.concatenate(rgb) if rgb else np.zeros((0, 3), np.uint8))
