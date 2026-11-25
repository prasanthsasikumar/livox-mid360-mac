# Livox Mid-360 on a Mac: from first packet to a room mesh

Small Python scripts that talk to a Livox Mid-360 lidar directly over Ethernet, with no Livox SDK and no ROS.
They were written in one sitting on a MacBook to go from "is the sensor alive?" to a cleaned 3D mesh of a room.

**[Open the interactive 3D viewers](https://prasanthsasikumar.github.io/livox-mid360-mac/)** (drag to orbit, scroll to zoom).

[![Room mesh with the ceiling hidden](docs/images/room-mesh-open.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/room-mesh.html?ceiling=off)

The example is a hall of about 10 m by 21.5 m with a 3.1 m ceiling. It was scanned in 75 seconds by lifting
and tilting the lidar around a table top; the lidar never left the table area.

| Stitched cloud, coloured by height | Mesh, seen from the lidar's position |
|---|---|
| [![Stitched point cloud](docs/images/cloud-orbit.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/cloud.html) | [![Inside the mesh](docs/images/room-mesh-inside.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/room-mesh.html?view=in) |

| Coloured from 26 iPhone photos | Seen from above |
|---|---|
| [![Photo-coloured cloud from inside](docs/images/colour-cloud-in.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/colour-cloud.html?view=in) | [![Photo-coloured cloud from above](docs/images/colour-cloud-orbit.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/colour-cloud.html) |

| Top-down density of the stitched cloud | Cleaned room cloud from above |
|---|---|
| ![Top-down view](docs/images/stitched-top-down.png) | [![Room cloud from above](docs/images/room-cloud-top.png)](https://prasanthsasikumar.github.io/livox-mid360-mac/room-cloud.html?view=top) |

## What you need

![The whole setup: a MacBook, an Ethernet adapter and the Mid-360 on a desk](docs/images/desk-setup.jpg)

- A Livox Mid-360, powered, connected to the computer by Ethernet (a USB or Thunderbolt adapter is fine).
- The computer's wired port set to a fixed address on the lidar's network. The scripts assume `192.168.1.50`
  for the computer and `192.168.1.119` for the lidar; pass `--host` and `--lidar` if yours differ. A factory
  Mid-360 is at `192.168.1.1XX`, where `XX` is the last two digits of its serial number.
- Python 3.9 or newer. The first three scripts use only the standard library. The rest need the packages in
  `requirements.txt`:

  ```sh
  python3 -m venv .venv
  .venv/bin/pip install -r requirements.txt
  ```

Developed and run on macOS (Apple silicon). Nothing is Mac-specific except the `open` command used to launch
the viewer, which you can skip with `--no-open`.

## The scripts, in the order you would use them

| Script | What it does | Needs |
|---|---|---|
| `livox_probe.py` | Tells the lidar where to send data, starts it, and counts points and motion-sensor packets for a few seconds. A health check. | standard library |
| `livox_cloud.py` | Captures a few seconds with the lidar standing still and writes a 3D viewer, a `.ply` and a top-down `.png`. | standard library |
| `livox_record.py` | Saves every raw point and motion-sensor packet to a file while you move the lidar. | standard library |
| `livox_stitch.py` | Works out how the lidar moved during a recording and merges all scans into one cloud. | `requirements.txt` |
| `livox_room.py` | Crops a stitched cloud to the room the lidar was in, adds a flat floor, flattens walls and ceiling, removes stray points. | `requirements.txt` |
| `livox_mesh.py` | Turns a cloud into a surface mesh and writes a mesh viewer. | `requirements.txt` |
| `livox_view.py` | Makes a point-cloud viewer from any `.ply` these scripts wrote. | `requirements.txt` |
| `livox_photos.py` | Works out where a set of ordinary photos was taken inside a stitched cloud. | `requirements.txt` |
| `livox_colour.py` | Paints the lidar points with the colours from those photos. | `requirements.txt` |

### 1. Check the sensor

```sh
python3 livox_probe.py
```

A healthy Mid-360 reports about 200,000 points per second, 200 motion-sensor packets per second, an
acceleration close to 1 g when it is sitting still, and roughly 23 Mbit/s of data. Add `--standby` to stop the
motor afterwards.

The Mid-360 sends nothing until a program tells it where to send. The destination address is written into the
lidar, so the next computer you plug it into has to set its own.

### 2. A still scan

```sh
python3 livox_cloud.py --seconds 3
```

Output goes to `scans/`. Three seconds gives about 470,000 points. Keep the lidar still: nothing in this script
corrects for movement.

### 3. Record while moving

```sh
python3 livox_record.py --seconds 75
```

The status line counts down a few seconds of `STAND STILL`, then says `WALK`. Recordings go to `recordings/`
at about 170 MB per minute. They hold the raw packets, so one recording can be re-stitched with different
software or settings later.

How to move:

- Start with the lidar resting on something, untouched, for the still phase.
- Move slowly. Take 2–3 seconds for a quarter turn, and go through doorways slowly.
- Hold it by the base, keep hands and body away from the dark window, and keep other people out of view.
- Changing height and tilting it gently (up to about 30°) is what lets it see the floor and fill the blind
  cone directly above it.
- Finish where you started. The stitcher prints the start-to-end gap, which is then your drift.

### 4. Stitch

```sh
.venv/bin/python livox_stitch.py            # newest recording
.venv/bin/python livox_stitch.py --method lio
```

Two methods are available:

- `icp` (default) uses [KISS-ICP](https://github.com/PRBonn/kiss-icp): points only.
- `lio` uses [RKO-LIO](https://github.com/PRBonn/rko_lio): points plus the motion sensor. It needs the lidar to
  be completely still at the start.

On the one moving recording made so far, `icp` gave the sharper map, and that recording did not have a still
start, so this is not a fair verdict on `lio`. Neither method has loop closure, so long walks will drift.

After stitching, the map is levelled on its largest flat horizontal surface, so it comes out upright even if
the lidar was in your hand at the start.

### 5. Crop to the room and clean up

```sh
.venv/bin/python livox_room.py
```

It finds the strongest wall on each side of the lidar and treats that rectangle as the room. If it picks the
wrong walls, give them yourself with `--box XMIN XMAX YMIN YMAX`. Flattening moves every point within 10 cm of
a wall or 18 cm of the ceiling onto that surface, which removes sensor noise and also any real detail of that
size. `--close` additionally fills in the ceiling and all four walls, covering real doorways and windows.

### 6. Mesh

```sh
.venv/bin/python livox_mesh.py scans/<name>-stitched-room.ply --smooth 8 --trim 0.02
```

Poisson surface reconstruction with Open3D, trimmed back to where there are real points, with small loose
pieces removed. The viewer has a "Hide ceiling" button for looking into the room from above.

### 7. Colour from photos (optional)

```sh
.venv/bin/python livox_photos.py "~/Downloads/office images" scans/<name>-stitched.ply --name office
.venv/bin/python livox_colour.py scans/<name>-stitched.ply --photos photos/office
```

The lidar has no camera, so colour comes from separate photos, here 26 iPhone ultra-wide shots.
`livox_photos.py` reconstructs the camera positions from the photos alone with
[COLMAP](https://colmap.github.io/) (about 10 minutes for 26 photos), then fits that reconstruction onto the
lidar cloud: up from the floor, scale from an assumed camera height of 1–2 m, heading and position by sliding
one floor plan over the other, then a fine 3D fit. It prints how good the fit is; in the example 68% of the
photo points landed within 8 cm of a lidar surface.

`livox_colour.py` then projects every lidar point into every photo and takes the colour, skipping points that
are hidden behind nearer ones. Points no photo sees stay grey. The viewer gains a "Colour: photo" button.

The example photo set is in `photos/office/images`, with its reconstruction in `photos/office/sfm` and the
fitted placement in `photos/office/sfm_to_lidar.npy`. The full coloured result (2.9 million points with RGB) is
`examples/office-colour.ply`, which opens in CloudCompare or MeshLab, or with `livox_view.py`.

![The 26 photos used for colour](docs/images/photo-set.jpg)

Photo tips: heavy overlap between neighbouring shots, one lens and no zoom changes, even light, and some floor
in view. Walking around gives the best coverage; a handheld spin on the spot also worked here, and colours
only what is visible from that spot (54% of the points in the example).

## What to expect from the results

- Walls, floor, ceiling and large furniture come out well and to scale.
- The Mid-360's range noise is 2–3 cm, so surfaces are correct in shape and a little lumpy. Small objects and
  anything close to the lidar turn into blobs.
- A scan from one area only sees what is in line of sight from there. In the example, between 49% and 82% of
  each wall was seen; the gaps are behind furniture. Walking the room is the fix.
- There is no colour. The lidar measures distance and reflectivity only.

## Notes on the protocol

The command frame, checksums, parameter keys and ports were read from
[Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) (`sdk_core/comm/sdk_protocol.cpp`,
`sdk_core/comm/define.h`, `include/livox_lidar_def.h`):

- Commands go to UDP port 56100 on the lidar. Points arrive on 56301 and motion-sensor data on 56401.
- A frame is a 24-byte header (start byte `0xAA`, length, sequence number, command id, CRC-16/CCITT-FALSE of
  the first 18 bytes, CRC-32 of the payload) followed by a list of key/value parameters.
- Command `0x0100` writes parameters: keys `0x0005` to `0x0007` set where status, points and motion-sensor
  data are sent, and key `0x001A` sets the work mode (1 = scanning, 2 = standby).
- Each data packet has a 36-byte header with the lidar's own nanosecond timestamp, then 96 points of 14 bytes
  each (x, y, z in millimetres, reflectivity, tag).

The viewers load [three.js](https://threejs.org/) from a CDN, so they need an internet connection.

## Known limits

- Tested with one Mid-360 on one Mac.
- The lidar's clock is not synchronised to anything; timestamps count from power-on.
- The mesher runs Poisson reconstruction on a single thread, because the multi-threaded version in Open3D 0.20
  crashed on this data.
