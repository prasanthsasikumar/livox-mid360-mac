#!/usr/bin/env python3
"""Drive a RICOH THETA Z1 over USB from the computer (via gphoto2).

    python3 livox_theta.py status          # model, battery, free space, camera clock
    python3 livox_theta.py shoot [DIR]     # take one 360 photo and download it (default: recordings/)

livox_record.py --theta uses this to take 360 photos during a lidar recording.

Needs gphoto2 (brew install gphoto2) and the camera plugged in by USB, switched on, in still-photo mode.
macOS grabs a newly plugged camera for its own photo import (ptpcamerad); this stops that service
each time before talking to the camera. It restarts on its own, which does no harm.
"""
import os
import re
import subprocess
import sys
import time


class ThetaError(RuntimeError):
    pass


def _free_camera():
    for proc in ("ptpcamerad", "mscamerad-xpc"):
        subprocess.run(["pkill", "-9", "-f", proc], capture_output=True)


class Theta:
    def __init__(self):
        out = self._run(["--auto-detect"], port=False)
        found = [line for line in out.splitlines() if "theta" in line.lower()]
        if not found:
            raise ThetaError("no THETA found on USB; is it plugged in and switched on?")
        self.port = found[0].split()[-1]
        self.model = re.sub(r"\s+usb:\S+$", "", found[0].strip()).strip()

    def _run(self, args, port=True, timeout=60):
        cmd = ["gphoto2"] + (["--port", self.port] if port else []) + args
        for attempt in range(3):
            _free_camera()
            try:
                r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
            except FileNotFoundError:
                raise ThetaError("gphoto2 not found; install it with: brew install gphoto2")
            if r.returncode == 0:
                return r.stdout
            if "claim" not in r.stderr.lower():
                break
            time.sleep(0.5)
        raise ThetaError((r.stderr or r.stdout).strip().splitlines()[-1] if (r.stderr or r.stdout) else "gphoto2 failed")

    def get(self, key):
        out = self._run(["--get-config", key])
        m = re.search(r"^Printable: (.*)$", out, re.M) or re.search(r"^Current: (.*)$", out, re.M)
        return m.group(1).strip() if m else None

    def sync_clock(self):
        """Set the camera clock to this computer's, so photo timestamps line up with the lidar file."""
        self._run(["--set-config", "/main/settings/datetime=now"])

    def status(self):
        return {"model": self.model, "port": self.port,
                "battery": self.get("/main/status/batterylevel"),
                "clock": self.get("/main/settings/datetime")}

    def shoot(self, path):
        """Take one photo and download it to `path` (the camera's copy is deleted).
        Returns the computer times when the shot was triggered and when the download finished."""
        t0 = time.time()
        self._run(["--capture-image-and-download", "--force-overwrite", "--filename", path], timeout=90)
        t1 = time.time()
        if not os.path.exists(path):
            raise ThetaError("capture reported success but %s was not written" % path)
        return {"file": os.path.basename(path), "triggered": t0, "downloaded": t1}


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    try:
        cam = Theta()
        if cmd == "status":
            for k, v in cam.status().items():
                print("%-8s %s" % (k, v))
        elif cmd == "shoot":
            out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(out_dir, "theta-%s.jpg" % time.strftime("%Y%m%d-%H%M%S"))
            cam.sync_clock()
            info = cam.shoot(path)
            print("%s (%.1f s)" % (path, info["downloaded"] - info["triggered"]))
        else:
            print(__doc__)
            return 1
    except ThetaError as e:
        print("THETA: %s" % e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
