#!/usr/bin/env python3
"""Drive the LidarCam iPhone app (phone-app/) from this computer.

    python3 livox_phone.py status
    python3 livox_phone.py start test-1 [--lens ultrawide]
    python3 livox_phone.py stop
    python3 livox_phone.py fetch test-1.mov recordings/
    python3 livox_phone.py list

The phone and this computer must be on the same Wi-Fi network with the app open.
The phone is found by its network name (Bonjour); pass --phone HOST:PORT or set
LIDARCAM_PHONE to skip that.
"""
import argparse
import json
import os
import re
import select
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

PORT = 8765


def find(timeout=6.0):
    """Return 'host:port' for the phone, via LIDARCAM_PHONE or Bonjour."""
    if os.environ.get("LIDARCAM_PHONE"):
        return os.environ["LIDARCAM_PHONE"]
    # dns-sd never exits on its own: read until it reports the address, then stop it.
    proc = subprocess.Popen(["dns-sd", "-L", "LidarCam", "_lidarcam._tcp", "local"],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    out, deadline = "", time.time() + timeout
    try:
        # read the pipe directly: a buffered reader would hide lines from select()
        while time.time() < deadline and not re.search(r"can be reached at \S+:\d+", out):
            if select.select([proc.stdout], [], [], 0.2)[0]:
                chunk = os.read(proc.stdout.fileno(), 4096)
                if not chunk:
                    break
                out += chunk.decode(errors="replace")
    finally:
        proc.kill()
    match = re.search(r"can be reached at (\S+?)\.?:(\d+)", out)
    if not match:
        raise RuntimeError("LidarCam not found on the network. Is the app open, on the same Wi-Fi, "
                           "and was 'local network' access allowed on the phone?")
    return "%s:%s" % (match.group(1), match.group(2))


class Phone:
    def __init__(self, address=None):
        self.address = address or find()

    def call(self, method, path, timeout=20, **query):
        url = "http://%s%s" % (self.address, path)
        if query:
            url += "?" + urllib.parse.urlencode({k: v for k, v in query.items() if v is not None})
        request = urllib.request.Request(url, method=method, data=b"" if method == "POST" else None)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as error:
            return json.loads(error.read() or b'{"error": "request failed"}')

    def status(self):
        return self.call("GET", "/status")

    def clock_offset(self, tries=5):
        """Seconds to ADD to a phone timestamp to get this computer's clock."""
        best = None
        for _ in range(tries):
            sent = time.time()
            phone_time = self.status()["phone_time"]
            back = time.time()
            if best is None or back - sent < best[0]:
                best = (back - sent, (sent + back) / 2 - phone_time)
        return best[1], best[0]

    def start(self, name, lens=None, shutter=None, focus=None):
        return self.call("POST", "/start", name=name, lens=lens, shutter=shutter, focus=focus)

    def stop(self):
        return self.call("POST", "/stop", timeout=60)

    def files(self):
        return self.call("GET", "/clips").get("files", [])

    def delete(self, filename):
        return self.call("POST", "/delete", file=filename)

    def fetch(self, filename, folder, progress=True):
        os.makedirs(folder, exist_ok=True)
        target = os.path.join(folder, filename)
        url = "http://%s/clips/%s" % (self.address, urllib.parse.quote(filename))
        with urllib.request.urlopen(url, timeout=30) as response, open(target, "wb") as out:
            total = int(response.headers.get("Content-Length", 0))
            done, began = 0, time.time()
            while True:
                chunk = response.read(1 << 20)
                if not chunk:
                    break
                out.write(chunk)
                done += len(chunk)
                if progress:
                    sys.stdout.write("\rfetching %s: %.0f of %.0f MB" % (filename, done / 1e6, total / 1e6))
                    sys.stdout.flush()
        if progress:
            print("  (%.0f s)" % (time.time() - began))
        if total and done != total:
            raise RuntimeError("download of %s was cut short" % filename)
        return target


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--phone", help="HOST:PORT, instead of finding it on the network")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("list")
    sub.add_parser("stop")
    start = sub.add_parser("start")
    start.add_argument("name")
    start.add_argument("--lens", choices=["wide", "ultrawide"])
    start.add_argument("--shutter", type=float, help="longest exposure per frame in seconds (default 0.005)")
    fetch = sub.add_parser("fetch")
    fetch.add_argument("file")
    fetch.add_argument("folder", nargs="?", default="recordings")
    args = ap.parse_args()

    phone = Phone(args.phone)
    if args.command == "status":
        status = phone.status()
        offset, round_trip = phone.clock_offset()
        status["clock_offset_s"], status["round_trip_s"] = round(offset, 4), round(round_trip, 4)
        print(json.dumps(status, indent=2))
    elif args.command == "list":
        print("\n".join(phone.files()))
    elif args.command == "start":
        print(json.dumps(phone.start(args.name, args.lens, args.shutter), indent=2))
    elif args.command == "stop":
        print(json.dumps(phone.stop(), indent=2))
    elif args.command == "fetch":
        print(phone.fetch(args.file, args.folder))


if __name__ == "__main__":
    main()
