#!/usr/bin/env python3
"""
RPLIDAR C1  ->  OSC bridge for TouchDesigner   (v2: 4 buttons + brick detection)
==============================================================================

Setup: Samsung The Frame 32" lying flat, LiDAR at the bottom-centre, laser plane
a few mm above the glass. Fingers = touches, a brick on the glass in the middle
= "brick present".

    u = 0 left edge ... 1 right edge
    v = 0 bottom edge (sensor side) ... 1 top edge

OSC (UDP, default 127.0.0.1:7000):
    /lidar/touches   u0 v0 u1 v1 ...      fingers (empty when none)
    /lidar/brick     present u v          present = 1/0, u v = brick centre
    /lidar/hz        scan rate

Only dependency:  pip install pyserial

    python lidar_bridge.py --info        check the sensor
    python lidar_bridge.py --calibrate   touch the 4 buttons, left to right
    python lidar_bridge.py --monitor     run and print touches / brick
    python lidar_bridge.py               run (quiet)
    python lidar_bridge.py --raw         print what the sensor sees (diagnostics)
    python lidar_bridge.py --sim         no sensor: fake taps + brick on/off
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "lidar_config.json")

# ---- UI layout shared with build_touch_test.py (1280x720 design frame) ----
UI_W, UI_H = 1280, 720
N_BUTTONS = 4
BTN_MARGIN, BTN_GAP, BTN_H, BTN_CY = 58, 16, 82, 632
BTN_W = (UI_W - 2 * BTN_MARGIN - (N_BUTTONS - 1) * BTN_GAP) / N_BUTTONS
BUTTON_CENTRES = [((BTN_MARGIN + BTN_W / 2 + i * (BTN_W + BTN_GAP)) / UI_W, 1 - BTN_CY / UI_H)
                  for i in range(N_BUTTONS)]

# Samsung The Frame 32" (LS03): 31.5" viewable, 16:9  ->  about 697 x 392 mm of picture.
# (727.3 x 426.8 mm is the outside of the frame.)
DEFAULT_CFG = {
    "port": "auto",
    "baud": 460800,
    "osc_host": "127.0.0.1",
    "osc_port": 7000,

    "screen_w_mm": 697.3,        # visible picture width
    "screen_h_mm": 392.2,        # visible picture height
    "sensor_x_mm": 348.6,        # starting guess: centred ...
    "sensor_y_mm": -40.0,        # ... 4 cm below the picture. Calibration overwrites these.
    "angle_offset_deg": 0.0,     # calibration
    "mirror": False,             # calibration

    "edge_margin_mm": 8.0,       # ignore hits this close to the picture edge
    "min_dist_mm": 30.0,
    "cluster_gap_mm": 30.0,      # points closer than this are one object
    "min_points": 2,
    "finger_max_mm": 45.0,       # objects wider than this are not fingers (hands, arms, bricks)
    "max_touches": 1,

    # Brick detection zone in u,v (screen fractions): around the brick frame in the UI
    "brick_zone": [0.30, 0.27, 0.70, 0.67],
    "brick_min_mm": 50.0,        # visible face must be at least this wide to count as the brick
    "brick_on_s": 0.6,           # must be seen this long to switch to "present"
    "brick_off_s": 1.0,          # must be gone this long to switch to "absent"

    "bg_seconds": 2.0,
    "bg_tolerance_mm": 40.0,
}


# --------------------------------------------------------------------------- config

def load_cfg():
    cfg = dict(DEFAULT_CFG)
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg.update(json.load(f))
    save_cfg(cfg)                     # adds any new keys to an older config
    return cfg


def save_cfg(cfg):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


# --------------------------------------------------------------------------- OSC

def _osc_str(s):
    b = s.encode("utf-8")
    return b + b"\0" * (4 - len(b) % 4)


def osc_message(address, floats):
    return (_osc_str(address) + _osc_str("," + "f" * len(floats)) +
            b"".join(struct.pack(">f", float(x)) for x in floats))


class OscOut:
    def __init__(self, host, port):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, address, floats=()):
        try:
            self.sock.sendto(osc_message(address, list(floats)), self.addr)
        except OSError as e:
            print(f"[osc] send failed: {e}")


# --------------------------------------------------------------------------- RPLIDAR protocol

CMD_STOP, CMD_SCAN, CMD_INFO, CMD_HEALTH = 0x25, 0x20, 0x50, 0x52
KNOWN_USB = {(0x10C4, 0xEA60): "CP210x", (0x1A86, 0x7523): "CH340", (0x1A86, 0x55D4): "CH9102"}


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"  {p.device:35s} {KNOWN_USB.get((p.vid, p.pid), ''):8s} {p.description}")


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if (p.vid, p.pid) in KNOWN_USB:
            return p.device
    for p in ports:
        if any(k in p.device.lower() for k in ("usbserial", "slab", "wchusb", "ttyusb")):
            return p.device
    return None


class RPLidar:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.ser.dtr = False
        self.buf = bytearray()
        self.flush_request = False

    def _cmd(self, cmd):
        self.ser.write(bytes([0xA5, cmd]))

    def _descriptor(self):
        d = self.ser.read(7)
        if len(d) != 7 or d[0] != 0xA5 or d[1] != 0x5A:
            raise IOError(f"bad response descriptor: {d.hex()}")
        return d[2] | d[3] << 8 | d[4] << 16 | (d[5] & 0x3F) << 24, d[6]

    def stop(self):
        self._cmd(CMD_STOP)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.buf.clear()

    def info(self):
        self.stop()
        self._cmd(CMD_INFO)
        size, _ = self._descriptor()
        r = self.ser.read(size)
        return {"model": r[0], "firmware": f"{r[2]}.{r[1]:02d}", "hardware": r[3],
                "serial": r[4:20].hex().upper()}

    def health(self):
        self.stop()
        self._cmd(CMD_HEALTH)
        size, _ = self._descriptor()
        r = self.ser.read(size)
        return {0: "Good", 1: "Warning", 2: "Error"}.get(r[0], "?"), r[1] | r[2] << 8

    def start_scan(self):
        self.stop()
        self._cmd(CMD_SCAN)
        self._descriptor()
        print("[lidar] scan started - waiting for the motor to spin up...")

    def nodes(self):
        ser, buf = self.ser, self.buf
        last, got_any = time.time(), False
        while True:
            if self.flush_request:
                self.flush_request = False
                ser.reset_input_buffer()
                buf.clear()
            chunk = ser.read(max(5, ser.in_waiting))
            if not chunk:
                limit = 3.0 if got_any else 8.0
                if time.time() - last > limit:
                    raise IOError("no data from LiDAR for %.0fs (motor not spinning?)" % limit)
                continue
            if not got_any:
                print("[lidar] data flowing")
                got_any = True
            last = time.time()
            buf.extend(chunk)
            i, n = 0, len(buf)
            while n - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                s = b0 & 1
                if s == ((b0 >> 1) & 1) or not (b1 & 1):
                    i += 1
                    continue
                yield (bool(s), ((b1 >> 1) | (buf[i + 2] << 7)) / 64.0,
                       (buf[i + 3] | (buf[i + 4] << 8)) / 4.0)
                i += 5
            del buf[:i]

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def scans(lidar):
    pts = []
    for new_scan, a, d in lidar.nodes():
        if new_scan and pts:
            yield pts
            pts = []
        if d > 0:
            pts.append((a, d))


# --------------------------------------------------------------------------- geometry

def to_screen_mm(cfg, a, d):
    """Raw (angle, dist) -> (x, y) mm from the bottom-left corner of the picture."""
    phi = (a - cfg["angle_offset_deg"]) if cfg["mirror"] else (cfg["angle_offset_deg"] - a)
    r = math.radians(phi)
    return cfg["sensor_x_mm"] + d * math.cos(r), cfg["sensor_y_mm"] + d * math.sin(r)


def on_screen(cfg, x, y, margin):
    return margin <= x <= cfg["screen_w_mm"] - margin and margin <= y <= cfg["screen_h_mm"] - margin


class Background:
    """Remembers the nearest thing outside the picture in every 1 deg direction
    (frame, walls, people standing around). Points on the picture are never learned,
    so a brick or a hand left on the glass at startup is still detected."""

    def __init__(self):
        self.bins = [math.inf] * 360

    def learn(self, cfg, pts):
        for a, d in pts:
            x, y = to_screen_mm(cfg, a, d)
            if on_screen(cfg, x, y, -5.0):
                continue
            b = int(a) % 360
            if d < self.bins[b]:
                self.bins[b] = d

    def foreground(self, pts, tol):
        out = []
        for a, d in pts:
            b = int(a) % 360
            bg = min(self.bins[b], self.bins[(b - 1) % 360], self.bins[(b + 1) % 360])
            if d < bg - tol:
                out.append((a, d))
        return out


def cluster(points, gap):
    clusters, cur, prev = [], [], None
    for p in points:
        if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) > gap:
            clusters.append(cur)
            cur = []
        cur.append(p)
        prev = p
    if cur:
        clusters.append(cur)
    # points are in angle order: join the last and first groups across the 0/360 seam
    if len(clusters) > 1:
        a, b = clusters[-1][-1], clusters[0][0]
        if math.hypot(a[0] - b[0], a[1] - b[1]) <= gap:
            clusters[0] = clusters.pop() + clusters[0]
    return clusters


def detect(cfg, fg_pts):
    """Returns (touches [(u,v)], brick_candidate (u, v, width_mm) or None)."""
    W, H = cfg["screen_w_mm"], cfg["screen_h_mm"]
    inside = []
    for a, d in sorted(fg_pts):
        if d < cfg["min_dist_mm"]:
            continue
        x, y = to_screen_mm(cfg, a, d)
        if on_screen(cfg, x, y, cfg["edge_margin_mm"]):
            inside.append((x, y))
    touches, brick = [], None
    z = cfg["brick_zone"]
    for c in cluster(inside, cfg["cluster_gap_mm"]):
        if len(c) < cfg["min_points"]:
            continue
        width = math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1])
        u = sum(p[0] for p in c) / len(c) / W
        v = sum(p[1] for p in c) / len(c) / H
        if width <= cfg["finger_max_mm"]:
            touches.append((len(c), u, v))
        elif width >= cfg["brick_min_mm"] and z[0] <= u <= z[2] and z[1] <= v <= z[3]:
            if brick is None or width > brick[2]:
                brick = (u, v, width)
    touches.sort(reverse=True)
    return [(u, v) for _, u, v in touches[: cfg["max_touches"]]], brick


class BrickState:
    def __init__(self, cfg):
        self.cfg, self.present, self.u, self.v = cfg, False, 0.5, 0.5
        self.seen_since, self.last_seen = None, 0.0

    def update(self, cand, now):
        if cand:
            self.u, self.v = cand[0], cand[1]
            self.last_seen = now
            if self.seen_since is None:
                self.seen_since = now
            if not self.present and now - self.seen_since >= self.cfg["brick_on_s"]:
                self.present = True
        else:
            self.seen_since = None
            if self.present and now - self.last_seen >= self.cfg["brick_off_s"]:
                self.present = False
        return self.present


# --------------------------------------------------------------------------- modes

def open_lidar(cfg):
    if serial is None:
        sys.exit("pyserial is missing:  pip install pyserial")
    port = cfg["port"] if cfg["port"] != "auto" else find_port()
    if not port:
        list_ports()
        sys.exit("LiDAR not found. Set \"port\" in lidar_config.json or pass --port.")
    print(f"[lidar] opening {port} @ {cfg['baud']}")
    lidar = RPLidar(port, cfg["baud"])
    try:
        info = lidar.info()
        status, err = lidar.health()
        print(f"[lidar] model 0x{info['model']:02X}  fw {info['firmware']}  "
              f"health: {status}" + (f" (code {err})" if err else ""))
    except IOError as e:
        print(f"[lidar] info/health failed ({e}) - continuing")
    return lidar


def learn_background(cfg, it):
    bg = Background()
    print(f"[bg] learning the surroundings for {cfg['bg_seconds']:.1f}s - keep people away from the table edge...")
    bg.learn(cfg, next(it))
    t0, n = time.time(), 1
    while time.time() - t0 < cfg["bg_seconds"] or n < 15:
        bg.learn(cfg, next(it))
        n += 1
    print(f"[bg] done ({n} scans)")
    return bg


def run(cfg, monitor=False):
    osc = OscOut(cfg["osc_host"], cfg["osc_port"])
    print(f"[osc] sending to {cfg['osc_host']}:{cfg['osc_port']}")
    while True:
        lidar = None
        try:
            lidar = open_lidar(cfg)
            lidar.start_scan()
            it = scans(lidar)
            bg = learn_background(cfg, it)
            brick = BrickState(cfg)
            last_t, hz, last_hz_send, last_print = time.time(), 0.0, 0.0, ""
            for pts in it:
                now = time.time()
                dt, last_t = now - last_t, now
                if dt > 0:
                    hz = (1 / dt) if hz == 0 else hz * 0.9 + (1 / dt) * 0.1
                touches, cand = detect(cfg, bg.foreground(pts, cfg["bg_tolerance_mm"]))
                present = brick.update(cand, now)
                osc.send("/lidar/touches", [c for t in touches for c in t])
                osc.send("/lidar/brick", [1.0 if present else 0.0, brick.u, brick.v])
                if now - last_hz_send > 0.5:
                    osc.send("/lidar/hz", [hz])
                    last_hz_send = now
                if monitor:
                    line = (f"{hz:5.1f} Hz   brick: {'YES' if present else 'no ':3s}   " +
                            "  ".join(f"touch ({u:.2f},{v:.2f})" for u, v in touches))
                    if line != last_print:
                        print(line)
                        last_print = line
        except KeyboardInterrupt:
            print("\nbye")
            break
        except (IOError, OSError) as e:
            print(f"[lidar] {e} - retrying in 2s")
            osc.send("/lidar/touches", [])
            time.sleep(2)
        finally:
            if lidar:
                try:
                    lidar.close()
                except Exception:
                    pass


def measure_touch(cfg, it, bg, seconds=2.0):
    """Median (angle, dist) of the most solid small blob over `seconds`."""
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        fg = sorted(p for p in bg.foreground(next(it), cfg["bg_tolerance_mm"])
                    if cfg["min_dist_mm"] <= p[1] <= 2000)
        local = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)), a, d) for a, d in fg]
        cl = [c for c in cluster(local, cfg["cluster_gap_mm"])
              if len(c) >= cfg["min_points"] and
              math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1]) <= cfg["finger_max_mm"] * 1.5]
        if cl:
            big = max(cl, key=len)
            samples.append((sum(p[2] for p in big) / len(big), sum(p[3] for p in big) / len(big)))
    if len(samples) < 5:
        return None
    samples.sort(key=lambda s: s[1])
    return samples[len(samples) // 2]


def fit_calibration(cfg, raw, targets_uv):
    """Rigid 2D fit (rotation + translation, optionally mirrored) of sensor points onto
    the known button centres. Returns the updated settings and per-target errors."""
    W, H = cfg["screen_w_mm"], cfg["screen_h_mm"]
    q = [(u * W, v * H) for u, v in targets_uv]
    results = []
    for flip in (False, True):
        p = [(d * math.cos(math.radians(a)), (-1 if flip else 1) * d * math.sin(math.radians(a)))
             for a, d in raw]
        pcx = sum(x for x, _ in p) / len(p); pcy = sum(y for _, y in p) / len(p)
        qcx = sum(x for x, _ in q) / len(q); qcy = sum(y for _, y in q) / len(q)
        num = sum((px - pcx) * (qy - qcy) - (py - pcy) * (qx - qcx) for (px, py), (qx, qy) in zip(p, q))
        den = sum((px - pcx) * (qx - qcx) + (py - pcy) * (qy - qcy) for (px, py), (qx, qy) in zip(p, q))
        th = math.atan2(num, den)
        c, s = math.cos(th), math.sin(th)
        tx, ty = qcx - (c * pcx - s * pcy), qcy - (s * pcx + c * pcy)
        errs = [math.hypot(c * px - s * py + tx - qx, s * px + c * py + ty - qy)
                for (px, py), (qx, qy) in zip(p, q)]
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        results.append({"flip": flip, "th": math.degrees(th), "tx": tx, "ty": ty, "errs": errs, "rms": rms})
    a, b = results
    row_y = sum(y for _, y in q) / len(q)
    if max(a["rms"], b["rms"]) > 1.5 * min(a["rms"], b["rms"]) + 5:
        best = min(results, key=lambda r: r["rms"])
    else:                              # (nearly) a straight row: pick the sensor-below-the-row solution
        best = min(results, key=lambda r: r["ty"] - row_y)
    # flip=True  -> phi = th - a  -> mirror False, offset = th
    # flip=False -> phi = th + a  -> mirror True,  offset = -th
    mirror = not best["flip"]
    offset = (best["th"] if best["flip"] else -best["th"]) % 360
    return {"mirror": mirror, "angle_offset_deg": round(offset, 2),
            "sensor_x_mm": round(best["tx"], 1), "sensor_y_mm": round(best["ty"], 1)}, best


def calibrate(cfg):
    print("\nCALIBRATION  (4 buttons, left to right)\n---------------------------------------")
    print("Open the TouchDesigner UI full screen. Keep the brick OFF the screen.\n")
    lidar = open_lidar(cfg)
    try:
        lidar.start_scan()
        it = scans(lidar)
        bg = learn_background(cfg, it)
        raw = []
        for i, uv in enumerate(BUTTON_CENTRES):
            input(f"\nPut ONE fingertip on the middle of button {i + 1} (from the LEFT), "
                  "hold it, press Enter...")
            lidar.flush_request = True
            m = None
            while m is None:
                m = measure_touch(cfg, it, bg)
                if m is None:
                    print("  nothing detected - keep the fingertip on the glass, retrying...")
            print(f"  seen at {m[0]:.1f} deg, {m[1]:.0f} mm")
            raw.append(m)
    finally:
        lidar.close()

    upd, best = fit_calibration(cfg, raw, BUTTON_CENTRES)
    cfg.update(upd)
    save_cfg(cfg)
    print(f"\nsensor at x = {upd['sensor_x_mm']:.0f} mm, y = {upd['sensor_y_mm']:.0f} mm "
          f"(from the bottom-left corner of the picture)")
    print(f"angle_offset_deg = {upd['angle_offset_deg']}, mirror = {upd['mirror']}")
    for i, e in enumerate(best["errs"]):
        print(f"  button {i + 1}: off by {e:.0f} mm")
    if best["rms"] > 15:
        print("\nWARNING: errors are large. Check that the UI fills the screen, the screen size "
              "in lidar_config.json, and that only your fingertip was in the laser plane.")
    else:
        print("\nGood fit.")
    print("Saved to", CONFIG_PATH)


def raw_view(cfg):
    lidar = open_lidar(cfg)
    try:
        lidar.start_scan()
        it = scans(lidar)
        bg = learn_background(cfg, it)
        print("blobs: angle / distance / points / width.  Ctrl+C to stop.\n")
        last = ""
        for pts in it:
            fg = sorted(p for p in bg.foreground(pts, cfg["bg_tolerance_mm"])
                        if cfg["min_dist_mm"] <= p[1] <= 3000)
            local = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)), a, d) for a, d in fg]
            out = []
            for c in cluster(local, cfg["cluster_gap_mm"]):
                if len(c) >= cfg["min_points"]:
                    w = math.hypot(c[-1][0] - c[0][0], c[-1][1] - c[0][1])
                    out.append((sum(p[3] for p in c) / len(c), sum(p[2] for p in c) / len(c), len(c), w))
            out.sort()
            line = "   ".join(f"{a:6.1f}deg {d:5.0f}mm ({n}) w{w:.0f}" for d, a, n, w in out[:4]) or "(nothing)"
            if line != last:
                print(line)
                last = line
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        lidar.close()


def simulate(cfg):
    osc = OscOut(cfg["osc_host"], cfg["osc_port"])
    print(f"[sim] fake taps + brick to {cfg['osc_host']}:{cfg['osc_port']}  (Ctrl+C to stop)")
    i = 0
    try:
        while True:
            u, v = BUTTON_CENTRES[i % N_BUTTONS]
            brick = 1.0 if (i // N_BUTTONS) % 2 else 0.0
            for t in range(14):
                touch = [u, v] if t < 4 else []
                osc.send("/lidar/touches", touch)
                osc.send("/lidar/brick", [brick, 0.5, 0.47])
                osc.send("/lidar/hz", [10.0])
                time.sleep(0.1)
            i += 1
    except KeyboardInterrupt:
        print("\nbye")


def main():
    ap = argparse.ArgumentParser(description="RPLIDAR C1 -> OSC bridge (4 buttons + brick)")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--info", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--monitor", action="store_true")
    ap.add_argument("--raw", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--osc-port", type=int)
    args = ap.parse_args()

    cfg = load_cfg()
    if args.port:
        cfg["port"] = args.port
    if args.osc_port:
        cfg["osc_port"] = args.osc_port

    if args.sim:
        return simulate(cfg)
    if serial is None:
        sys.exit("pyserial is missing:  pip install pyserial")
    if args.list:
        list_ports()
    elif args.info:
        open_lidar(cfg).close()
    elif args.calibrate:
        calibrate(cfg)
    elif args.raw:
        raw_view(cfg)
    else:
        run(cfg, monitor=args.monitor)


if __name__ == "__main__":
    main()
