#!/usr/bin/env python3
"""
RPLIDAR C1  ->  OSC touch bridge for TouchDesigner
==================================================

Reads the RPLIDAR C1 over USB serial, finds fingers in the scan plane in
front of the screen, converts them to normalised screen coordinates and sends
them to TouchDesigner over OSC.

    u = 0 left edge ... 1 right edge
    v = 0 bottom edge ... 1 top edge      (TouchDesigner convention)

OSC messages (UDP, default 127.0.0.1:7000):
    /lidar/touches   u0 v0 u1 v1 ...   (floats; empty when nothing is touching)
    /lidar/hz        scan rate in Hz

Only dependency:  pip install pyserial

Usage
-----
    python lidar_bridge.py --list          # list serial ports
    python lidar_bridge.py --info          # connect, print model / health
    python lidar_bridge.py --calibrate     # two-touch calibration -> lidar_config.json
    python lidar_bridge.py                 # run (sends OSC)
    python lidar_bridge.py --monitor       # run + print touches in terminal
    python lidar_bridge.py --sim           # no sensor: fake taps on the 5 buttons

Settings live in lidar_config.json next to this script (created on first run).
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

# 32" 16:9 panel active area is about 708 x 398 mm. Measure yours and adjust.
DEFAULT_CFG = {
    "port": "auto",              # "auto", or e.g. "COM5" / "/dev/cu.usbserial-0001"
    "baud": 460800,              # RPLIDAR C1 default
    "osc_host": "127.0.0.1",
    "osc_port": 7000,

    "screen_w_mm": 708.0,        # active (visible) area width
    "screen_h_mm": 398.0,        # active (visible) area height
    # Position of the LiDAR's spinning centre relative to the bottom-left corner
    # of the visible image. Negative = outside the image (e.g. on the bezel).
    "sensor_x_mm": -15.0,
    "sensor_y_mm": -15.0,

    # Filled in by --calibrate
    "angle_offset_deg": 0.0,
    "mirror": False,

    "edge_margin_mm": 8.0,       # ignore hits this close to the image edge (bezel/frame)
    "min_dist_mm": 30.0,         # ignore anything closer than this to the sensor
    "cluster_gap_mm": 35.0,      # points closer than this belong to the same finger
    "min_points": 2,             # points needed to count as a finger (1 = more sensitive, more noise)
    "max_touches": 5,

    "bg_seconds": 2.0,           # learn the empty scene at startup (keep the screen clear!)
    "bg_tolerance_mm": 40.0,     # must be this much closer than the background to count
}

# Calibration targets (u, v) — centres of elements in the TouchDesigner test UI
CAL_TARGETS = [
    ("the RIGHTMOST button (การเมือง / Politics)", (0.869, 0.122)),
    ("the centre of the RED BRICK", (0.500, 0.469)),
]


# --------------------------------------------------------------------------- config

def load_cfg():
    cfg = dict(DEFAULT_CFG)
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            cfg.update(json.load(f))
    else:
        save_cfg(cfg)
        print(f"[cfg] created {CONFIG_PATH}")
    return cfg


def save_cfg(cfg):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


# --------------------------------------------------------------------------- OSC (no dependency)

def _osc_str(s):
    b = s.encode("utf-8")
    return b + b"\0" * (4 - len(b) % 4)


def osc_message(address, floats):
    tags = "," + "f" * len(floats)
    return _osc_str(address) + _osc_str(tags) + b"".join(struct.pack(">f", float(x)) for x in floats)


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

CMD_STOP, CMD_RESET, CMD_SCAN, CMD_INFO, CMD_HEALTH = 0x25, 0x40, 0x20, 0x50, 0x52
KNOWN_USB = {(0x10C4, 0xEA60): "CP210x", (0x1A86, 0x7523): "CH340", (0x1A86, 0x55D4): "CH9102"}


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Is the C1 plugged in? (macOS/Windows may need the "
              "Silicon Labs CP210x VCP driver.)")
    for p in ports:
        tag = KNOWN_USB.get((p.vid, p.pid), "")
        print(f"  {p.device:35s} {tag:8s} {p.description}")
    return ports


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if (p.vid, p.pid) in KNOWN_USB:
            return p.device
    for p in ports:
        d = p.device.lower()
        if any(k in d for k in ("usbserial", "slab", "wchusb", "ttyusb")):
            return p.device
    return None


class RPLidar:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.ser.dtr = False            # motor-enable line on Slamtec USB adapters
        self.buf = bytearray()

    def _cmd(self, cmd):
        self.ser.write(bytes([0xA5, cmd]))

    def _descriptor(self):
        d = self.ser.read(7)
        if len(d) != 7 or d[0] != 0xA5 or d[1] != 0x5A:
            raise IOError(f"bad response descriptor: {d.hex()}")
        size = d[2] | d[3] << 8 | d[4] << 16 | (d[5] & 0x3F) << 24
        return size, d[6]

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

    def nodes(self):
        """Yield (new_scan, angle_deg, dist_mm, quality) forever."""
        ser, buf = self.ser, self.buf
        while True:
            chunk = ser.read(max(5, ser.in_waiting))
            if not chunk:
                raise IOError("no data from LiDAR (timeout)")
            buf.extend(chunk)
            i, n = 0, len(buf)
            while n - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                s = b0 & 1
                if s == ((b0 >> 1) & 1) or not (b1 & 1):
                    i += 1               # out of sync: slide one byte
                    continue
                angle = ((b1 >> 1) | (buf[i + 2] << 7)) / 64.0
                dist = (buf[i + 3] | (buf[i + 4] << 8)) / 4.0
                yield bool(s), angle, dist, b0 >> 2
                i += 5
            del buf[:i]

    def close(self):
        try:
            self.stop()
        finally:
            self.ser.close()


def scans(lidar):
    """Yield one full revolution at a time: list of (angle_deg, dist_mm)."""
    pts = []
    for new_scan, a, d, q in lidar.nodes():
        if new_scan and pts:
            yield pts
            pts = []
        if d > 0:
            pts.append((a, d))


# --------------------------------------------------------------------------- geometry / detection

class Background:
    def __init__(self):
        self.bins = [math.inf] * 360

    def learn(self, pts):
        for a, d in pts:
            b = int(a) % 360
            if d < self.bins[b]:
                self.bins[b] = d

    def foreground(self, pts, tol):
        out = []
        for a, d in pts:
            b = int(a) % 360
            # compare with this bin and its neighbours (angle jitter)
            bg = min(self.bins[b], self.bins[(b - 1) % 360], self.bins[(b + 1) % 360])
            if d < bg - tol:
                out.append((a, d))
        return out


def to_screen_mm(cfg, a, d):
    """Raw (angle, dist) -> (x, y) mm from the bottom-left corner of the image."""
    if cfg["mirror"]:
        phi = a - cfg["angle_offset_deg"]
    else:
        phi = cfg["angle_offset_deg"] - a
    r = math.radians(phi)
    return cfg["sensor_x_mm"] + d * math.cos(r), cfg["sensor_y_mm"] + d * math.sin(r)


def cluster(points, gap):
    """points: list of (x, y, ...) in scan order. Returns list of lists."""
    clusters, cur, prev = [], [], None
    for p in points:
        if prev is not None and math.hypot(p[0] - prev[0], p[1] - prev[1]) > gap:
            clusters.append(cur)
            cur = []
        cur.append(p)
        prev = p
    if cur:
        clusters.append(cur)
    return clusters


def detect_touches(cfg, fg_pts):
    W, H, m = cfg["screen_w_mm"], cfg["screen_h_mm"], cfg["edge_margin_mm"]
    inside = []
    for a, d in sorted(fg_pts):
        if d < cfg["min_dist_mm"]:
            continue
        x, y = to_screen_mm(cfg, a, d)
        if m <= x <= W - m and m <= y <= H - m:
            inside.append((x, y))
    touches = []
    for c in cluster(inside, cfg["cluster_gap_mm"]):
        if len(c) >= cfg["min_points"]:
            x = sum(p[0] for p in c) / len(c)
            y = sum(p[1] for p in c) / len(c)
            touches.append((len(c), x / W, y / H))
    touches.sort(reverse=True)               # biggest blobs first
    return [(u, v) for _, u, v in touches[: cfg["max_touches"]]]


# --------------------------------------------------------------------------- modes

def open_lidar(cfg):
    if serial is None:
        sys.exit("pyserial is missing:  pip install pyserial")
    port = cfg["port"] if cfg["port"] != "auto" else find_port()
    if not port:
        print("Could not find the LiDAR. Available ports:")
        list_ports()
        sys.exit("Set \"port\" in lidar_config.json or pass --port.")
    print(f"[lidar] opening {port} @ {cfg['baud']}")
    lidar = RPLidar(port, cfg["baud"])
    try:
        info = lidar.info()
        status, err = lidar.health()
        print(f"[lidar] model 0x{info['model']:02X}  fw {info['firmware']}  hw {info['hardware']}  "
              f"S/N {info['serial']}  health: {status}" + (f" (code {err})" if err else ""))
    except IOError as e:
        print(f"[lidar] info/health failed ({e}) - continuing")
    return lidar


def learn_background(cfg, it):
    bg = Background()
    secs = cfg["bg_seconds"]
    if secs <= 0:
        return bg
    print(f"[bg] learning empty scene for {secs:.1f}s - keep hands away from the screen...")
    t0 = time.time()
    while time.time() - t0 < secs:
        bg.learn(next(it))
    print("[bg] done")
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
            last_t, hz, last_hz_send, last_print = time.time(), 0.0, 0.0, ""
            for pts in it:
                now = time.time()
                dt = now - last_t
                last_t = now
                if dt > 0:
                    hz = hz * 0.9 + (1.0 / dt) * 0.1
                touches = detect_touches(cfg, bg.foreground(pts, cfg["bg_tolerance_mm"]))
                osc.send("/lidar/touches", [c for t in touches for c in t])
                if now - last_hz_send > 0.5:
                    osc.send("/lidar/hz", [hz])
                    last_hz_send = now
                if monitor:
                    line = f"{hz:5.1f} Hz  {len(pts):4d} pts  " + \
                           "  ".join(f"({u:.2f},{v:.2f})" for u, v in touches)
                    if line != last_print:
                        print(line)
                        last_print = line
        except KeyboardInterrupt:
            print("\nbye")
            break
        except (IOError, OSError, serial.SerialException) as e:
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
    """Return median (angle, dist) of the biggest foreground blob over `seconds`."""
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        fg = sorted(p for p in bg.foreground(next(it), cfg["bg_tolerance_mm"])
                    if cfg["min_dist_mm"] <= p[1] <= 3000)
        # cluster in polar->local cartesian (sensor frame) so it is independent of calibration
        local = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)), a, d) for a, d in fg]
        cl = [c for c in cluster(local, cfg["cluster_gap_mm"]) if len(c) >= cfg["min_points"]]
        if cl:
            big = max(cl, key=len)
            samples.append((sum(p[2] for p in big) / len(big), sum(p[3] for p in big) / len(big)))
    if len(samples) < 5:
        return None
    samples.sort()
    return samples[len(samples) // 2]


def calibrate(cfg):
    print("\nCALIBRATION\n-----------")
    print("Open the TouchDesigner test UI full screen. You will touch two targets.\n"
          "Check sensor_x_mm / sensor_y_mm and screen size in lidar_config.json first.\n")
    lidar = open_lidar(cfg)
    try:
        lidar.start_scan()
        it = scans(lidar)
        bg = learn_background(cfg, it)
        raw = []
        for label, (u, v) in CAL_TARGETS:
            input(f"\nPut ONE finger on {label}, hold it there, then press Enter...")
            m = None
            while m is None:
                m = measure_touch(cfg, it, bg)
                if m is None:
                    print("  nothing detected - keep the finger flat against the glass, retrying...")
            print(f"  raw angle {m[0]:.2f} deg, distance {m[1]:.0f} mm")
            raw.append(m)
            # drain buffered data while the user moves
            lidar.ser.reset_input_buffer()
            lidar.buf.clear()
    finally:
        lidar.close()

    W, H, sx, sy = cfg["screen_w_mm"], cfg["screen_h_mm"], cfg["sensor_x_mm"], cfg["sensor_y_mm"]
    exp = []
    for _, (u, v) in CAL_TARGETS:
        dx, dy = u * W - sx, v * H - sy
        exp.append((math.degrees(math.atan2(dy, dx)), math.hypot(dx, dy)))

    def wrap(x):
        return (x + 180) % 360 - 180

    d_raw = wrap(raw[1][0] - raw[0][0])
    d_exp = wrap(exp[1][0] - exp[0][0])
    mirror = (d_raw * d_exp) > 0          # same rotation sense -> mirrored formula
    if mirror:
        offs = [wrap(raw[i][0] - exp[i][0]) for i in range(2)]
    else:
        offs = [wrap(exp[i][0] + raw[i][0]) for i in range(2)]
    # circular mean of the two estimates
    off = math.degrees(math.atan2(sum(math.sin(math.radians(o)) for o in offs),
                                  sum(math.cos(math.radians(o)) for o in offs))) % 360

    cfg["mirror"] = mirror
    cfg["angle_offset_deg"] = round(off, 2)
    save_cfg(cfg)
    print(f"\nangle_offset_deg = {cfg['angle_offset_deg']}, mirror = {mirror}")
    print(f"angle spread between targets: measured {abs(d_raw):.1f} deg, expected {abs(d_exp):.1f} deg")
    for i, (label, _) in enumerate(CAL_TARGETS):
        print(f"distance to target {i + 1}: measured {raw[i][1]:.0f} mm, expected {exp[i][1]:.0f} mm")
    print("If the distances are off by more than ~20 mm, fix sensor_x_mm / sensor_y_mm or the "
          "screen size and calibrate again.\nSaved to", CONFIG_PATH)


def simulate(cfg):
    """No hardware: tap each button in turn so you can test the TouchDesigner side."""
    osc = OscOut(cfg["osc_host"], cfg["osc_port"])
    print(f"[sim] sending fake taps to {cfg['osc_host']}:{cfg['osc_port']}  (Ctrl+C to stop)")
    centres = [(0.131, 0.122), (0.316, 0.122), (0.500, 0.122), (0.684, 0.122), (0.869, 0.122)]
    i = 0
    try:
        while True:
            u, v = centres[i % 5]
            t_end = time.time() + 0.4
            while time.time() < t_end:          # finger down for 0.4 s at 10 Hz
                osc.send("/lidar/touches", [u, v])
                osc.send("/lidar/hz", [10.0])
                time.sleep(0.1)
            t_end = time.time() + 1.0
            while time.time() < t_end:          # finger up for 1 s
                osc.send("/lidar/touches", [])
                osc.send("/lidar/hz", [10.0])
                time.sleep(0.1)
            i += 1
    except KeyboardInterrupt:
        print("\nbye")


def main():
    ap = argparse.ArgumentParser(description="RPLIDAR C1 -> OSC touch bridge")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--info", action="store_true", help="print LiDAR info/health and exit")
    ap.add_argument("--calibrate", action="store_true", help="two-touch calibration")
    ap.add_argument("--monitor", action="store_true", help="print touches while running")
    ap.add_argument("--sim", action="store_true", help="no sensor: send fake taps")
    ap.add_argument("--port", help="serial port (overrides config)")
    ap.add_argument("--osc-port", type=int, help="OSC port (overrides config)")
    args = ap.parse_args()

    cfg = load_cfg()
    if args.port:
        cfg["port"] = args.port
    if args.osc_port:
        cfg["osc_port"] = args.osc_port

    if args.sim:
        simulate(cfg)
        return
    if serial is None:
        sys.exit("pyserial is missing:  pip install pyserial")
    if args.list:
        list_ports()
    elif args.info:
        open_lidar(cfg).close()
    elif args.calibrate:
        calibrate(cfg)
    else:
        run(cfg, monitor=args.monitor)


if __name__ == "__main__":
    main()
