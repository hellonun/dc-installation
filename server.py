#!/usr/bin/env python3
"""
Dream Con brick station - one program for the station computer
==============================================================

  * reads the RPLIDAR C1 (fingers on the topic buttons, brick on the spot)
  * saves every message to data/messages.csv (+ hourly backups in data/backups/)
  * serves the pages:
        http://localhost:8080/station        the table screen (The Frame)
        http://<this-computer>:8080/wall     the scrolling message screens
  * pushes live updates to the pages (Server-Sent Events, auto-reconnecting)

    python server.py            normal run (with the LiDAR)
    python server.py --nolidar  no sensor: use the station page with ?debug=1
                                (mouse = finger, F2 = brick on/off)

Only dependency: pip install pyserial
"""

import argparse
import csv
import datetime as dt
import json
import os
import queue
import shutil
import threading
import time
import uuid
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import lidar as L

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
DATA = os.path.join(HERE, "data")
CSV_PATH = os.path.join(DATA, "messages.csv")
BACKUPS = os.path.join(DATA, "backups")
COLOR_CAL = os.path.join(HERE, "color_cal.json")     # brick colour calibration, written by /station?colorcal=1
CSV_FIELDS = ["id", "time", "topic_id", "topic_th", "topic_en", "message", "brick_color"]
MAX_LEN = 80

with open(os.path.join(WEB, "topics.json"), encoding="utf-8") as f:
    TOPICS = {t["id"]: t for t in json.load(f)}


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# --------------------------------------------------------------------------- live updates

class Hub:
    """Fan-out of events to every connected page, per channel ('station' / 'wall')."""

    def __init__(self):
        self.lock = threading.Lock()
        self.subs = {"station": [], "wall": []}

    def subscribe(self, ch):
        q = queue.Queue(maxsize=200)
        with self.lock:
            self.subs.setdefault(ch, []).append(q)
        return q

    def unsubscribe(self, ch, q):
        with self.lock:
            if q in self.subs.get(ch, []):
                self.subs[ch].remove(q)

    def publish(self, ch, event, data):
        msg = (event, json.dumps(data, ensure_ascii=False))
        with self.lock:
            for q in list(self.subs.get(ch, [])):
                try:
                    q.put_nowait(msg)
                except queue.Full:          # a stuck page: drop it, it will reconnect
                    self.subs[ch].remove(q)


HUB = Hub()


# --------------------------------------------------------------------------- messages

class Store:
    def __init__(self):
        os.makedirs(BACKUPS, exist_ok=True)
        self.lock = threading.Lock()
        self.rows = []
        if os.path.exists(CSV_PATH):
            with open(CSV_PATH, encoding="utf-8-sig", newline="") as f:
                self.rows = list(csv.DictReader(f))
        else:
            with open(CSV_PATH, "w", encoding="utf-8-sig", newline="") as f:
                csv.DictWriter(f, CSV_FIELDS).writeheader()
        self.dirty = False
        log(f"[data] {len(self.rows)} messages in {CSV_PATH}")

    def add(self, topic_id, text, color=""):
        t = TOPICS[topic_id]
        row = {"id": uuid.uuid4().hex[:10], "time": dt.datetime.now().isoformat(timespec="seconds"),
               "topic_id": topic_id, "topic_th": t["th"], "topic_en": t["en"],
               "message": text, "brick_color": color}
        with self.lock:
            with open(CSV_PATH, "a", encoding="utf-8", newline="") as f:
                csv.DictWriter(f, CSV_FIELDS).writerow(row)
                f.flush()
                os.fsync(f.fileno())
            self.rows.append(row)
            self.dirty = True
        return row

    def latest(self, n):
        with self.lock:
            return self.rows[-n:]

    def backup_loop(self):
        last_hour = None
        while True:
            time.sleep(30)
            hour = time.strftime("%Y%m%d-%H")
            if self.dirty and hour != last_hour:
                last_hour = hour
                name = os.path.join(BACKUPS, f"messages-{hour}00.csv")
                with self.lock:
                    shutil.copy2(CSV_PATH, name)
                    self.dirty = False
                log("[data] backup", os.path.basename(name))


STORE = None


def public(row):
    return {"id": row["id"], "time": row["time"], "topic": row["topic_id"],
            "message": row["message"], "color": row.get("brick_color", "")}


# --------------------------------------------------------------------------- LiDAR

STATE = {"lidar": "off", "hz": 0.0, "touches": [], "brick": False, "brick_uv": [0.5, 0.5]}
STATE_LOCK = threading.Lock()


def set_state(**kw):
    with STATE_LOCK:
        STATE.update(kw)
        snap = dict(STATE)
    HUB.publish("station", "state", snap)


def lidar_loop():
    cfg = L.load_cfg()
    while True:
        lid = None
        try:
            set_state(lidar="connecting")
            lid = L.open_lidar(cfg)
            lid.start_scan()
            it = L.scans(lid)
            bg = L.learn_background(cfg, it)
            brick = L.BrickState(cfg)
            set_state(lidar="ok")
            last_t, hz, last_sent, last_key = time.time(), 0.0, 0.0, None
            for pts in it:
                now = time.time()
                d, last_t = now - last_t, now
                if d > 0:
                    hz = 1 / d if hz == 0 else hz * 0.9 + 0.1 / d
                touches, cand = L.detect(cfg, bg.foreground(pts, cfg["bg_tolerance_mm"]))
                present = brick.update(cand, now)
                touches = [[round(u, 4), round(v, 4)] for u, v in touches]
                key = (tuple(map(tuple, touches)), present)
                if key != last_key or now - last_sent > 0.5:     # changes at once, heartbeat 2x/s
                    set_state(touches=touches, brick=present,
                              brick_uv=[round(brick.u, 3), round(brick.v, 3)], hz=round(hz, 1))
                    last_key, last_sent = key, now
        except SystemExit as e:       # LiDAR not found: keep the pages running, retry
            log(f"[lidar] {e}")
            set_state(lidar="missing", touches=[], brick=False)
            time.sleep(3)
        except Exception as e:
            log(f"[lidar] {type(e).__name__}: {e} - retrying in 2s")
            set_state(lidar="error", touches=[], brick=False)
            time.sleep(2)
        finally:
            if lid:
                try:
                    lid.close()
                except Exception:
                    pass


# --------------------------------------------------------------------------- keyboard focus

REFOCUS = {"on": True, "last": 0.0, "warned": False}


def refocus(d):
    """The station page lost the keyboard to another window: click once on the middle of the station window.
    Needs `pip install pyautogui` (Mac: allow Terminal under Privacy & Security > Accessibility).
    lidar_config.json "refocus_xy": [x, y] overrides the spot, in the screen pixels pyautogui uses."""
    if not REFOCUS["on"] or time.time() - REFOCUS["last"] < 1.2:
        return
    try:
        import pyautogui
    except ImportError:
        if not REFOCUS["warned"]:
            log("[focus] the station window lost the keyboard. pip install pyautogui to fix it automatically")
            REFOCUS["warned"] = True
        return
    xy = L.load_cfg().get("refocus_xy")
    if xy:
        x, y = xy
    else:
        k = float(d.get("dpr", 1)) if os.name == "nt" else 1.0      # Windows: pyautogui works in physical pixels
        x, y = float(d["x"]) * k, float(d["y"]) * k
    REFOCUS["last"] = time.time()
    pyautogui.FAILSAFE = False
    pyautogui.click(int(x), int(y))
    log(f"[focus] clicked the station window at {int(x)}, {int(y)} to take the keyboard back")


# --------------------------------------------------------------------------- HTTP

MIME = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
        ".json": "application/json; charset=utf-8", ".css": "text/css; charset=utf-8",
        ".woff2": "font/woff2", ".png": "image/png", ".svg": "image/svg+xml"}
ROUTES = {"/": "station.html", "/station": "station.html", "/wall": "wall.html"}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):      # keep the terminal quiet
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8", cache=False):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False)
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=86400" if cache else "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        path, qs = url.path, parse_qs(url.query)
        if path == "/events":
            return self.events(qs.get("ch", ["station"])[0])
        if path == "/api/messages":
            n = max(1, min(1000, int(qs.get("limit", ["300"])[0])))
            return self._send(200, [public(r) for r in STORE.latest(n)])
        if path == "/api/config":
            cfg = L.load_cfg()
            return self._send(200, {"screen_w_mm": cfg["screen_w_mm"], "screen_h_mm": cfg["screen_h_mm"],
                                    "cal_points": L.CAL_POINTS, "max_len": MAX_LEN})
        if path == "/api/colorcal":
            if not os.path.exists(COLOR_CAL):
                return self._send(200, {})
            with open(COLOR_CAL, encoding="utf-8") as f:
                return self._send(200, f.read())
        if path == "/api/state":
            with STATE_LOCK:
                return self._send(200, dict(STATE))
        name = ROUTES.get(path, path.lstrip("/"))
        full = os.path.normpath(os.path.join(WEB, name))
        if not full.startswith(WEB) or not os.path.isfile(full):
            return self._send(404, {"error": "not found"})
        with open(full, "rb") as f:
            body = f.read()
        ext = os.path.splitext(full)[1]
        self._send(200, body, MIME.get(ext, "application/octet-stream"), cache=ext == ".woff2")

    def do_POST(self):
        if urlparse(self.path).path == "/api/colorcal":
            try:
                n = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(min(n, 200000)).decode("utf-8"))
                with open(COLOR_CAL, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=1)
                log(f"[colour] calibration saved: {len(data.get('samples', []))} samples")
                return self._send(200, {"ok": True})
            except Exception as e:
                return self._send(400, {"error": str(e)})
        if urlparse(self.path).path == "/api/refocus":
            try:
                n = int(self.headers.get("Content-Length", "0"))
                refocus(json.loads(self.rfile.read(min(n, 1000)).decode("utf-8")))
            except Exception as e:
                log(f"[focus] {e}")
            return self._send(200, {"ok": True})
        if urlparse(self.path).path != "/api/message":
            return self._send(404, {"error": "not found"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            data = json.loads(self.rfile.read(min(n, 10000)).decode("utf-8"))
            topic = data.get("topic")
            text = " ".join(str(data.get("message", "")).split())[:MAX_LEN + 20]   # + the "อยากเห็น" prefix
            if topic not in TOPICS or not text:
                return self._send(400, {"error": "topic and message required"})
            row = STORE.add(topic, text, str(data.get("color", ""))[:16])
        except Exception as e:
            log(f"[data] save failed: {e}")
            return self._send(500, {"error": "save failed"})
        log(f"[data] saved: [{topic}] {text}")
        HUB.publish("wall", "message", public(row))
        self._send(200, {"ok": True, "id": row["id"]})

    def events(self, ch):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q = HUB.subscribe(ch)
        try:
            self.wfile.write(b"retry: 2000\n\n")
            if ch == "station":
                with STATE_LOCK:
                    snap = json.dumps(STATE, ensure_ascii=False)
                self.wfile.write(f"event: state\ndata: {snap}\n\n".encode("utf-8"))
            self.wfile.flush()
            while True:
                try:
                    event, data = q.get(timeout=10)
                    self.wfile.write(f"event: {event}\ndata: {data}\n\n".encode("utf-8"))
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")         # keeps the connection alive
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            HUB.unsubscribe(ch, q)
            self.close_connection = True


def main():
    global STORE
    ap = argparse.ArgumentParser(description="Dream Con brick station server")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--nolidar", action="store_true", help="run without the sensor (debug)")
    ap.add_argument("--norefocus", action="store_true", help="don't click the station window to take the keyboard back")
    args = ap.parse_args()

    REFOCUS["on"] = not args.norefocus
    STORE = Store()
    threading.Thread(target=STORE.backup_loop, daemon=True).start()
    if args.nolidar:
        log("[lidar] disabled (--nolidar). Open /station?debug=1: mouse = finger, F2 = brick")
        set_state(lidar="off")
    else:
        threading.Thread(target=lidar_loop, daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    srv.daemon_threads = True
    log(f"[web] station: http://localhost:{args.port}/station   wall: http://<this-ip>:{args.port}/wall")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log("bye")


if __name__ == "__main__":
    main()
