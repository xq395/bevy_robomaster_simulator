"""Daedalus operator UI: browser heartbeat -> short-lived simulator control lease."""
import json
import io
import os
import socket
import threading
import time
import urllib.request
from urllib.parse import urlparse, parse_qs
from PIL import Image
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from narrowband_video import NarrowbandVideo

# RMU2026 official protocol link (MQTT 3333 + UDP 3334). Off unless configured,
# so the simulator-only setup behaves exactly as before.
RMU_ENABLED = os.environ.get("RMU_ENABLED", "0") == "1"
RMU_HOST = os.environ.get("RMU_MQTT_HOST", "192.168.12.1")
RMU_PORT = int(os.environ.get("RMU_MQTT_PORT", "3333"))
RMU_ROBOT_ID = int(os.environ.get("RMU_ROBOT_ID", "3"))
RMU_VIDEO_PORT = int(os.environ.get("RMU_VIDEO_PORT", "3334"))
RMU_VIDEO_HOST = os.environ.get("RMU_VIDEO_HOST", "0.0.0.0")

# Defaults match the local single-machine setup; override for a split install.
SIM_HOST = os.environ.get("DAEDALUS_SIM_HOST", "127.0.0.1")
SIM_PORT = int(os.environ.get("DAEDALUS_SIM_PORT", "5001"))
VIDEO_HOST = os.environ.get("DAEDALUS_VIDEO_HOST", "127.0.0.1")
VIDEO_PORT = int(os.environ.get("DAEDALUS_VIDEO_PORT", "5000"))
LISTEN_HOST = os.environ.get("DAEDALUS_OPERATOR_HOST", "127.0.0.1")
LISTEN_PORT = int(os.environ.get("DAEDALUS_OPERATOR_PORT", "5002"))
ALLOWED_ORIGINS = (None, f"http://{LISTEN_HOST}:{LISTEN_PORT}",
                   f"http://localhost:{LISTEN_PORT}", f"http://127.0.0.1:{LISTEN_PORT}")


class Relay:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.updated = 0
        self.intent = None
        self.browser_seen = 0
        self.error = "等待模拟器"
        self.seq = 0

    def run(self):
        sock = None
        while True:
            if sock is None:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    sock.connect((SIM_HOST, SIM_PORT))
                    sock.settimeout(0.1)
                except OSError as exc:
                    with self.lock:
                        self.error = str(exc)
                    sock = None
                    time.sleep(0.3)
                    continue
            try:
                with self.lock:
                    intent = self.intent
                    if time.monotonic() - self.browser_seen > 0.4:
                        intent = self.intent = None
                    previous = self.state
                    # Keep the lease alive between browser heartbeats: send every
                    # loop with a strictly increasing seq so no send is ever a
                    # duplicate. Only a session change (restart/local takeover)
                    # drops the intent, never a transient rejection.
                    if intent and previous and intent["session"] == previous["session"]:
                        self.seq = max(self.seq + 1, previous["applied_seq"] + 1)
                        seq = self.seq
                    else:
                        seq = None
                if seq is None:
                    sock.send(b"status")
                else:
                    packet = dict(intent, seq=seq,
                                  sent_ms=time.time_ns() // 1_000_000, ttl_ms=250)
                    sock.send(json.dumps(packet).encode())
                state = json.loads(sock.recv(4096))
                with self.lock:
                    if previous and state["session"] != previous["session"]:
                        self.intent = None
                        self.seq = state["applied_seq"]
                    self.state, self.updated, self.error = state, time.monotonic(), ""
            except TimeoutError:
                # No reply this cycle; the simulator may be busy or offline.
                with self.lock:
                    self.error = "等待模拟器响应"
            except (OSError, ValueError) as exc:
                # A connected UDP socket is poisoned by an ICMP port-unreachable
                # ([WinError 10054]) when the simulator restarts. Recreate it so
                # the relay self-heals instead of freezing on a dead socket.
                with self.lock:
                    self.error = str(exc)
                    self.state = None
                sock.close()
                sock = None
            time.sleep(0.03)


relay = Relay()
video = NarrowbandVideo()
UI_PATH = Path(__file__).with_name("operator_ui.html")
rmu = None  # RmuLink when RMU_ENABLED


def start_rmu():
    global rmu
    if not RMU_ENABLED:
        return
    from rmu_protocol.client_link import RmuLink
    rmu = RmuLink(RMU_HOST, RMU_PORT, RMU_ROBOT_ID, RMU_VIDEO_PORT or None, RMU_VIDEO_HOST).start()


def read_json(handler, limit):
    """Same-origin JSON body or ValueError. Mirrors /api/control's checks."""
    if handler.headers.get("Origin") not in ALLOWED_ORIGINS:
        raise PermissionError("origin")
    length = int(handler.headers.get("Content-Length", "0"))
    if not 0 < length <= limit or handler.headers.get("Content-Type") != "application/json":
        raise ValueError("invalid request")
    return json.loads(handler.rfile.read(length))


def ui_version():
    try:
        return str(UI_PATH.stat().st_mtime_ns)
    except OSError:
        return "0"


class Handler(BaseHTTPRequestHandler):
    def send(self, code, data, kind="application/json"):
        try:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionError, OSError):
            # Browser closed the connection (navigation or aborted fetch).
            pass

    def do_GET(self):
        if self.path == "/":
            self.send(200, UI_PATH.read_bytes(), "text/html; charset=utf-8")
        elif self.path == "/api/state":
            version = ui_version()
            with relay.lock:
                data = dict(state=relay.state, age_ms=round((time.monotonic()-relay.updated)*1000),
                            error=relay.error, controlling=relay.intent is not None,
                            ui_version=version)
            self.send(200, json.dumps(data).encode())
        elif self.path.startswith("/frame"):
            try:
                query = parse_qs(urlparse(self.path).query)
                width = max(160, min(1280, int(query.get("width", [640])[0])))
                quality = max(5, min(90, int(query.get("quality", [55])[0])))
                with urllib.request.urlopen(f"http://{VIDEO_HOST}:{VIDEO_PORT}/snapshot/raw", timeout=0.5) as response:
                    image = Image.open(io.BytesIO(response.read())).convert("RGB")
                    image.thumbnail((width, width))
                    output = io.BytesIO()
                    image.save(output, format="JPEG", quality=quality, optimize=True)
                    self.send(200, output.getvalue(), "image/jpeg")
            except (OSError, ValueError):
                self.send(503, b'{}')
        elif self.path == "/narrowband/frame":
            with video.lock:
                image = video.jpeg
            if image is None:
                self.send(503, b'{}', "application/json")
            else:
                self.send(200, image, "image/jpeg")
        elif self.path == "/api/narrowband":
            self.send(200, json.dumps(video.snapshot()).encode())
        elif self.path == "/api/rmu/state":
            data = rmu.snapshot() if rmu else dict(enabled=False)
            data.setdefault("enabled", True)
            self.send(200, json.dumps(data, ensure_ascii=False).encode())
        elif self.path == "/rmu/custom_frame":
            image = rmu.custom_video.latest_jpeg() if rmu else None
            if image is None:
                self.send(503, b'{}')
            else:
                self.send(200, image, "image/jpeg")
        elif self.path == "/rmu/frame":
            image = rmu.video.latest_jpeg() if rmu and rmu.video else None
            if image is None:
                self.send(503, b'{}')
            else:
                self.send(200, image, "image/jpeg")
        else:
            self.send(404, b'{}')

    def do_rmu_post(self):
        from rmu_protocol.client_link import RateLimited
        if rmu is None:
            self.send(404, b'{"accepted":false,"error":"RMU disabled"}')
            return
        try:
            data = read_json(self, 4096)
            if self.path == "/api/rmu/drive":
                if type(data.get("on")) is not bool:
                    raise ValueError("on must be bool")
                rmu.set_driving(data["on"])
                body = dict(accepted=True, driving=data["on"])
            elif self.path == "/api/rmu/input":
                rmu.update_input(data)
                body = dict(accepted=True)
            elif self.path == "/api/rmu/command":
                topic, fields = data.get("topic"), data.get("fields", {})
                role = data.get("role", "gimbal")
                if not isinstance(topic, str):
                    raise ValueError("topic required")
                body = dict(accepted=True, result=rmu.send_command(topic, fields, role))
            else:
                self.send(404, b'{}')
                return
            self.send(200, json.dumps(body, ensure_ascii=False).encode())
        except PermissionError:
            self.send(403, b'{"accepted":false}')
        except RateLimited as exc:
            self.send(429, json.dumps(dict(accepted=False, error=str(exc), wait_ms=exc.wait_ms), ensure_ascii=False).encode())
        except (ValueError, KeyError, TypeError) as exc:
            self.send(409, json.dumps(dict(accepted=False, error=str(exc)), ensure_ascii=False).encode())

    def do_POST(self):
        if self.path.startswith("/api/rmu/"):
            self.do_rmu_post()
            return
        if self.path == "/api/narrowband":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 2048 or self.headers.get("Origin") not in ALLOWED_ORIGINS:
                    raise ValueError('invalid request')
                data = json.loads(self.rfile.read(length))
                video.configure(data)
                self.send(202, b'{"accepted":true}')
            except (ValueError, KeyError, TypeError):
                self.send(400, b'{"accepted":false}')
            return
        if self.path != "/api/control":
            self.send(404, b'{}')
            return
        # Only same-origin JSON requests; no cross-origin control API.
        if self.headers.get("Origin") not in ALLOWED_ORIGINS:
            self.send(403, b'{}')
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length < 1024 or self.headers.get("Content-Type") != "application/json":
                raise ValueError("invalid request")
            data = json.loads(self.rfile.read(length))
            for key in ("aim_enabled", "fire_authorized", "spin_enabled"):
                if type(data.get(key)) is not bool:
                    raise ValueError("invalid switch")
            if data["fire_authorized"] and not data["aim_enabled"]:
                raise ValueError("aim required")
            mode = data.get("mode", "command")
            manual = data.get("manual", dict(movement=[0, 0], gimbal=[0, 0], shoot=False))
            if mode not in ("command", "manual") or not isinstance(manual, dict):
                raise ValueError("invalid mode")
            for key in ("movement", "gimbal"):
                values = manual.get(key)
                if not isinstance(values, list) or len(values) != 2 or any(type(v) not in (int, float) or not -1 <= v <= 1 for v in values):
                    raise ValueError("invalid axes")
            if type(manual.get("shoot")) is not bool:
                raise ValueError("invalid shoot")
            if mode == "manual" and any(data[k] for k in ("aim_enabled", "fire_authorized", "spin_enabled")):
                raise ValueError("manual excludes automation")
            with relay.lock:
                if not relay.state or time.monotonic()-relay.updated > 0.5 or data.get("session") != relay.state["session"]:
                    raise ValueError("stale session")
                relay.intent = {key: data[key] for key in ("session", "aim_enabled", "fire_authorized", "spin_enabled")}
                relay.intent.update(mode=mode, manual=manual)
                relay.browser_seen = time.monotonic()
            self.send(200, b'{"accepted":true}')
        except (ValueError, KeyError):
            self.send(409, b'{"accepted":false}')

    def log_message(self, *_):
        pass


if __name__ == "__main__":
    threading.Thread(target=relay.run, daemon=True).start()
    start_rmu()
    print(f"Daedalus operator: http://{LISTEN_HOST}:{LISTEN_PORT}/  (simulator {SIM_HOST}:{SIM_PORT})", flush=True)
    if rmu:
        print(f"RMU 协议链路：MQTT {RMU_HOST}:{RMU_PORT} clientID={RMU_ROBOT_ID}  图传 UDP {RMU_VIDEO_HOST}:{RMU_VIDEO_PORT}", flush=True)
    ThreadingHTTPServer((LISTEN_HOST, LISTEN_PORT), Handler).serve_forever()