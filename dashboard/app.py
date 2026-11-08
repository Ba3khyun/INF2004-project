"""Local MQTT telemetry dashboard for the Pico W robot."""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: install it with "
        "'python -m pip install -r dashboard/requirements.txt'"
    ) from exc


ROOT = Path(__file__).resolve().parent
TOOLS = ROOT.parent / "tools"
sys.path.insert(0, str(TOOLS))
from wifi_telemetry_tool import decode_telem  # noqa: E402


HOST = os.getenv("MQTT_HOST", "172.20.10.2")
PORT = int(os.getenv("MQTT_PORT", "1884"))
TOPIC = os.getenv("MQTT_TOPIC", "robot/is06/state")
HTTP_HOST = os.getenv("DASHBOARD_HOST", "127.0.0.1")
HTTP_PORT = int(os.getenv("DASHBOARD_PORT", "5000"))
OFFLINE_AFTER_SECONDS = float(os.getenv("OFFLINE_AFTER_SECONDS", "5"))

state_lock = threading.Lock()
latest: dict[str, object] = {
    "connected": False,
    "last_seen": None,
    "age_seconds": None,
    "telemetry": None,
    "message_count": 0,
    "error": None,
}


def reason_value(reason_code: object) -> int:
    value = getattr(reason_code, "value", reason_code)
    return int(value)


def snapshot() -> dict[str, object]:
    with state_lock:
        result = dict(latest)
    if result["last_seen"] is not None:
        result["age_seconds"] = round(time.time() - float(result["last_seen"]), 1)
        if result["age_seconds"] > OFFLINE_AFTER_SECONDS:
            result["connected"] = False
    return result


def on_connect(client: mqtt.Client, userdata: object, flags: dict, reason_code: object, properties: object = None) -> None:
    if reason_value(reason_code) != 0:
        with state_lock:
            latest["error"] = f"MQTT connection refused (code {reason_code})"
        return
    client.subscribe(TOPIC, qos=0)
    with state_lock:
        latest["connected"] = True
        latest["error"] = None


def on_disconnect(client: mqtt.Client, userdata: object, disconnect_flags: object, reason_code: object, properties: object = None) -> None:
    with state_lock:
        latest["connected"] = False
        if reason_value(reason_code) != 0:
            latest["error"] = f"MQTT disconnected (code {reason_code})"


def on_message(client: mqtt.Client, userdata: object, message: mqtt.MQTTMessage) -> None:
    try:
        telemetry = decode_telem(bytes(message.payload))
    except (TypeError, ValueError) as exc:
        with state_lock:
            latest["error"] = f"Invalid telemetry received: {exc}"
        return
    with state_lock:
        latest["telemetry"] = telemetry
        latest["last_seen"] = time.time()
        latest["message_count"] = int(latest["message_count"]) + 1
        latest["connected"] = True
        latest["error"] = None


def mqtt_worker() -> None:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is06-dashboard")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    try:
        client.connect(HOST, PORT, keepalive=30)
        client.loop_forever()
    except Exception as exc:
        with state_lock:
            latest["connected"] = False
            latest["error"] = f"MQTT startup failed: {exc}"


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self.send_file(ROOT / "templates" / "index.html", "text/html; charset=utf-8")
        elif path == "/static/style.css":
            self.send_file(ROOT / "static" / "style.css", "text/css; charset=utf-8")
        elif path == "/static/dashboard.js":
            self.send_file(ROOT / "static" / "dashboard.js", "text/javascript; charset=utf-8")
        elif path == "/api/status":
            self.send_json(snapshot())
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def send_file(self, path: Path, content_type: str) -> None:
        try:
            content = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def send_json(self, value: object) -> None:
        content = json.dumps(value).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, format: str, *args: object) -> None:
        return


def main() -> None:
    threading.Thread(target=mqtt_worker, daemon=True).start()
    server = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), DashboardHandler)
    print(f"Dashboard: http://{HTTP_HOST}:{HTTP_PORT}")
    print(f"MQTT: {HOST}:{PORT}, topic {TOPIC}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nDashboard stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
