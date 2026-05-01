"""Topside browser dashboard for recording and auxiliary actuator control."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import rclpy
from fish_operator_msgs.msg import ActuatorCommand, RecordingStatus
from fish_operator_msgs.srv import StartHighResRecording, StopHighResRecording
from rclpy.node import Node


DASHBOARD_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Leviathan Operator</title>
  <style>
    :root {
      color-scheme: dark;
      font-family: Arial, sans-serif;
      background: #101418;
      color: #eef2f5;
    }
    body { margin: 0; }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 14px 18px;
      background: #17202a;
      border-bottom: 1px solid #2b3a48;
    }
    main {
      display: grid;
      gap: 16px;
      grid-template-columns: repeat(auto-fit, minmax(310px, 1fr));
      padding: 16px;
    }
    section {
      border: 1px solid #2b3a48;
      border-radius: 8px;
      background: #151b22;
      padding: 14px;
    }
    h1 { font-size: 20px; margin: 0; }
    h2 { font-size: 16px; margin: 0 0 12px; }
    label { display: block; margin: 10px 0 6px; color: #b8c4cf; }
    select, input, button {
      box-sizing: border-box;
      width: 100%;
      min-height: 38px;
      border-radius: 6px;
      border: 1px solid #344658;
      background: #0f1419;
      color: #eef2f5;
      padding: 8px;
      font-size: 14px;
    }
    button {
      cursor: pointer;
      background: #265f92;
      border-color: #3178b8;
      font-weight: 700;
    }
    button.stop { background: #7d2f2f; border-color: #a84545; }
    button.secondary { background: #26313d; border-color: #3c4e60; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .status {
      margin-top: 12px;
      padding: 10px;
      border-radius: 6px;
      background: #0f1419;
      min-height: 96px;
      white-space: pre-wrap;
      color: #cbd6df;
    }
    input[type="range"] { padding: 0; }
    .servo-value { float: right; color: #eef2f5; }
  </style>
</head>
<body>
  <header>
    <h1>Leviathan Operator</h1>
    <button class="stop" style="max-width: 160px" onclick="stopAll()">Stop All</button>
  </header>
  <main>
    <section>
      <h2>High-Res Capture</h2>
      <label for="camera">Camera</label>
      <select id="camera">
        <option value="front">front</option>
        <option value="bottom">bottom</option>
        <option value="left">left</option>
        <option value="right">right</option>
      </select>
      <label for="mode">Mode</label>
      <select id="mode">
        <option value="video">video</option>
        <option value="image">image</option>
      </select>
      <label for="label">Label</label>
      <input id="label" value="operator">
      <div class="row" style="margin-top: 12px">
        <button onclick="startCapture()">Start</button>
        <button class="stop" onclick="stopCapture()">Stop</button>
      </div>
      <button class="secondary" style="margin-top: 10px" onclick="copyLatest()">
        Copy Latest Topside
      </button>
      <div id="recordStatus" class="status">No status yet.</div>
      <div id="copyStatus" class="status">No copy yet.</div>
    </section>
    <section>
      <h2>Servos</h2>
      <label>Servo 20 <span id="servo20Value" class="servo-value">0.00</span></label>
      <input id="servo20" type="range" min="0" max="1" step="0.01" value="0"
             oninput="setServo('servo_20', this.value, 'servo20Value')">
      <label>Servo 01 <span id="servo01Value" class="servo-value">0.00</span></label>
      <input id="servo01" type="range" min="0" max="1" step="0.01" value="0"
             oninput="setServo('servo_01', this.value, 'servo01Value')">
    </section>
    <section>
      <h2>Linear Actuator</h2>
      <label for="linearSpeed">Speed <span id="linearSpeedValue">1.00</span></label>
      <input id="linearSpeed" type="range" min="0" max="1" step="0.05" value="1"
             oninput="linearSpeedValue.textContent = Number(this.value).toFixed(2)">
      <div class="row" style="margin-top: 12px">
        <button id="linearBack" class="secondary">Back</button>
        <button id="linearForward">Forward</button>
      </div>
      <button class="stop" style="margin-top: 10px" onclick="linearCommand(0)">Stop</button>
    </section>
  </main>
  <script>
    let linearTimer = null;
    async function post(path, payload) {
      const response = await fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload || {})
      });
      return response.json();
    }
    async function startCapture() {
      await post('/api/record/start', {
        camera: camera.value,
        mode: mode.value,
        label: label.value
      });
      refreshStatus();
    }
    async function stopCapture() {
      await post('/api/record/stop', {});
      refreshStatus();
    }
    async function copyLatest() {
      copyStatus.textContent = 'Copying...';
      const result = await post('/api/copy_latest', {});
      copyStatus.textContent =
        `success: ${result.success}\\n` +
        `message: ${result.message || '-'}\\n` +
        `source: ${result.source || '-'}\\n` +
        `local: ${result.local_path || '-'}`;
    }
    async function setServo(name, value, outputId) {
      document.getElementById(outputId).textContent = Number(value).toFixed(2);
      await post('/api/actuator', {
        name: name,
        actuator_type: 'servo',
        value: Number(value)
      });
    }
    async function linearCommand(value) {
      await post('/api/actuator', {
        name: 'linear_actuator',
        actuator_type: 'linear',
        value: value
      });
    }
    function holdLinear(direction) {
      clearInterval(linearTimer);
      const send = () => linearCommand(direction * Number(linearSpeed.value));
      send();
      linearTimer = setInterval(send, 100);
    }
    function releaseLinear() {
      clearInterval(linearTimer);
      linearTimer = null;
      linearCommand(0);
    }
    async function stopAll() {
      releaseLinear();
      await post('/api/stop_all', {});
    }
    async function refreshStatus() {
      const status = await fetch('/api/status').then(r => r.json());
      recordStatus.textContent =
        `active: ${status.active}\\n` +
        `mode: ${status.mode || '-'}\\n` +
        `camera: ${status.camera || '-'}\\n` +
        `frames: ${status.frame_count}\\n` +
        `elapsed: ${status.elapsed_sec.toFixed(1)}s\\n` +
        `file: ${status.file_path || '-'}\\n` +
        `copy target: ${status.copy_target || '-'}\\n` +
        `error: ${status.error || '-'}`;
    }
    for (const [id, direction] of [['linearBack', -1], ['linearForward', 1]]) {
      const button = document.getElementById(id);
      button.addEventListener('pointerdown', () => holdLinear(direction));
      button.addEventListener('pointerup', releaseLinear);
      button.addEventListener('pointerleave', releaseLinear);
      button.addEventListener('pointercancel', releaseLinear);
    }
    setInterval(refreshStatus, 1000);
    refreshStatus();
  </script>
</body>
</html>
"""


class OperatorDashboard(Node):
    def __init__(self) -> None:
        super().__init__("operator_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("jetson_ssh_target", "ubuntu@jetson.local")
        self.declare_parameter("local_media_dir", "~/fish_captures")
        self.declare_parameter("scp_timeout_sec", 60.0)

        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        self.jetson_ssh_target = str(
            self.get_parameter("jetson_ssh_target").value
        ).strip()
        self.local_media_dir = Path(
            os.path.expanduser(str(self.get_parameter("local_media_dir").value))
        )
        self.scp_timeout_sec = float(self.get_parameter("scp_timeout_sec").value)
        self.local_media_dir.mkdir(parents=True, exist_ok=True)
        self.latest_status = RecordingStatus()

        self.actuator_pub = self.create_publisher(
            ActuatorCommand,
            "operator/actuator_command",
            10,
        )
        self.status_sub = self.create_subscription(
            RecordingStatus,
            "high_res_recording/status",
            self._status_callback,
            10,
        )
        self.start_client = self.create_client(
            StartHighResRecording,
            "high_res_recording/start",
        )
        self.stop_client = self.create_client(
            StopHighResRecording,
            "high_res_recording/stop",
        )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.httpd = ThreadingHTTPServer((host, port), self._make_handler())
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever,
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(f"Operator dashboard listening on http://{host}:{port}")

    def _status_callback(self, msg: RecordingStatus) -> None:
        self.latest_status = msg

    def status_dict(self) -> dict[str, Any]:
        status = self.latest_status
        return {
            "active": bool(status.active),
            "mode": status.mode,
            "camera": status.camera,
            "file_path": status.file_path,
            "frame_count": int(status.frame_count),
            "elapsed_sec": float(status.elapsed_sec),
            "error": status.error,
            "copy_target": str(self.local_media_dir),
        }

    def publish_actuator(self, name: str, actuator_type: str, value: float) -> None:
        msg = ActuatorCommand()
        msg.name = name
        msg.actuator_type = actuator_type
        msg.value = float(value)
        self.actuator_pub.publish(msg)

    def stop_all(self) -> None:
        self.publish_actuator("all", "stop_all", 0.0)

    def start_capture(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.start_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "recording service unavailable"}

        request = StartHighResRecording.Request()
        request.camera = str(payload.get("camera", ""))
        request.label = str(payload.get("label", "operator"))
        request.mode = str(payload.get("mode", "video"))
        return self._call_service(self.start_client, request)

    def stop_capture(self) -> dict[str, Any]:
        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            return {"success": False, "message": "recording service unavailable"}

        request = StopHighResRecording.Request()
        return self._call_service(self.stop_client, request)

    def copy_latest(self) -> dict[str, Any]:
        status = self.latest_status
        if status.active:
            return {
                "success": False,
                "message": "stop the active capture before copying",
                "source": status.file_path,
                "local_path": "",
            }

        if not status.file_path:
            return {
                "success": False,
                "message": "no captured file path is available",
                "source": "",
                "local_path": "",
            }

        source_path = status.file_path
        local_path = self.local_media_dir / Path(source_path).name

        if self.jetson_ssh_target:
            source = f"{self.jetson_ssh_target}:{source_path}"
            command = ["scp", source, str(local_path)]
        else:
            source = source_path
            command = []

        try:
            if command:
                completed = subprocess.run(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=self.scp_timeout_sec,
                )
                if completed.returncode != 0:
                    message = completed.stderr.strip() or completed.stdout.strip()
                    return {
                        "success": False,
                        "message": message or f"scp exited {completed.returncode}",
                        "source": source,
                        "local_path": str(local_path),
                    }
            else:
                shutil.copy2(source_path, local_path)
        except FileNotFoundError as exc:
            return {
                "success": False,
                "message": str(exc),
                "source": source,
                "local_path": str(local_path),
            }
        except subprocess.TimeoutExpired:
            return {
                "success": False,
                "message": "scp timed out",
                "source": source,
                "local_path": str(local_path),
            }

        return {
            "success": True,
            "message": "copied",
            "source": source,
            "local_path": str(local_path),
        }

    def _call_service(self, client, request) -> dict[str, Any]:
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.service_timeout_sec):
            return {"success": False, "message": "service call timed out"}

        result = future.result()
        return {
            field: getattr(result, field)
            for field in result.get_fields_and_field_types().keys()
        }

    def _make_handler(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path == "/" or self.path == "/index.html":
                    self._send_text(DASHBOARD_HTML, "text/html")
                elif self.path == "/api/status":
                    self._send_json(dashboard.status_dict())
                else:
                    self.send_error(404)

            def do_POST(self) -> None:
                payload = self._read_json()
                if self.path == "/api/record/start":
                    self._send_json(dashboard.start_capture(payload))
                elif self.path == "/api/record/stop":
                    self._send_json(dashboard.stop_capture())
                elif self.path == "/api/copy_latest":
                    self._send_json(dashboard.copy_latest())
                elif self.path == "/api/actuator":
                    dashboard.publish_actuator(
                        str(payload.get("name", "")),
                        str(payload.get("actuator_type", "")),
                        float(payload.get("value", 0.0)),
                    )
                    self._send_json({"success": True})
                elif self.path == "/api/stop_all":
                    dashboard.stop_all()
                    self._send_json({"success": True})
                else:
                    self.send_error(404)

            def log_message(self, format: str, *args) -> None:
                dashboard.get_logger().debug(format % args)

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                body = self.rfile.read(length).decode("utf-8")
                return json.loads(body) if body else {}

            def _send_json(self, payload: dict[str, Any]) -> None:
                data = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def _send_text(self, text: str, content_type: str) -> None:
                data = text.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        return Handler

    def destroy_node(self) -> bool:
        self.httpd.shutdown()
        self.httpd.server_close()
        self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OperatorDashboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
