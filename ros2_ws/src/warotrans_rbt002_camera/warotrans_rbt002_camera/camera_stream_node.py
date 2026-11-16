"""Camera Module 3 Wide -> MJPEG over HTTP on :8081 (same contract the WaroTrans fleet backend relays:
RobotCamera:Robots:<code> = http://<robot>:8081/stream, see warotrans-system StreamRobotCamera).

  GET /stream        multipart/x-mixed-replace; boundary=frame
  GET /snapshot.jpg  latest frame
  GET /              small viewer page
  GET /health        JSON

rpicam-vid only runs while somebody watches (stops 5 s after the last viewer) and is restarted if it dies.
Needs Raspberry Pi's libcamera + rpicam-apps (tools/build_camera_stack.sh; Ubuntu noble's libcamera 0.2 has
no Pi 5 support).
"""

from __future__ import annotations

import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .mjpeg import JpegSplitter

PAGE = b"""<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>RBT-002 camera</title><style>body{margin:0;background:#000;display:flex;justify-content:center}
img{max-width:100%;max-height:100vh}</style></head><body><img src="/stream" alt="camera"></body></html>"""


class Camera:
    """Owns the rpicam-vid process and the latest frame."""

    def __init__(self, cmd: list[str], idle_stop_s: float, logger) -> None:
        self.cmd = cmd
        self.idle_stop_s = idle_stop_s
        self.log = logger
        self.cond = threading.Condition()
        self.frame: Optional[bytes] = None
        self.seq = 0
        self.viewers = 0
        self.last_viewer_t = 0.0
        self.proc: Optional[subprocess.Popen] = None
        self.frames_total = 0
        self.restarts = 0
        self.last_error = ""
        self._stop = threading.Event()
        threading.Thread(target=self._supervise, name="camera_supervisor", daemon=True).start()

    # viewer accounting --------------------------------------------------------------------------------
    def add_viewer(self) -> None:
        with self.cond:
            self.viewers += 1
            self.last_viewer_t = time.monotonic()
            self.cond.notify_all()

    def remove_viewer(self) -> None:
        with self.cond:
            self.viewers = max(0, self.viewers - 1)
            self.last_viewer_t = time.monotonic()

    def wait_frame(self, after_seq: int, timeout: float) -> tuple[Optional[bytes], int]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq != after_seq or self._stop.is_set(), timeout)
            return self.frame, self.seq

    # process supervision ------------------------------------------------------------------------------
    def _supervise(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            with self.cond:
                wanted = self.viewers > 0 or time.monotonic() - self.last_viewer_t < self.idle_stop_s
            running = self.proc is not None and self.proc.poll() is None
            if wanted and not running:
                if self.proc is not None:
                    self.restarts += 1
                    self.last_error = f"rpicam-vid exited with {self.proc.returncode}"
                    self.log.warning(self.last_error)
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 10.0)
                self._start()
            elif running and not wanted:
                self.log.info("no viewers: stopping rpicam-vid")
                self._kill()
            elif running:
                backoff = 1.0
            time.sleep(0.5)

    def _start(self) -> None:
        try:
            self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        except OSError as exc:
            self.last_error = f"cannot start {self.cmd[0]}: {exc}"
            self.log.error(self.last_error)
            self.proc = None
            time.sleep(5.0)
            return
        self.log.info("started: " + " ".join(self.cmd))
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()
        threading.Thread(target=self._stderr, args=(self.proc,), daemon=True).start()

    def _read(self, proc: subprocess.Popen) -> None:
        splitter = JpegSplitter()
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                return
            for frame in splitter.feed(chunk):
                with self.cond:
                    self.frame = frame
                    self.seq += 1
                    self.frames_total += 1
                    self.cond.notify_all()

    def _stderr(self, proc: subprocess.Popen) -> None:
        for line in proc.stderr:
            text = line.decode(errors="replace").strip()
            if "ERROR" in text or "error" in text:
                self.last_error = text[:200]
                self.log.warning(f"rpicam-vid: {text}")

    def _kill(self) -> None:
        p, self.proc = self.proc, None
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()

    def shutdown(self) -> None:
        self._stop.set()
        with self.cond:
            self.cond.notify_all()
        self._kill()


def make_handler(cam: Camera):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, fmt, *args):  # quiet
            pass

        def do_GET(self):  # noqa: N802
            path = self.path.split("?")[0]
            if path == "/stream":
                self._stream()
            elif path == "/snapshot.jpg":
                self._snapshot()
            elif path == "/health":
                body = json.dumps({"viewers": cam.viewers, "frames": cam.frames_total, "restarts": cam.restarts,
                                   "running": cam.proc is not None and cam.proc.poll() is None,
                                   "last_error": cam.last_error}).encode()
                self._send(200, "application/json", body)
            elif path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", PAGE)
            else:
                self._send(404, "text/plain", b"not found")

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _snapshot(self):
            cam.add_viewer()
            try:
                # newest frame if one exists, else wait (up to 8 s) for the first one after rpicam-vid starts
                frame, _ = (cam.frame, cam.seq) if cam.frame is not None else cam.wait_frame(cam.seq, 8.0)
            finally:
                cam.remove_viewer()
            if frame is None:
                self._send(503, "text/plain", b"camera not ready")
            else:
                self._send(200, "image/jpeg", frame)

        def _stream(self):
            cam.add_viewer()
            try:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                seq = -1
                while True:
                    frame, new_seq = cam.wait_frame(seq, 5.0)
                    if frame is None or new_seq == seq:
                        continue
                    seq = new_seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                cam.remove_viewer()

    return Handler


class CameraStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_stream")
        d = self.declare_parameter
        d("host", "0.0.0.0")
        d("port", 8081)
        d("width", 1280)
        d("height", 720)
        d("framerate", 15)
        d("quality", 80)
        d("rotation", 0)  # 0 or 180 (rpicam-vid --rotation) if the camera is mounted upside down
        d("rpicam_vid", "rpicam-vid")
        d("idle_stop_s", 5.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        cmd = [str(g("rpicam_vid")), "-t", "0", "--nopreview", "--codec", "mjpeg",
               "--width", str(g("width")), "--height", str(g("height")), "--framerate", str(g("framerate")),
               "--quality", str(g("quality")), "--rotation", str(g("rotation")), "-o", "-"]
        self.cam = Camera(cmd, float(g("idle_stop_s")), self.get_logger())
        self.server = ThreadingHTTPServer((str(g("host")), int(g("port"))), make_handler(self.cam))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, name="camera_http", daemon=True).start()
        self.get_logger().info(f"MJPEG on http://{g('host')}:{g('port')}/stream ({g('width')}x{g('height')}@{g('framerate')})")
        self._diag = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_timer(2.0, self._publish_diag)

    def _publish_diag(self) -> None:
        cam = self.cam
        running = cam.proc is not None and cam.proc.poll() is None
        d = DiagnosticStatus(name="camera: stream", hardware_id="Camera Module 3 Wide")
        d.level = DiagnosticStatus.WARN if cam.last_error and not running and cam.viewers else DiagnosticStatus.OK
        d.message = "streaming" if running else ("idle (no viewers)" if not cam.viewers else "starting")
        d.values = [KeyValue(key=k, value=str(v)) for k, v in
                    (("viewers", cam.viewers), ("frames", cam.frames_total), ("restarts", cam.restarts),
                     ("last_error", cam.last_error))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [d]
        self._diag.publish(arr)

    def destroy_node(self) -> bool:
        self.server.shutdown()
        self.cam.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraStreamNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
