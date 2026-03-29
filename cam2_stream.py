#!/usr/bin/env python3
"""
cam2_stream.py — Arducam B0589 local camera → MJPEG HTTP on port 8081
Serves at:  http://localhost:8081/stream
"""
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

CAM_DEVICE = "/dev/video0"
PORT = 8081
FPS_CAP = 10        # frames/sec sent to browser
QUALITY = 55        # JPEG quality (lower = smaller, less CPU)

# ---- GStreamer pipeline ------------------------------------------------
# Capture at 640x480 YUY2 (camera native), scale down to 320x240 for
# browser rendering. Lower res = less decode CPU in Chromium.
GST_CMD = [
    "gst-launch-1.0", "-q",
    "v4l2src", f"device={CAM_DEVICE}",
    "!", f"video/x-raw,format=YUY2,width=640,height=480",
    "!", "videoconvert",
    "!", f"jpegenc", f"quality={QUALITY}",
    "!", "fdsink", "fd=1",
]

# ---- Shared latest JPEG frame (producer/consumer via lock) ----------------
_frame_lock = threading.Lock()
_latest_frame: bytes = b""
_last_frame_time: float = 0.0
WATCHDOG_TIMEOUT = 5.0  # seconds without new frame → restart pipeline


def gst_reader():
    """Reads MJPEG frames from GStreamer stdout, keeps only the latest.
    Self-recovering: watchdog kills stalled pipelines after 5s."""
    global _latest_frame, _last_frame_time
    BOUNDARY = b"\xff\xd8"   # JPEG SOI marker
    JPEG_END  = b"\xff\xd9"  # JPEG EOI marker

    while True:
        proc = None
        buf = b""
        try:
            proc = subprocess.Popen(GST_CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            _last_frame_time = time.time()
            print(f"[cam2] GStreamer started (PID {proc.pid})", flush=True)

            # Watchdog thread — kills GStreamer if no new frames
            def watchdog():
                while proc.poll() is None:
                    time.sleep(2)
                    if time.time() - _last_frame_time > WATCHDOG_TIMEOUT:
                        print(f"[cam2] Watchdog: no frame for {WATCHDOG_TIMEOUT}s — killing pipeline", flush=True)
                        try:
                            proc.kill()
                        except Exception:
                            pass
                        return

            wd = threading.Thread(target=watchdog, daemon=True)
            wd.start()

            while True:
                chunk = proc.stdout.read(4096)
                if not chunk:
                    raise RuntimeError("GStreamer stdout closed")
                buf += chunk
                while True:
                    start = buf.find(BOUNDARY)
                    if start == -1:
                        buf = b""
                        break
                    end = buf.find(JPEG_END, start + 2)
                    if end == -1:
                        buf = buf[start:]
                        break
                    frame = buf[start:end + 2]
                    with _frame_lock:
                        _latest_frame = frame
                        _last_frame_time = time.time()
                    buf = buf[end + 2:]
        except Exception as e:
            print(f"[cam2] Pipeline error: {e} — restarting in 3s", flush=True)
        finally:
            if proc:
                try:
                    proc.kill()
                except Exception:
                    pass
        time.sleep(3)


# ---- MJPEG HTTP Handler ---------------------------------------------------
class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress per-request logs

    def do_GET(self):
        path = self.path.split("?")[0]  # strip query string (?t=...)
        # Single JPEG snapshot — works in all browsers
        if path == "/snapshot":
            with _frame_lock:
                frame = _latest_frame
            if frame:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(frame)
            else:
                self.send_response(503)
                self.end_headers()
            return

        if path not in ("/stream", "/"):
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        interval = 1.0 / FPS_CAP
        try:
            while True:
                t0 = time.time()
                with _frame_lock:
                    frame = _latest_frame
                if frame:
                    try:
                        self.wfile.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n" +
                            frame + b"\r\n"
                        )
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                elapsed = time.time() - t0
                time.sleep(max(0, interval - elapsed))
        except Exception:
            pass


if __name__ == "__main__":
    # Start GStreamer reader in background thread
    t = threading.Thread(target=gst_reader, daemon=True)
    t.start()

    print(f"[cam2] MJPEG server on http://0.0.0.0:{PORT}/stream")
    server = HTTPServer(("0.0.0.0", PORT), MJPEGHandler)
    server.serve_forever()
