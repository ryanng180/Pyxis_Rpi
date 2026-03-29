#!/usr/bin/env python3
"""
cam1_relay.py — H264 RTP from Jetson (UDP port 5700) → MJPEG HTTP on port 8080
Serves at:  http://localhost:8080/stream

Jetson sends H264 RTP using:
  appsrc ! x264enc ! rtph264pay ! udpsink host=192.168.10.2 port=5700
"""
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

RTP_PORT = 5700   # UDP port Jetson sends H264 RTP to
HTTP_PORT = 8080
JPEG_QUALITY = 55  # lower = smaller frames, less browser CPU
FPS_CAP = 10       # frames/sec sent to browser

# ---- GStreamer pipeline ------------------------------------------------
# udpsrc → RTP depay → H264 decode → scale (optional) → JPEG encode → stdout
GST_CMD = [
    "gst-launch-1.0", "-q",
    "udpsrc", f"port={RTP_PORT}",
    "!", "application/x-rtp,media=video,payload=96,encoding-name=H264",
    "!", "rtph264depay",
    "!", "avdec_h264", "max-threads=2",
    "!", "videoconvert",
    "!", "video/x-raw,format=BGR",
    "!", "jpegenc", f"quality={JPEG_QUALITY}",
    "!", "fdsink", "fd=1",
]

_frame_lock = threading.Lock()
_latest_frame: bytes = b""
_last_frame_time: float = 0.0
WATCHDOG_TIMEOUT = 10.0  # longer for cam1 since Jetson may not always stream


def gst_reader():
    global _latest_frame, _last_frame_time
    BOUNDARY = b"\xff\xd8"
    JPEG_END  = b"\xff\xd9"

    while True:
        proc = None
        buf = b""
        try:
            proc = subprocess.Popen(GST_CMD, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            _last_frame_time = time.time()
            print(f"[cam1] GStreamer started (PID {proc.pid}), waiting for Jetson RTP on UDP:{RTP_PORT}...", flush=True)

            def watchdog():
                while proc.poll() is None:
                    time.sleep(2)
                    if _last_frame_time > 0 and time.time() - _last_frame_time > WATCHDOG_TIMEOUT:
                        print(f"[cam1] Watchdog: no frame for {WATCHDOG_TIMEOUT}s — killing pipeline", flush=True)
                        try:
                            proc.kill()
                        except Exception:
                            pass
                        return

            wd = threading.Thread(target=watchdog, daemon=True)
            wd.start()

            while True:
                chunk = proc.stdout.read(8192)
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
            print(f"[cam1] Pipeline error: {e} — restarting in 3s", flush=True)
        finally:
            if proc:
                try:
                    proc.kill()
                except Exception:
                    pass
        time.sleep(3)


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

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
    t = threading.Thread(target=gst_reader, daemon=True)
    t.start()

    print(f"[cam1] MJPEG relay on http://0.0.0.0:{HTTP_PORT}/stream")
    server = HTTPServer(("0.0.0.0", HTTP_PORT), MJPEGHandler)
    server.serve_forever()
