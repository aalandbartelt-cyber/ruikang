# -*- coding: utf-8 -*-
"""
D435i 实时画面预览 — MJPEG 推流到浏览器
浏览器: http://<狗IP>:8889
"""
import cv2
import threading
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
import pyrealsense2 as rs

# --- 1. 启动 D435i ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
print("[D435i] 彩色流已启动 → http://" + open('/etc/hostname').read().strip() + ".local:8889")

frame = None

# --- 2. 后台取流 ---
def capture_loop():
    global frame
    while True:
        frames = pipeline.wait_for_frames()
        color = frames.get_color_frame()
        if color:
            frame = np.asanyarray(color.get_data())

threading.Thread(target=capture_loop, daemon=True).start()

# --- 3. MJPEG HTTP ---
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            if frame is not None:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

HTTPServer(('0.0.0.0', 8889), Handler).serve_forever()
