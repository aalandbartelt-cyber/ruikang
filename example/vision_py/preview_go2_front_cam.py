# -*- coding: utf-8 -*-
"""
Go2 内置前摄预览脚本 — MJPEG 推流到浏览器（不依赖显示器）
用法: python3 preview_go2_front_cam.py [网卡名，默认 eth0]
浏览器: http://<狗IP>:8888
"""
import sys
import cv2
import numpy as np
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

# --- 1. 初始化 DDS 通道 + VideoClient ---
sys.path.insert(0, 'path_here')  # TODO: 改成 SDK Python 的实际路径

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

if len(sys.argv) > 1:
    ChannelFactoryInitialize(0, sys.argv[1])
else:
    ChannelFactoryInitialize(0, "eth0")

client = VideoClient()
client.SetTimeout(3.0)
client.Init()
print("📷 Go2 前摄已连接")

# --- 2. MJPEG HTTP 服务 ---
frame = None

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            if frame is not None:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

# --- 3. 后台取流 ---
def capture_loop():
    global frame
    while True:
        code, data = client.GetImageSample()
        if code == 0:
            arr = np.frombuffer(bytes(data), dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

threading.Thread(target=capture_loop, daemon=True).start()

# --- 4. 启动 ---
import socket
host = socket.gethostname()
print(f"📡 浏览器打开: http://{host}.local:8888  或  http://<狗IP>:8888")
HTTPServer(('0.0.0.0', 8888), MJPEGHandler).serve_forever()
