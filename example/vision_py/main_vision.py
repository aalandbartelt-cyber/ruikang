# -*- coding: utf-8 -*-
"""
main_vision.py V2.0
- D435i：寻迹（彩色） + 避障（深度）
- Go2 自带摄像头：警示牌 / 平台标志 / ArUco

⚠️ 注意：本版本临时屏蔽 cv2.aruco（OpenCV 4.1.1 不带），
   ArUco 功能等 Y 来后用 OpenCV 4.13 venv 启动时再开启
"""
import cv2
import json
import os
import numpy as np
import time
from collections import deque, Counter

from detectors.line_tracker import find_line_offset
from detectors.sign_detector import process_sign

# ⚠️ 关键：tag_detector 含 cv2.aruco，Python 3.8 + cv2 4.1.1 不可用
# 临时用一个 stub 函数代替
try:
    from detectors.tag_detector import process_tag
    TAG_DETECTOR_AVAILABLE = True
    print("✅ tag_detector 可用")
except ImportError as e:
    print(f"⚠️ tag_detector 不可用（cv2.aruco 缺失），使用 stub: {e}")
    TAG_DETECTOR_AVAILABLE = False
    def process_tag(frame):
        return "NONE", frame  # stub：始终返回无标志

from communication.udp_sender import UDPSender
from hardware.realsense_handler import RealSenseHandler


def main():
    print("🚀 启动 main_vision.py V2.0 (D435i + Go2 双相机)")

    # ==========================================
    # 配置
    # ==========================================
    current_path = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_path, 'config.json')
    if not os.path.exists(config_path):
        print(f"❌ 缺少 config.json：{config_path}")
        return
    with open(config_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    line_threshold = config.get("black_line_threshold", 60)
    udp_ip = config.get("udp_ip", "127.0.0.1")
    udp_port = config.get("udp_port", 8080)
    front_source = config.get("front_camera_id", None)  # Go2 自带摄像头
    enable_go2_cam = config.get("enable_go2_cam", True)

    # ==========================================
    # 启动 UDP
    # ==========================================
    sender = UDPSender(ip=udp_ip, port=udp_port)
    print(f"📡 UDP 准备就绪：{udp_ip}:{udp_port}")

    # ==========================================
    # 启动 D435i
    # ==========================================
    rs_handler = None
    try:
        rs_handler = RealSenseHandler()
    except Exception as e:
        print(f"❌ D435i 启动失败：{e}")
        print("⚠️ 没有 D435i，避障功能将不可用")

    if rs_handler is None:
        print("❌ 没有 D435i，程序退出")
        return

    # ==========================================
    # 启动 Go2 自带摄像头（用于警示牌/标志）
    # ==========================================
    cap_front = None
    if enable_go2_cam and front_source is not None:
        print(f"📷 启动 Go2 自带摄像头: {front_source}")
        if isinstance(front_source, str) and ("v4l2src" in front_source or "udpsrc" in front_source):
            cap_front = cv2.VideoCapture(front_source, cv2.CAP_GSTREAMER)
        else:
            cap_front = cv2.VideoCapture(front_source)
        if not cap_front.isOpened():
            print("⚠️ Go2 摄像头打开失败，警示牌识别将不可用")
            cap_front = None
        else:
            print("✅ Go2 摄像头启动成功")

    # ==========================================
    # 防抖
    # ==========================================
    WINDOW_SIZE = 15
    sign_history = deque(maxlen=WINDOW_SIZE)
    tag_history = deque(maxlen=WINDOW_SIZE)

    def get_stable(history, raw):
        history.append(raw)
        top, votes = Counter(history).most_common(1)[0]
        return top if votes >= 10 else "NONE"

    print("--- 👁️ 视觉总控启动！按 q 退出 ---")
    prev_time = 0

    try:
        while True:
            # ========== 1. 拉 D435i 帧 ==========
            if not rs_handler.update():
                print("⌛ D435i 帧丢失")
                continue

            color_frame = rs_handler.get_color()
            if color_frame is None:
                continue

            # ========== 2. 寻迹（用 D435i 彩色帧）==========
            offset = find_line_offset(color_frame.copy(), threshold=line_threshold)

            # ========== 3. 警示牌 / 标志（用 Go2 自带摄像头）==========
            raw_sign = "NONE"
            raw_tag = "NONE"
            if cap_front is not None:
                ret_f, frame_front = cap_front.read()
                if ret_f:
                    raw_tag, frame_front = process_tag(frame_front)
                    raw_sign, frame_front = process_sign(frame_front)
            else:
                frame_front = None

            stable_sign = get_stable(sign_history, raw_sign)
            stable_tag = get_stable(tag_history, raw_tag)

            # ========== 4. 深度数据 ==========
            depth_front = rs_handler.get_front_distance()
            depth_left = rs_handler.get_left_distance()
            depth_right = rs_handler.get_right_distance()

            # ========== 5. 发送 UDP ==========
            if offset is not None:
                payload = {
                    "line_offset": float(offset),
                    "warning_sign": stable_sign,
                    "platform_tag": stable_tag,
                    "aruco_detected": "ARUCO" in stable_tag,
                    "depth_front": float(depth_front),
                    "depth_left": float(depth_left),
                    "depth_right": float(depth_right),
                }
                sender.send_data(payload)

            # ========== 6. 显示（调试）==========
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
         
            display_color = color_frame.copy()
            cv2.putText(
                display_color,
                f"FPS:{int(fps)} Off:{offset} F:{depth_front:.2f} L:{depth_left:.2f} R:{depth_right:.2f}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2
            )
            cv2.imshow("D435i Color (Tracking + Depth Status)", display_color)

            if frame_front is not None:
                cv2.putText(
                    frame_front,
                    f"Sign:{stable_sign} Tag:{stable_tag}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 100, 255), 2
                )
                cv2.imshow("Go2 Front (Sign/Tag)", frame_front)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
           
        
    except KeyboardInterrupt:
        print("\n🛑 用户停止")
    finally:
        if rs_handler is not None:
            rs_handler.stop()
        if cap_front is not None:
            cap_front.release()
        sender.close()
        cv2.destroyAllWindows()
        print("🏁 安全退出")


if __name__ == "__main__":
    main()