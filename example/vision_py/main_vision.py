# -*- coding: utf-8 -*-
"""
main_vision.py V2.5 (最终交付版)
- 适配扁平化 config.json 及其 null 值逻辑
- 生产环境安全模式：默认关闭 GUI 窗口，防止实车崩溃
- 交付标准：输出中心像素坐标 + 映射 platform_tag (0/1/2)
"""
import cv2
import json
import os
import numpy as np
import time
from collections import deque, Counter

from detectors.line_tracker import find_line_offset
from detectors.sign_detector import process_sign
from detectors.red_dot_detector import detect_red_dot  # 👈 新增这一行

# ⚠️ 兼容层：处理 tag_detector 接口
try:
    from detectors.tag_detector import process_tag
    TAG_DETECTOR_AVAILABLE = True
    print("✅ tag_detector 模块加载成功")
except ImportError as e:
    print(f"⚠️ tag_detector 不可用，使用 stub 代替: {e}")
    TAG_DETECTOR_AVAILABLE = False
    def process_tag(frame):
        return "NONE", -1, -1, frame 

from communication.udp_sender import UDPSender
from hardware.realsense_handler import RealSenseHandler
from hardware.go2_camera import Go2FrontCamera

def get_stable(history_deque):
    """防抖函数：连续出现 3 次才认为有效"""
    if not history_deque:
        return "NONE"
    counts = Counter(history_deque)
    most_common, count = counts.most_common(1)[0]
    return most_common if count >= 3 else "NONE"

def main():
    print("🚀 启动 main_vision.py V2.5 (机器人实车就绪版)")
    
    # ================= 1. 安全加载最新版扁平化配置 =================
    current_path = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_path, 'config.json')
    config = {}
    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)

    # 提取字段（适配 M 的最新格式）
    udp_ip = config.get('udp_ip', '127.0.0.1')
    udp_port = config.get('udp_port', 8080)
    enable_go2_cam = config.get('enable_go2_cam', True)
    front_cam_source = config.get('front_camera_source', 'v4l2')  # unitree 或 v4l2
    front_cam_id = config.get('front_camera_id')
    network_iface = config.get('network_interface', 'eth0')

    # ================= 2. 初始化硬件 (防弹衣模式) =================
    # A. 深度相机 (D435i)
    try:
        rs_handler = RealSenseHandler()
        if not rs_handler.start():
            print("❌ D435i 启动失败：未连接硬件")
            rs_handler = None
    except Exception as e:
        print(f"⚠️ D435i 初始化异常: {e}")
        rs_handler = None

    # B. 前置摄像头 (Go2 自带 或 外接USB)
    cap_front = None
    if enable_go2_cam and front_cam_id is not None:
        if front_cam_source == "unitree":
            try:
                cap_front = Go2FrontCamera(network_iface)
                print(f"📷 前置摄像头已启用: unitree://go2_front (VideoClient)")
            except Exception as e:
                print(f"⚠️ Go2 前摄初始化失败: {e}")
        else:
            # v4l2 模式：直接用 cv2.VideoCapture 打开 /dev/videoX
            cap_front = cv2.VideoCapture(front_cam_id)
            if cap_front.isOpened():
                print(f"📷 前置摄像头已启用: {front_cam_id} (V4L2)")
            else:
                print(f"⚠️ 无法打开 V4L2 设备: {front_cam_id}")
                cap_front = None
    else:
        print("⚠️ 前置摄像头已禁用 (config 中设为 null 或 enable_go2_cam=false)")

    # C. UDP 通信
    try:
        sender = UDPSender(udp_ip, udp_port)
        print(f"📡 UDP 链路就绪：{udp_ip}:{udp_port}")
    except Exception as e:
        print(f"⚠️ UDP 链路初始化失败: {e}")
        sender = None

    # 防抖队列
    sign_history = deque(maxlen=10)
    tag_history = deque(maxlen=15)
    log_tick = 0
    
    print("--- 👁️ 视觉系统开始循环，按 Ctrl+C 或 Q 退出 ---")

    try:
        while True:
            # ========== A. D435i：寻迹与避障 ==========
            if rs_handler is None or not rs_handler.update():
                color_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                depth_front, depth_left, depth_right = 0.0, 0.0, 0.0
            else:
                color_frame = rs_handler.get_color_frame()
                depth_front, depth_left, depth_right = rs_handler.get_obstacle_distances()

            # 接收两个返回值：当前偏移量 和 弯道趋势
            offset, trend = find_line_offset(color_frame, threshold=config.get("black_line_threshold", 60))

            # ★ 红点检测：用 D435i 朝下看地面（不是 Go2 前摄！）
            red_dot_detected, red_dot_cx, _ = detect_red_dot(color_frame)

            # ========== B. 前置相机：警示牌与标识识别 ==========
            stable_sign = "NONE"
            stable_tag = "NONE"
            tag_cx, tag_cy = -1, -1
            frame_front = None

            if cap_front is not None:
                ret, frame_front = cap_front.read()
                if ret:
                    # 识别警示牌
                    raw_sign, _ = process_sign(frame_front)
                    sign_history.append(raw_sign)
                    stable_sign = get_stable(sign_history)

                    # 识别 ArUco/标志 (必须确保 tag_detector 字典为 4X4_50)
                    raw_tag, raw_cx, raw_cy, frame_front = process_tag(frame_front)
                    tag_history.append(raw_tag)
                    stable_tag = get_stable(tag_history)

                    if raw_cx != -1:
                        tag_cx, tag_cy = raw_cx, raw_cy

            # ========== C. 逻辑映射 (0/1/2) ==========
            platform_id = 0
            # 💡 确保这里包含 "ARUCO_0"
            if "ARUCO_1" in stable_tag or "ARUCO_37" in stable_tag or "ARUCO_0" in stable_tag:
                platform_id = 1  # 对应 1 号台阶/任务
            elif "ARUCO_2" in stable_tag or "BLACK" in stable_tag:
                platform_id = 2  # 对应 2 号台阶/任务

            

            # ========== D. UDP 数据封装与发送 ==========
            if sender:
                payload = {
                    "warning_sign": stable_sign,
                    "platform_tag": platform_id,
                    "aruco_detected": "ARUCO" in stable_tag,
                    "aruco_center_x": int(tag_cx),
                    "aruco_center_y": int(tag_cy),
                    "line_offset": int(offset),
                    "turn_trend": int(trend),
                    "depth_front": float(depth_front),
                    "depth_left": float(depth_left),
                    "depth_right": float(depth_right),
                    # 红点定位字段（State07 RED_DOT_ALIGN 依赖）
                    "red_dot_detected": red_dot_detected,
                    "red_dot_center_x": int(red_dot_cx),
                }
                sender.send_data(payload)

                # 调试日志：每秒打印一次警示牌 + 红点状态（深度/偏移已由 udp_sender 每帧打印）
                if log_tick % 30 == 0:
                    print(f"[VIS] Sign={stable_sign} | Tag={stable_tag} | "
                          f"RedDot={red_dot_detected}(cx={red_dot_cx}) | "
                          f"off={offset:.0f} trend={trend:.0f}")

            # ========== E. 画面预览 (🚨 提交前请保持注释状态) ==========
            # if cap_front is not None and frame_front is not None:
            #     if tag_cx != -1:
            #         cv2.drawMarker(frame_front, (int(tag_cx), int(tag_cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
            #     cv2.putText(frame_front, f"Stable: {stable_tag} ID:{platform_id}", (10, 30), 
            #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)
            #     cv2.imshow("Go2 Vision Debug", frame_front)

            log_tick += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n👋 接收到退出信号")
    finally:
        print("🏁 正在清理资源并退出...")
        if cap_front is not None:
            cap_front.release()
        if rs_handler is not None:
            rs_handler.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()