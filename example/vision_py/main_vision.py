# -*- coding: utf-8 -*-
import cv2
import json
import os
import numpy as np
import time # 用于计算帧率

# 引入你的三大视觉模块
from detectors.line_tracker import find_line_offset
from detectors.sign_detector import process_sign
from detectors.tag_detector import process_tag
from communication.udp_sender import UDPSender

def main():
    print("🚀 正在启动 main_vision.py (全能视觉总控版)...")

    # 1. 加载配置
    current_path = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(current_path, 'config.json')

    if not os.path.exists(config_path):
        print(f"❌ 错误：在路径 {config_path} 未找到 config.json！")
        return

    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)
        print("✅ 成功读取 config.json")
    except Exception as e:
        print(f"❌ 错误：解析 config.json 失败: {e}")
        return

    # 2. 获取参数
    source = config.get("video_path", "test.mp4") if config.get("use_local_video", False) else config.get("camera_id", 0)
    line_threshold = config.get("black_line_threshold", 60)
    udp_ip = config.get("udp_ip", "127.0.0.1")
    udp_port = config.get("udp_port", 8080)

    # 3. 启动视觉 UDP 发送器
    try:
        sender = UDPSender(ip=udp_ip, port=udp_port)
        print(f"📡 UDP 发送器准备就绪，目标: {udp_ip}:{udp_port}")
    except Exception as e:
        print(f"❌ 错误：无法启动 UDP 发送器: {e}")
        return

    # 4. 开启视频流
    if isinstance(source, str) and ("udpsrc" in source or "v4l2src" in source):
        print(f"📷 正在使用 GStreamer 管道启动摄像头...")
        cap = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)
    else:
        print(f"📷 正在尝试直接打开视频源: {source}")
        cap = cv2.VideoCapture(source)
        
    if not cap.isOpened():
        print("❌ 错误：无法打开视频源！")
        return

    print("--- 👁️ 视觉总控已启动！寻迹与地标检测并行中！按 'Ctrl + C' 或 'q' 退出 ---")

    prev_time = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("⌛ 等待画面中或视频已结束...")
                break
                
            # 实车部署时如果画面方向正确，请注释掉翻转
            # frame = cv2.flip(frame, 1)

            # ==========================================
            # 🟢 视觉流水线 (防污染重构版)
            # ==========================================
            
            # 🛡️ 1. 寻迹模块：务必加上 .copy()！
            # 这样就算寻迹模块把画面切碎、缩小，也不会影响后续的识别
            offset = find_line_offset(frame.copy(), threshold=line_threshold)
            
            # 🔄 2. 调换顺序：让 Tag (红圈标志) 先执行！
            # 此时画面最干净，没有别人画的红色线条干扰它的红色遮罩
            tag_result, frame = process_tag(frame)
            
            # 🔄 3. 最后执行 Sign (黄三角警示牌)！
            # 因为它只找黄色，Tag 刚才就算在画面上画了红圈和绿圈，也完全干扰不到它
            sign_result, frame = process_sign(frame)
            
            # ==========================================
            # ==========================================
            # 🔵 数据打包与发送 (UDP 状态机)
            # ==========================================
            if offset is not None:
                # 判断是否扫到了真正的 ArUco (如果 tag_result 包含 "ARUCO" 字样)
                is_aruco = "ARUCO" in tag_result

                payload = {
                    "line_offset": float(offset),
                    "warning_sign": sign_result,     # 可能是 "FIRE", "ELECTRIC", "RADIATION", "NONE"
                    "platform_tag": tag_result,      # 可能是 "ARUCO_1", "WHITE-OUTER", "NONE" 等
                    "aruco_detected": is_aruco       # True 或 False
                }
                sender.send_data(payload)
            else:
                # 找不到线时，为了安全，可以发送一个默认包或者静默
                pass
                
            # ==========================================
            # 🟣 联调显示与帧率计算
            # ==========================================
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            # 在画面上打印总控状态，方便你一次性看到所有结果
            info_text = f"FPS:{int(fps)} | Offset:{offset if offset else 'N/A'} | Sign:{sign_result} | Tag:{tag_result}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cv2.imshow("Main Vision Pipeline", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n🛑 用户手动停止程序")
    finally:
        cap.release()
        sender.close()
        cv2.destroyAllWindows()
        print("🏁 程序已安全退出")

if __name__ == "__main__":
    main()