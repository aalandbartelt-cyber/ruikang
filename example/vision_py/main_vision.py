# -*- coding: utf-8 -*-
import cv2
import json
import os
import numpy as np
import time
from collections import deque, Counter  # 用于防抖过滤的超级神器

# 引入你的三大视觉模块
from detectors.line_tracker import find_line_offset
from detectors.sign_detector import process_sign
from detectors.tag_detector import process_tag
from communication.udp_sender import UDPSender

def main():
    print("🚀 正在启动 main_vision.py (工业级防抖总控版 - 明显肉眼可见延迟版)...")

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

    # 临时强改：无视 config.json，强制用电脑摄像头进行本地调试
    # source = 0 

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

    print("--- 👁️ 视觉总控已启动！防抖机制已部署！按 'Ctrl + C' 或 'q' 退出 ---")

    prev_time = 0

    # ==========================================
    # 🛡️ 核心：防抖窗口初始化 (加大容量，肉眼可见的延迟！)
    # ==========================================
    WINDOW_SIZE = 15 # 扩大到 15 帧的记忆容量
    sign_history = deque(maxlen=WINDOW_SIZE)
    tag_history = deque(maxlen=WINDOW_SIZE)

    def get_stable_result(history_queue, current_raw_result):
        """
        滑动窗口投票器：计算最近 N 帧里的绝对多数
        """
        history_queue.append(current_raw_result)
        
        # 统计最近几帧里，各个结果出现的次数
        vote_counts = Counter(history_queue)
        
        # 选出得票最高的那一个
        top_item, top_votes = vote_counts.most_common(1)[0]
        
        # 必须在 15 帧里拿到 10 票（绝对多数），才算真正稳定！
        if top_votes >= 10:
            return top_item
        else:
            return "NONE"

    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("⌛ 等待画面中或视频已结束...")
                break

            # ==========================================
            # 🟢 1. 视觉流水线 (获取原始波动数据)
            # ==========================================
            # 寻迹模块：传替身，防污染
            offset = find_line_offset(frame.copy(), threshold=line_threshold)
            
            # Tag 检测 (ArUco / C标志) -> 先执行，画面最干净
            raw_tag, frame = process_tag(frame)
            
            # Sign 检测 (警示牌) -> 后执行，不受红色圈圈干扰
            raw_sign, frame = process_sign(frame)
            
            # ==========================================
            # 🛡️ 2. 启动防抖过滤！(洗掉跳变数据)
            # ==========================================
            stable_tag = get_stable_result(tag_history, raw_tag)
            stable_sign = get_stable_result(sign_history, raw_sign)

            # ==========================================
            # 🔵 3. 数据打包与发送 (只发稳定的数据)
            # ==========================================
            if offset is not None:
                is_aruco = "ARUCO" in stable_tag

                payload = {
                    "line_offset": float(offset),
                    "warning_sign": stable_sign,     # 过滤后的稳定标志
                    "platform_tag": stable_tag,      # 过滤后的稳定 Tag
                    "aruco_detected": is_aruco
                }
                sender.send_data(payload)
                
            # ==========================================
            # 🟣 4. 联调显示与帧率计算
            # ==========================================
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            # 屏幕上同时显示 [原始数据] 和 [防抖后数据]
            cv2.putText(frame, f"FPS:{int(fps)} | Offset:{offset if offset else 'N/A'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Raw Tag: {raw_tag} -> STABLE: {stable_tag}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)
            cv2.putText(frame, f"Raw Sign: {raw_sign} -> STABLE: {stable_sign}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
            
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