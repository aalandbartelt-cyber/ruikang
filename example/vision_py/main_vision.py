# -*- coding: utf-8 -*-
import cv2
import json
import os
import numpy as np
import time
from collections import deque, Counter  # 防抖过滤神器

from detectors.line_tracker import find_line_offset
from detectors.sign_detector import process_sign
from detectors.tag_detector import process_tag
from communication.udp_sender import UDPSender

def main():
    print("🚀 正在启动 main_vision.py (双摄并发 + 工业级防抖版)...")

    # ==========================================
    # ⚠️ 调试开关：下周上真狗联调时，请务必把这里改成 False！
    # ==========================================
    IS_LOCAL_SINGLE_CAM_TEST = True  
    
    if IS_LOCAL_SINGLE_CAM_TEST:
        print("🛠️ 【警告】当前处于本地单摄白嫖模式！仅供无真狗时测试逻辑使用！")

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

    # 2. 获取双摄参数与网络参数
    track_source = config.get("track_camera_id", 0)
    front_source = config.get("front_camera_id", "v4l2src...") 
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

    # ==========================================
    # 📷 4. 启动摄像头
    # ==========================================
    print(f"📷 正在启动 [寻迹] 专属摄像头 (ID: {track_source})...")
    cap_track = cv2.VideoCapture(track_source)

    if not cap_track.isOpened():
        print("❌ 错误：无法打开 [寻迹] 摄像头！")
        return

    if not IS_LOCAL_SINGLE_CAM_TEST:
        # 真车模式：老老实实打开前置图传
        print(f"📷 正在启动 [前方识别] 专属摄像头 (Source: {front_source})...")
        if isinstance(front_source, str) and ("udpsrc" in front_source or "v4l2src" in front_source):
            cap_front = cv2.VideoCapture(front_source, cv2.CAP_GSTREAMER)
        else:
            cap_front = cv2.VideoCapture(front_source)
            
        if not cap_front.isOpened():
            print("❌ 错误：无法打开 [前方识别] 摄像头！")
            return
    else:
        # 宿舍模式：不打开前置，假装它存在
        cap_front = None

    print("--- 👁️ 视觉总控已启动！按 'Ctrl + C' 或 'q' 退出 ---")

    prev_time = 0

    # 🛡️ 5. 防抖窗口初始化
    WINDOW_SIZE = 15 
    sign_history = deque(maxlen=WINDOW_SIZE)
    tag_history = deque(maxlen=WINDOW_SIZE)

    def get_stable_result(history_queue, current_raw_result):
        history_queue.append(current_raw_result)
        vote_counts = Counter(history_queue)
        top_item, top_votes = vote_counts.most_common(1)[0]
        if top_votes >= 10:
            return top_item
        else:
            return "NONE"

    try:
        while True:
            # 🌟 核心：画面读取逻辑分流
            ret_t, frame_track = cap_track.read()
            if not ret_t:
                print("⌛ 寻迹摄像头画面中断...")
                break
                
            if IS_LOCAL_SINGLE_CAM_TEST:
                # 宿舍模式：强行白嫖寻迹的画面给前方
                ret_f = True
                frame_front = frame_track.copy()
            else:
                # 真车模式：各自读各自的
                ret_f, frame_front = cap_front.read()
                if not ret_f:
                    print("⌛ 前方图传画面中断...")
                    break

            # ==========================================
            # 🟢 6. 双摄视觉流水线 (专职专办)
            # ==========================================
            # 【下视】：寻迹模块专心看地面
            offset = find_line_offset(frame_track.copy(), threshold=line_threshold)
            
            # 【前视】：Tag和Sign专心看前方
            raw_tag, frame_front = process_tag(frame_front)
            raw_sign, frame_front = process_sign(frame_front)
            
            # ==========================================
            # 🛡️ 7. 启动防抖过滤！
            # ==========================================
            stable_tag = get_stable_result(tag_history, raw_tag)
            stable_sign = get_stable_result(sign_history, raw_sign)

            # ==========================================
            # 🔵 8. 数据融合打包与发送
            # ==========================================
            if offset is not None:
                is_aruco = "ARUCO" in stable_tag

                payload = {
                    "line_offset": float(offset),
                    "warning_sign": stable_sign,     
                    "platform_tag": stable_tag,      
                    "aruco_detected": is_aruco
                }
                sender.send_data(payload)
                
            # ==========================================
            # 🟣 9. 联调显示与帧率计算
            # ==========================================
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if prev_time != 0 else 0
            prev_time = curr_time
            
            cv2.putText(frame_track, f"FPS:{int(fps)} | Offset:{offset if offset else 'N/A'}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame_front, f"Raw Tag: {raw_tag} -> STABLE: {stable_tag}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)
            cv2.putText(frame_front, f"Raw Sign: {raw_sign} -> STABLE: {stable_sign}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 255), 2)
            
            cv2.imshow("Vision: Ground Tracking (Track Cam)", frame_track)
            cv2.imshow("Vision: Front Detection (Front Cam)", frame_front)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n🛑 用户手动停止程序")
    finally:
        cap_track.release()
        if cap_front is not None:
            cap_front.release()
        sender.close()
        cv2.destroyAllWindows()
        print("🏁 程序已安全退出")

if __name__ == "__main__":
    main()