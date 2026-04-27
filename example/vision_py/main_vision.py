# -*- coding: utf-8 -*-
import cv2
import json
import os
import numpy as np

# 注意：请确保你的项目结构中 detectors、communication 文件夹存在
from detectors.line_tracker import find_line_offset
from communication.udp_sender import UDPSender
# 🌟 已删去：from hardware.lidar_node import LidarReader (彻底告别 Python 雷达！)

def main():
    print("🚀 正在启动 main_vision.py (纯视觉极速版)...")

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

    # 3. 启动视觉 UDP 发送器 (向 C++ 状态机发数据)
    try:
        sender = UDPSender(ip=udp_ip, port=udp_port)
        print(f"📡 UDP 发送器准备就绪，目标: {udp_ip}:{udp_port}")
    except Exception as e:
        print(f"❌ 错误：无法启动 UDP 发送器: {e}")
        return

    # 🌟 4. (已删去：雷达初始化)

    # 4. 开启视频流
    # 只要包含 udpsrc 或 v4l2src，就用 GStreamer
    if isinstance(source, str) and ("udpsrc" in source or "v4l2src" in source):
        print(f"📷 正在使用 GStreamer 管道启动摄像头...")
        cap = cv2.VideoCapture(source, cv2.CAP_GSTREAMER)
    else:
        print(f"📷 正在尝试直接打开视频源: {source}")
        cap = cv2.VideoCapture(source)
        
    if not cap.isOpened():
        print("❌ 错误：无法打开视频源！")
        print(f"当前尝试的 source 是: {source}")
        return

    print("--- 👁️ 视觉节点已启动！专心寻迹中！按 'Ctrl + C' 退出 ---")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("⌛ 等待画面中或视频已结束...")
                break
                
            # 调用寻迹算法获取偏移量
            offset = find_line_offset(frame, threshold=line_threshold)
            
            # 🌟 动态获取雷达距离代码已删除
            
            if offset is not None:
                payload = {
                    "line_offset": float(offset),
                    # 占位符：给 C++ 留个位置，C++ 会用底层的真实雷达距离覆盖它！
                    #"obstacle_distance": 9.99, 
                    "platform_tag": 0,
                    "warning_sign": "NONE",
                    "aruco_detected": False
                }
                sender.send_data(payload)
                
                # 调试打印（联调时可以取消注释看看）
                # print(f"[视觉发送] 黑线偏移量: {offset:.2f}")
            else:
                # 找不到线时保持静默
                pass
                
    except KeyboardInterrupt:
        print("\n🛑 用户手动停止程序")
    finally:
        cap.release()
        sender.close()
        # 🌟 安全关闭雷达监听代码已删除
        cv2.destroyAllWindows()
        print("🏁 程序已安全退出")

if __name__ == "__main__":
    main()