# -*- coding: utf-8 -*-
"""
keyboard_mock.py — 键盘模拟视觉 UDP 数据发送
==============================================
用途：5.7 三人 Mock 联调时替代 Y 的视觉进程
用法：python keyboard_mock.py [--ip 127.0.0.1] [--port 8080]

键盘映射：
  a     — 切换 ArUco 检测 ON/OFF（默认 ON 时发送居中坐标 320）
  1     — ArUco 偏左 (cx=250, 需右转对齐)
  2     — ArUco 居中 (cx=320, 已对齐)
  3     — ArUco 偏右 (cx=390, 需左转对齐)
  l     — 切换黑线：有线(offset=5) / 无线(offset=999)
  s     — 切换警示牌：NONE / ELECTRIC / OXIDANT / RADIATION
  p     — 切换 platform_tag：0 / 1 / 2
  q/ESC — 退出

发送的 JSON 格式（与 main_vision.py 完全一致）：
  {
    "warning_sign": "NONE",
    "platform_tag": 0,
    "aruco_detected": true,
    "aruco_center_x": 320,
    "aruco_center_y": 240,
    "line_offset": 5,
    "turn_trend": 0,
    "depth_front": 1.5,
    "depth_left": 1.5,
    "depth_right": 1.5
  }
"""

import socket
import json
import time
import sys
import threading
import os

# 跨平台键盘读取
try:
    import msvcrt
    _WIN = True
except ImportError:
    import tty, termios
    _WIN = False

def get_key():
    """非阻塞读键，无输入返回 None"""
    if _WIN:
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore').lower()
    else:
        import sys, select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
    return None

def make_payload(state):
    """根据当前模拟状态构造 JSON 负载"""
    sign_map = {0: "NONE", 1: "ELECTRIC", 2: "OXIDANT", 3: "RADIATION"}
    return {
        "warning_sign": sign_map[state["sign_idx"]],
        "platform_tag": state["platform_tag"],
        "aruco_detected": state["aruco_on"],
        "aruco_center_x": state["aruco_cx"] if state["aruco_on"] else -1,
        "aruco_center_y": 240 if state["aruco_on"] else -1,
        "line_offset": state["line_offset"],
        "turn_trend": 0,
        "depth_front": 1.5,
        "depth_left": 1.5,
        "depth_right": 1.5,
    }

def print_help():
    print("""
  ╔══════════════════════════════════════════╗
  ║   🎮 State04 键盘 Mock 已启动             ║
  ╠══════════════════════════════════════════╣
  ║ a = ArUco ON/OFF    1/2/3 = cx 偏移     ║
  ║ l = 黑线 ON/OFF     s = 切换警示牌      ║
  ║ p = platform_tag     q = 退出           ║
  ╚══════════════════════════════════════════╝
  测试 State04 推荐步骤:
    1. 按 l 确保黑线 ON
    2. 按 a 开启 ArUco, 按 2 对齐
    3. 观察 C++ 端 APPROACH → ALIGN 流转
    4. 对齐完成后状态机自动进入后续阶段
""")

def main():
    ip   = sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == "--ip" else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[1] == "--port" else 8080
    # 解析 --ip 和 --port（支持同时出现）
    for i, arg in enumerate(sys.argv):
        if arg == "--ip" and i + 1 < len(sys.argv):
            ip = sys.argv[i + 1]
        if arg == "--port" and i + 1 < len(sys.argv):
            port = int(sys.argv[i + 1])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"📡 UDP → {ip}:{port}")

    state = {
        "aruco_on": False,
        "aruco_cx": 320,
        "line_offset": 5,
        "sign_idx": 0,
        "platform_tag": 0,
    }

    print_help()
    print(" " * 40)

    interval = 0.05  # 20Hz
    count = 0

    try:
        while True:
            key = get_key()
            if key:
                if key == 'q' or key == '\x1b':
                    break
                elif key == 'a':
                    state["aruco_on"] = not state["aruco_on"]
                    print(f"  🔄 ArUco = {'ON' if state['aruco_on'] else 'OFF'}  cx={state['aruco_cx']}")
                elif key == '1':
                    state["aruco_cx"] = 250
                    print(f"  ◀  ArUco cx=250 (偏左，需右转)")
                elif key == '2':
                    state["aruco_cx"] = 320
                    print(f"  ●  ArUco cx=320 (居中)")
                elif key == '3':
                    state["aruco_cx"] = 390
                    print(f"  ▶  ArUco cx=390 (偏右，需左转)")
                elif key == 'l':
                    state["line_offset"] = 999 if state["line_offset"] < 100 else 5
                    print(f"  📏 line_offset = {state['line_offset']}")
                elif key == 's':
                    state["sign_idx"] = (state["sign_idx"] + 1) % 4
                    names = ["NONE", "ELECTRIC", "OXIDANT", "RADIATION"]
                    print(f"  ⚠  warning_sign = {names[state['sign_idx']]}")
                elif key == 'p':
                    state["platform_tag"] = (state["platform_tag"] + 1) % 3
                    print(f"  🏷  platform_tag = {state['platform_tag']}")

            payload = make_payload(state)
            data = json.dumps(payload).encode('utf-8')
            sock.sendto(data, (ip, port))

            if count % 40 == 0:  # 每 2 秒
                tag = "ARUCO" if state["aruco_on"] else "----"
                print(f"  [{tag}] cx={state['aruco_cx']:3d}  "
                      f"line={state['line_offset']:3.0f}  "
                      f"sign={payload['warning_sign']:10s}  "
                      f"plat={state['platform_tag']}", end='\r')
            count += 1
            time.sleep(interval)

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print("\n👋 Mock 已退出")

if __name__ == "__main__":
    main()
