# 测试视觉数据代码
import socket
import json
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 8080

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"开始发送测试数据...")

try:
    # 1. 先发送距离障碍物 2.0 米的数据（狗应该继续走）
    print(">>> 模拟：距离障碍物 2.0 米")
    for _ in range(20): # 发 20 次，也就是持续 2 秒
        payload = {
            "line_offset": 0.0,
            "obstacle_distance": 2.0, # 大于 0.8，狗不会跳
            "platform_tag": 0,
            "warning_sign": "NONE",
            "aruco_detected": False
        }
        sock.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))
        time.sleep(0.1)
    
    # 2. 突然发现障碍物！（距离小于 0.8 米）
    print(">>> 模拟：发现障碍物！距离 0.5 米")
    for _ in range(50): # 持续发送，让 C++ 有时间完成 2 秒的跳跃动作
        payload = {
            "line_offset": 0.0,
            "obstacle_distance": 0.5, # 小于 0.8，触发跳跃！
            "platform_tag": 0,
            "warning_sign": "NONE",
            "aruco_detected": False
        }
        sock.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))
        time.sleep(0.1)

    print(">>> 测试脚本发送完毕。")

except KeyboardInterrupt:
    print("\n停止发送。")