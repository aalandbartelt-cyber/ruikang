# -*- coding: utf-8 -*-
import socket
import json

class UDPSender:
    def __init__(self, ip="127.0.0.1", port=8080):
        """初始化 UDP 发送器，默认发送到本地 8080 端口"""
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_data(self, data_dict):
        """
        接收一个字典，将其转换为符合协议的 JSON 字符串并发送
        """
        try:
            # 将字典转换为 JSON 格式的字符串
            json_str = json.dumps(data_dict)
            # UDP 发送需要将字符串编码为字节流
            self.sock.sendto(json_str.encode('utf-8'), (self.ip, self.port))
            # 调试打印，方便你在狗身上看到发了什么
            print(f"[发送成功] -> {json_str}")
        except Exception as e:
            print(f"[发送失败] 错误信息: {e}")

    def close(self):
        self.sock.close()

# 为了兼容你之前的代码，保留这个方法名作为快捷方式
    def send_offset(self, offset):
        payload = {
            "line_offset": float(offset),
            "obstacle_distance": 0.0,
            "platform_tag": 0,
            "warning_sign": "NONE",
            "aruco_detected": False
        }
        self.send_data(payload)