# -*- coding: utf-8 -*-
import socket
import json


class UDPSender:
    def __init__(self, ip="127.0.0.1", port=8080):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_data(self, data_dict):
        """
        发送字典 → JSON
        新增字段：depth_front, depth_left, depth_right
        """
        try:
            json_str = json.dumps(data_dict)
            self.sock.sendto(json_str.encode('utf-8'), (self.ip, self.port))
            # 精简打印（只打印关键字段）
            preview = (
                f"off={data_dict.get('line_offset', 0):.0f} "
                f"DF={data_dict.get('depth_front', 9.99):.2f} "
                f"DL={data_dict.get('depth_left', 9.99):.2f} "
                f"DR={data_dict.get('depth_right', 9.99):.2f}"
            )
            print(f"[UDP] {preview}")
        except Exception as e:
            print(f"[UDP 失败] {e}")

    def close(self):
        self.sock.close()

    # 兼容旧调用
    def send_offset(self, offset):
        self.send_data({
            "line_offset": float(offset),
            "depth_front": 9.99,
            "depth_left": 9.99,
            "depth_right": 9.99,
        })