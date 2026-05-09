# -*- coding: utf-8 -*-
"""
hardware/go2_camera.py
Go2 内置前摄封装 — 通过 Unitree VideoClient (DDS) 拉流
对外暴露 cv2.VideoCapture 兼容接口 (.read() / .release())
"""
import cv2
import numpy as np
import sys


class Go2FrontCamera:
    def __init__(self, network_interface="eth0"):
        # 动态导入，避免未安装 SDK 时影响其它模块
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient

        ChannelFactoryInitialize(0, network_interface)

        self._client = VideoClient()
        self._client.SetTimeout(3.0)
        self._client.Init()
        self._opened = True
        print(f"[Go2Cam] 前置摄像头已连接 (interface={network_interface})")

    def read(self):
        """返回 (ret: bool, frame: np.ndarray) — 兼容 cv2.VideoCapture 接口"""
        if not self._opened:
            return False, None
        code, data = self._client.GetImageSample()
        if code == 0:
            arr = np.frombuffer(bytes(data), dtype=np.uint8)
            return True, cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return False, None

    def isOpened(self):
        return self._opened

    def release(self):
        self._opened = False
