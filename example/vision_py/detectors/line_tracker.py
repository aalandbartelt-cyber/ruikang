# -*- coding: utf-8 -*-
import cv2
import numpy as np

_llog_cnt = 0  # 调试日志计数器

def find_line_offset(frame, threshold=60):
    """
    三切片预判寻迹 + 直角弯检测（5.12版）
    返回: (immediate_offset, turn_trend, is_sharp_turn)
    """
    # 1. 预处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)

    height, width = thresh.shape
    center_x = width // 2

    # ---------------------------------------------------------
    # 2. 内部工具函数：计算指定区域的黑线质心 X 坐标
    # ---------------------------------------------------------
    def get_roi_center(y_start_percent, y_end_percent, crop_ratio=0.10):
        y_top = int(height * y_start_percent)
        y_bottom = int(height * y_end_percent)
        crop = int(width * crop_ratio)
        roi = thresh[y_top:y_bottom, crop:width - crop]

        M = cv2.moments(roi)
        if M["m00"] > 0:
            return int(M["m10"] / M["m00"]) + crop
        return None

    # ---------------------------------------------------------
    # 3. 三切片采样
    # ---------------------------------------------------------
    # 近端（脚下 80%-100%）：用于维持当前平衡
    x_near = get_roi_center(0.80, 1.0)
    # 中段（50%-75%）：用于弯道趋势
    x_mid  = get_roi_center(0.50, 0.75)
    # 远端（20%-45%）：用于预判直角弯（与近段中段同一裁剪，排除边缘噪点）
    x_far  = get_roi_center(0.20, 0.45)

    # ---------------------------------------------------------
    # 4. 常规巡线输出
    # ---------------------------------------------------------
    immediate_offset = (x_near - center_x) if x_near is not None else 0
    turn_trend = (x_mid - x_near) if (x_near is not None and x_mid is not None) else 0

    # ---------------------------------------------------------
    # 5. ★ 直角弯检测：仅右偏方向，近→远跳变 > 150px
    # ---------------------------------------------------------
    is_sharp_turn = False
    if x_near is not None and x_far is not None:
        dx = x_far - x_near          # >0=偏右(右转)  <0=偏左(左转)
        if dx > 150 and x_far > 500: # 右偏跳变大 + 远端线在画面右侧
            is_sharp_turn = True
    elif x_far is None and x_mid is not None:
        if x_mid - center_x > 120:   # 仅右偏丢线触发
            is_sharp_turn = True

    # 调试日志（每10帧打印x值，方便观察实际数据）
    global _llog_cnt
    _llog_cnt += 1
    if _llog_cnt % 10 == 0:
        print(f"[TRACK] near={x_near} mid={x_mid} far={x_far} sharp={is_sharp_turn}")

    # 可视化（仅供本地调试）
    if x_near: cv2.circle(frame, (x_near, int(height*0.90)), 7, (0, 255, 0), -1)
    if x_mid:  cv2.circle(frame, (x_mid,  int(height*0.62)), 7, (255, 255, 0), -1)
    if x_far:  cv2.circle(frame, (x_far,  int(height*0.32)), 7, (0, 0, 255), -1)

    return float(immediate_offset), float(turn_trend), bool(is_sharp_turn)
