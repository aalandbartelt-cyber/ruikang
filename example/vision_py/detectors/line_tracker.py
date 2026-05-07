# -*- coding: utf-8 -*-
import cv2
import numpy as np

def find_line_offset(frame, threshold=60):
    """
    双切片预判寻迹：解决连续 S 弯和发夹弯
    返回: (immediate_offset, turn_trend)
    """
    # 1. 预处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 直接二值化，速度最快
    _, thresh = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    
    height, width = thresh.shape
    center_x = width // 2

    # ---------------------------------------------------------
    # 2. 内部工具函数：计算指定区域的黑线中心
    # ---------------------------------------------------------
    def get_roi_center(y_start_percent, y_end_percent):
        y_top = int(height * y_start_percent)
        y_bottom = int(height * y_end_percent)
        # 左右裁剪 10%，排除边缘干扰
        roi = thresh[y_top:y_bottom, int(width*0.1):int(width*0.9)]
        
        M = cv2.moments(roi)
        if M["m00"] > 0:
            # 返回相对于整幅图的 X 坐标
            return int(M["m10"] / M["m00"]) + int(width * 0.1)
        return None

    # ---------------------------------------------------------
    # 3. 双切片采样
    # ---------------------------------------------------------
    # 近端（脚下 85%-100% 区域）：用于维持当前平衡
    x_near = get_roi_center(0.85, 1.0)
    
    # 远端（前方 60%-75% 区域）：用于预判弯道趋势
    x_far = get_roi_center(0.60, 0.75)

    # ---------------------------------------------------------
    # 4. 逻辑计算
    # ---------------------------------------------------------
    # 当前偏差 (Offset)
    immediate_offset = (x_near - center_x) if x_near is not None else 0
    
    # 弯道趋势 (Trend)：远端中心 - 近端中心
    # 如果 Trend 为正，说明线往右撇（准备右转）；为负则左转
    turn_trend = (x_far - x_near) if (x_near is not None and x_far is not None) else 0

    # 可视化（仅供本地调试，实车运行不影响速度）
    if x_near: cv2.circle(frame, (x_near, int(height*0.92)), 7, (0, 255, 0), -1)
    if x_far: cv2.circle(frame, (x_far, int(height*0.67)), 7, (0, 0, 255), -1)

    return float(immediate_offset), float(turn_trend)