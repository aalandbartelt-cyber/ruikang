# -*- coding: utf-8 -*-
import cv2
import numpy as np

def process_sign(frame):
    """
    接收一帧画面，返回识别到的警示牌类型和标注后的画面
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    sign_type = "NONE"

    # 1. 锁定黄色的三角形区域
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours_y, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours_y:
        if cv2.contourArea(cnt) > 2000:
            epsilon = 0.04 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            if len(approx) == 3:
                x, y, w, h = cv2.boundingRect(approx)
                inner_mask = np.zeros_like(mask_yellow)
                cv2.drawContours(inner_mask, [approx], 0, 255, thickness=-1)
                
                lower_black = np.array([0, 0, 0])
                upper_black = np.array([180, 255, 90]) 
                mask_black = cv2.inRange(hsv, lower_black, upper_black)
                clean_black = cv2.bitwise_and(mask_black, inner_mask)
                
                contours_b, _ = cv2.findContours(clean_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                valid_contours = [cb for cb in contours_b if cv2.contourArea(cb) > 20 and 
                                 cv2.boundingRect(cb)[2] < 0.85 * w and cv2.boundingRect(cb)[3] < 0.85 * h]
                
                if valid_contours:
                    all_points = np.vstack(valid_contours)
                    bx, by, bw, bh = cv2.boundingRect(all_points)
                    pattern_canvas = np.zeros((bh, bw), dtype=np.uint8)
                    for v_cnt in valid_contours:
                        shifted_cnt = v_cnt - [bx, by]
                        cv2.drawContours(pattern_canvas, [shifted_cnt], -1, 255, thickness=-1)

                    aspect_ratio = bw / float(bh)
                    
                    bottom_zone_h = int(bh * 0.20)
                    has_base_bar = False
                    if bottom_zone_h > 0:
                        bottom_zone = pattern_canvas[bh - bottom_zone_h : bh, :]
                        row_counts = np.sum(bottom_zone == 255, axis=1)
                        if np.max(row_counts) > (bw * 0.60):
                            has_base_bar = True
                    
                    black_pixels = cv2.countNonZero(pattern_canvas)
                    half_h = bh // 2
                    bottom_pixels = cv2.countNonZero(pattern_canvas[half_h:bh, :])
                    bot_ratio = bottom_pixels / float(black_pixels) if black_pixels > 0 else 0
                    
                    # 判定逻辑
                    if has_base_bar:
                        sign_type = "FIRE"
                    elif aspect_ratio < 0.70:
                        sign_type = "ELECTRIC"
                    elif bot_ratio < 0.45:
                        sign_type = "RADIATION"
                    else:
                        sign_type = "FIRE" if bot_ratio > 0.52 else "RADIATION"

                    # 绘制画框
                    cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                    text = f"SIGN: {sign_type}"
                    cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
    return sign_type, frame

# 这里的代码只在单独运行这个脚本时执行，方便你以后单体调试
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.flip(frame, 1)
        res, frame = process_sign(frame)
        cv2.imshow("Sign Test", frame)
        if cv2.waitKey(1) == ord('q'): break