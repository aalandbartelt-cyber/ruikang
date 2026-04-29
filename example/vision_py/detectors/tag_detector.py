# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np

# 🌟 初始化只需做一次，放在外面可以大幅提升帧率！
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

def process_tag(frame):
    """
    接收一帧画面，返回识别到的标签/站点ID，以及标注后的画面
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    tag_result = "NONE"

    # 1. ArUco 标签检测
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for marker_id in ids.flatten():
            # 你可以随时在这里加上 3 号
            if marker_id in [1, 2, 3]: 
                tag_result = f"ARUCO_{marker_id}"

    # 2. 红色 C 标志检测
    lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                              cv2.inRange(hsv, lower_red2, upper_red2))
    
    contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_r:
        area = cv2.contourArea(cnt)
        if area > 1500:
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if area / (np.pi * (radius ** 2)) > 0.70:
                r_crop = int(radius * 0.75)
                y1, y2 = int(cy - r_crop), int(cy + r_crop)
                x1, x2 = int(cx - r_crop), int(cx + r_crop)
                if y1<0 or y2>frame.shape[0] or x1<0 or x2>frame.shape[1]: continue
                
                roi_gray = gray[y1:y2, x1:x2]
                _, roi_bin = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                
                if roi_bin.size > 0:
                    h_roi, w_roi = roi_bin.shape
                    circle_mask = np.zeros_like(roi_bin)
                    cv2.circle(circle_mask, (w_roi//2, h_roi//2), int(w_roi/2), 255, -1)
                    clean_pattern = cv2.bitwise_and(roi_bin, roi_bin, mask=circle_mask)
                    
                    ink_pixels = cv2.countNonZero(clean_pattern)
                    total_circle_pixels = np.pi * ((w_roi/2)**2)
                    if total_circle_pixels == 0: continue
                    ink_ratio = ink_pixels / float(total_circle_pixels)
                    
                    # 墨量判定
                    if ink_ratio > 0.40:
                        tag_result = "C_MARK_WHITE_OUTER"
                        color = (0, 255, 0)
                    else:
                        tag_result = "C_MARK_BLACK_OUTER"
                        color = (0, 0, 255)
                        
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), color, 3)
                    cv2.putText(frame, tag_result, (int(cx)-100, int(cy)-int(radius)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    return tag_result, frame

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret: break
        frame = cv2.flip(frame, 1)
        res, frame = process_tag(frame)
        cv2.imshow("Tag Test", frame)
        if cv2.waitKey(1) == ord('q'): break