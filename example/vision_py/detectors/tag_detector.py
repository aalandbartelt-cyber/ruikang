# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np

# =====================================================
# 🔧 OpenCV 版本兼容层
# - OpenCV 4.7+：新 API（ArucoDetector 类）
# - OpenCV 4.5/4.6：老 API（DetectorParameters_create + detectMarkers）
# =====================================================

# 用 ArucoDetector 是否存在判断 API 风格
_USE_NEW_API = hasattr(aruco, 'ArucoDetector')
print(f"[tag_detector] OpenCV {cv2.__version__}, "
      f"使用 {'新' if _USE_NEW_API else '老'} ArUco API")

if _USE_NEW_API:
    # ===== 新 API（4.7+）=====
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    _detector = aruco.ArucoDetector(aruco_dict, parameters)
    
    def _detect(gray):
        corners, ids, _ = _detector.detectMarkers(gray)
        return corners, ids
else:
    # ===== 老 API（4.5/4.6）=====
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    
    def _detect(gray):
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        return corners, ids


def process_tag(frame):
    # --- A. 图像标准化 (对抗光线) ---
    # 使用自适应均衡化，把太亮或太暗的地方拉回到正常水平
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    frame_enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    
    gray = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2HSV)
    tag_result = "NONE"

    # --- B. ArUco 码检测（兼容老/新 API）---
    corners, ids = _detect(gray)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for marker_id in ids.flatten():
            # 👇 就是下面这一行！让它把看到的任何 ID 都强行打印在终端里！
            print(f"🔥 发现隐藏 ArUco 码！它的真实 ID 是: {marker_id}")
            
            if marker_id in [1, 2, 3, 16, 17, 30]: 
                tag_result = f"ARUCO_{marker_id}"

    # --- C. 红色 C 标志检测 (抗干扰重构) ---
    lower_red1, upper_red1 = np.array([0, 110, 50]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([160, 110, 50]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                              cv2.inRange(hsv, lower_red2, upper_red2))
    
    # 【核心加固】闭运算：把摄像头噪点造成的红圈裂缝"焊"上
    kernel = np.ones((5,5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    
    contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours_r:
        area = cv2.contourArea(cnt)
        if area > 1000:
            # 计算圆度：4*pi*A/P^2
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            # 【降压判定】只要圆度大于 0.4 且外接圆比例达标，就认为是标志
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            rect_ratio = area / (np.pi * (radius ** 2))
            
            if circularity > 0.4 or rect_ratio > 0.5:
                r_crop = int(radius * 0.7)
                y1, y2 = int(cy - r_crop), int(cy + r_crop)
                x1, x2 = int(cx - r_crop), int(cx + r_crop)
                
                if y1<0 or y2>frame.shape[0] or x1<0 or x2>frame.shape[1]: continue
                
                roi_gray = gray[y1:y2, x1:x2]
                # 使用大津法自动找二值化阈值，无视光线
                _, roi_bin = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                
                if roi_bin.size > 0:
                    h_roi, w_roi = roi_bin.shape
                    circle_mask = np.zeros_like(roi_bin)
                    cv2.circle(circle_mask, (w_roi//2, h_roi//2), int(w_roi/2), 255, -1)
                    clean_pattern = cv2.bitwise_and(roi_bin, roi_bin, mask=circle_mask)
                    
                    ink_pixels = cv2.countNonZero(clean_pattern)
                    total_circle_pixels = np.pi * ((w_roi/2)**2)
                    ink_ratio = ink_pixels / float(total_circle_pixels) if total_circle_pixels > 0 else 0
                    
                    # 墨量判定：保持你原本的逻辑
                    if ink_ratio > 0.42: # 略微调高一点点，防止黑圈误判为白圈
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
        res, frame = process_tag(frame)
        cv2.imshow("Tag Robust System", frame)
        if cv2.waitKey(1) == ord('q'): break
    cap.release()
    cv2.destroyAllWindows()