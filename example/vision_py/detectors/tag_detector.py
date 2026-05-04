# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np

# =====================================================
# 🔧 OpenCV 版本兼容层
# =====================================================
_USE_NEW_API = hasattr(aruco, 'ArucoDetector')
print(f"[tag_detector] OpenCV {cv2.__version__}, "
      f"使用 {'新' if _USE_NEW_API else '老'} ArUco API")

if _USE_NEW_API:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    # 注入高灵敏度参数
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    _detector = aruco.ArucoDetector(aruco_dict, parameters)
    
    def _detect(gray):
        corners, ids, _ = _detector.detectMarkers(gray)
        return corners, ids
else:
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshWinSizeMin = 3
    
    def _detect(gray):
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        return corners, ids

def process_tag(frame):
    # --- A. 图像标准化 ---
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=1.2, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl, a, b))
    frame_enhanced = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    
    gray = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame_enhanced, cv2.COLOR_BGR2HSV)
    tag_result = "NONE"
    cx, cy = -1, -1 

    # --- B. ArUco 码检测 (第一优先级) ---
    corners, ids = _detect(gray)
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in [0, 1, 2, 3, 16, 17, 37]: 
                tag_result = f"ARUCO_{marker_id}"
                corner = corners[i][0] 
                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

    # --- C. 红色 C 标志检测 (第二优先级) ---
    lower_red1, upper_red1 = np.array([0, 110, 50]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([160, 110, 50]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                              cv2.inRange(hsv, lower_red2, upper_red2))
    
    kernel = np.ones((5,5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours_r:
        area = cv2.contourArea(cnt)
        if area > 1000:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            # 👇 就是这里！先计算外接圆，再算 rect_ratio，最后才做 if 判断
            (circle_x, circle_y), radius = cv2.minEnclosingCircle(cnt)
            rect_ratio = area / (np.pi * (radius ** 2)) if radius > 0 else 0
            
            if circularity > 0.4 or rect_ratio > 0.5:
                # 如果没认出 ArUco，才采用圆圈的坐标
                if tag_result == "NONE":
                    cx, cy = int(circle_x), int(circle_y)

                r_crop = int(radius * 0.7)
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
                    ink_ratio = ink_pixels / float(total_circle_pixels) if total_circle_pixels > 0 else 0
                    
                    if ink_ratio > 0.42:
                        if tag_result == "NONE": tag_result = "C_MARK_WHITE_OUTER"
                        color = (0, 255, 0)
                    else:
                        if tag_result == "NONE": tag_result = "C_MARK_BLACK_OUTER"
                        color = (0, 0, 255)
                        
                    cv2.circle(frame, (int(circle_x), int(circle_y)), int(radius), color, 3)

    return tag_result, cx, cy, frame

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret: break
        res, cx, cy, frame = process_tag(frame)
        # （可选）你可以顺便加个 print，看看它算出来的中心点对不对：
        print(f"识别结果: {res}, 中心坐标: ({cx}, {cy})")
        cv2.imshow("Tag Robust System", frame)
        if cv2.waitKey(1) == ord('q'): break
    cap.release()
    cv2.destroyAllWindows()