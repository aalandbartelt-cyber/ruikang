# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np

def detect_marks():
    cap = cv2.VideoCapture(0)
    
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    print("🚀 【全局占比 + 距离免疫版】启动！名称已互换。按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. ArUco 标签检测
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for marker_id in ids.flatten():
                if marker_id in [1, 2]:
                    print(f"📡 识别到 ArUco ID: {marker_id}")

        # ==========================================
        # 🔴 2. 红色 C 标志检测 (抗距离缩放面积法)
        # ==========================================
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
                        
                        # ==========================================
                        # 🌟 仅仅互换了这里的名称和颜色，其他一行没动！
                        # ==========================================
                        if ink_ratio > 0.40:
                            c_name = "WHITE-OUTER (白圈在外)"
                            color = (0, 255, 0) # 绿色
                        else:
                            c_name = "BLACK-OUTER (黑圈在外)"
                            color = (0, 0, 255) # 红色
                            
                        cv2.circle(frame, (int(cx), int(cy)), int(radius), color, 3)
                        info = f"{c_name} (Ink:{ink_ratio:.2f})"
                        cv2.putText(frame, info, (int(cx)-100, int(cy)-int(radius)-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        cv2.imshow("Core BIN (Distance Immune)", clean_pattern)

        cv2.imshow("Vision Core", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_marks()