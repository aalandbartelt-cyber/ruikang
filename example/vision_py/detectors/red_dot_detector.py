import cv2
import numpy as np

def detect_red_dot(frame):
    """
    基于 5.9 测试方案的红点检测逻辑
    返回: (red_dot_detected(bool), red_dot_center_x(int), 处理后的画面)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 按照方案 7.2 要求的 HSV 阈值
    # 红色 Hin [0,10] U [160,180], S > 100, V > 100
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 简单的闭运算去噪点
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    red_dot_detected = False
    red_dot_center_x = -1
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            
            # 1. 获取外接圆及半径
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            circle_area = np.pi * (radius ** 2)
            
            # 2. 计算实心度 (轮廓面积 / 外接圆面积)
            # 如果是空心的或者不规则的，这个值会很小；完美的实心圆接近 1.0
            fill_ratio = area / circle_area if circle_area > 0 else 0
            
            # 3. 计算轮廓圆度 (边缘的平滑程度)
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            # 4. 计算长宽比
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            
            # 🚀 终极三重装甲：必须是正方形区域内 + 边缘必须够圆 + 必须是实心！
            if 0.8 < aspect_ratio < 1.2 and circularity > 0.75 and fill_ratio > 0.8:
                red_dot_detected = True
                red_dot_center_x = int(cx)
                
                # 画图预览（绿圈包围，红点准心）
                cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)
                break
            
    return red_dot_detected, red_dot_center_x, frame

# 本地笔记本摄像头直接测试
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    print("开始本地红点检测测试，拿个红色的东西到镜头前...")
    while True:
        ret, frame = cap.read()
        if not ret: break
        detected, cx, res_frame = detect_red_dot(frame)
        print(f"检测到红点: {detected}, 中心 X: {cx}")
        cv2.imshow("Red Dot Test", res_frame)
        if cv2.waitKey(1) == ord('q'): break
    cap.release()
    cv2.destroyAllWindows()