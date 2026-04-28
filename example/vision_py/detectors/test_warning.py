# -*- coding: utf-8 -*-
import cv2
import numpy as np

def detect_warning_signs():
    # 打开默认摄像头
    cap = cv2.VideoCapture(0)
    print("👁️ 警示牌识别测试已启动！请将标志图片对准摄像头。按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 将图像转换为 HSV 颜色空间（对光照变化更鲁棒）
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ==========================================
        # 1. 提取黄色三角形 (触电、辐射、火灾)
        # ==========================================
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 寻找黄色区域的轮廓
        contours_y, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours_y:
            # 过滤掉太小的面积（防干扰）
            if cv2.contourArea(cnt) > 1500:
                # 轮廓近似，判断多边形顶点数
                epsilon = 0.04 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # 如果是3个顶点，大概率是三角形
                if len(approx) == 3:
                    cv2.drawContours(frame, [approx], 0, (0, 255, 255), 3)
                    cv2.putText(frame, "YELLOW TRIANGLE", (approx[0][0][0], approx[0][0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ==========================================
        # 2. 提取红色圆形 (C字样标志)
        # ==========================================
        # 红色在 HSV 中跨越了 0 和 180 两个区域，需要合并两个掩膜
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # 寻找红色区域的轮廓
        contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours_r:
            area = cv2.contourArea(cnt)
            # 过滤掉太小的红色背景干扰物
            if area > 1500:
                # 使用最小包围圆来判断圆滑度
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                circle_area = np.pi * (radius ** 2)
                
                # 如果轮廓面积和最小包围圆面积非常接近（比例 > 0.75），那就是圆形
                if area / circle_area > 0.75:
                    center = (int(x), int(y))
                    cv2.circle(frame, center, int(radius), (0, 0, 255), 3)
                    cv2.putText(frame, "RED CIRCLE", (center[0] - 50, center[1] - int(radius) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 显示画面
        cv2.imshow("Warning Sign Detector", frame)
        # 如果需要调试颜色过滤效果，可以把下面两行取消注释
        # cv2.imshow("Yellow Mask", mask_yellow)
        # cv2.imshow("Red Mask", mask_red)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_warning_signs()