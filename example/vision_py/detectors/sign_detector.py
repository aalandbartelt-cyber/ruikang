# -*- coding: utf-8 -*-
import cv2
import numpy as np

def detect_warning_signs():
    cap = cv2.VideoCapture(0)
    # 增加提示：目标三：高优中断测试（警示牌触发）[cite: 18]
    print("👁️ FIRE 专项横杠特征检测启动！按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1) # 解决镜像问题
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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
                    # 创建不缩小的遮罩，完整保留底部特征
                    inner_mask = np.zeros_like(mask_yellow)
                    cv2.drawContours(inner_mask, [approx], 0, 255, thickness=-1)
                    
                    # 提取黑色像素 (pattern为白色，背景为黑色)
                    lower_black = np.array([0, 0, 0])
                    upper_black = np.array([180, 255, 90]) 
                    mask_black = cv2.inRange(hsv, lower_black, upper_black)
                    clean_black = cv2.bitwise_and(mask_black, inner_mask)
                    
                    # 剥离外边框，提取核心图案
                    contours_b, _ = cv2.findContours(clean_black, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    valid_contours = [cb for cb in contours_b if cv2.contourArea(cb) > 20 and 
                                     cv2.boundingRect(cb)[2] < 0.85 * w and cv2.boundingRect(cb)[3] < 0.85 * h]
                    
                    if valid_contours:
                        all_points = np.vstack(valid_contours)
                        bx, by, bw, bh = cv2.boundingRect(all_points)
                        pattern_canvas = np.zeros((bh, bw), dtype=np.uint8)
                        # 转换偏移量绘图
                        for v_cnt in valid_contours:
                            shifted_cnt = v_cnt - [bx, by]
                            cv2.drawContours(pattern_canvas, [shifted_cnt], -1, 255, thickness=-1)
                        
                        cv2.imshow("Debug ROI", pattern_canvas)

                        # ==========================================
                        # 🌟 物理特征计算开始
                        # ==========================================
                        aspect_ratio = bw / float(bh)
                        
                        # 1. FIRE 专项：检测底部 20% 区域是否有长横杠
                        bottom_zone_h = int(bh * 0.20)
                        has_base_bar = False
                        if bottom_zone_h > 0:
                            bottom_zone = pattern_canvas[bh - bottom_zone_h : bh, :]
                            # 统计每一行的白色像素数量（图案部分）
                            row_counts = np.sum(bottom_zone == 255, axis=1)
                            # 如果最长的一行占比超过 60%，判定为有横杠
                            if np.max(row_counts) > (bw * 0.60):
                                has_base_bar = True
                        
                        # 2. 重心计算
                        black_pixels = cv2.countNonZero(pattern_canvas)
                        half_h = bh // 2
                        bottom_pixels = cv2.countNonZero(pattern_canvas[half_h:bh, :])
                        bot_ratio = bottom_pixels / float(black_pixels) if black_pixels > 0 else 0
                        
                        sign_type = "UNKNOWN"
                        
                        # --- 判决逻辑 ---
                        if has_base_bar:
                            # 🌟 只要有横杠，强行判定为 FIRE
                            sign_type = "FIRE"
                        elif aspect_ratio < 0.70:
                            # 没横杠且瘦长 -> 闪电
                            sign_type = "ELECTRIC"
                        elif bot_ratio < 0.45:
                            # 没横杠且头重脚轻 -> 辐射
                            sign_type = "RADIATION"
                        else:
                            # 其他情况模糊匹配
                            sign_type = "FIRE" if bot_ratio > 0.52 else "RADIATION"

                        # ==========================================

                        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
                        # 实时显示 Bar 检测状态和重心数据
                        text = f"{sign_type} (Bar:{'Y' if has_base_bar else 'N'}, Bot:{bot_ratio:.2f})"
                        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        print(f"W/H:{aspect_ratio:.2f} | Bar:{'YES' if has_base_bar else 'NO '} | Bot:{bot_ratio:.2f} ==> {sign_type}")

        # 红色圆形检测 (保持不变) [cite: 80]
        lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
        mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        contours_r, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours_r:
            if cv2.contourArea(cnt) > 1500:
                (cx, cy), radius = cv2.minEnclosingCircle(cnt)
                if cv2.contourArea(cnt) / (np.pi * (radius ** 2)) > 0.75:
                    cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 0, 255), 3)
                    cv2.putText(frame, "C-MARK", (int(cx) - 40, int(cy) - int(radius) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Final Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_warning_signs()