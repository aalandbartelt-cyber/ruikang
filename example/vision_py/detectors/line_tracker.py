import cv2
import numpy as np
import os

def find_line_offset(frame, threshold=60):
    """
    输入：一帧图像 (BGR)，以及二值化的光线阈值
    输出：黑线中心偏离图像中心的像素差（负数偏左，正数偏右）
    如果没找到线，返回 None
    """
    # 1. 预处理：转为灰度图并模糊化以减少噪声
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 2. 二值化：将黑线从背景中提取出来
    _, thresh = cv2.threshold(blurred, threshold, 255, cv2.THRESH_BINARY_INV)

    # ==========================================
    # 3. 终极 ROI 裁剪（切掉上面，切掉左边，切掉右边）
    # ==========================================
    height, width = thresh.shape
    
    # 上下裁剪：只留底部 30% 的区域 (从 0.7 到 1.0)
    roi_top = int(height * 0.7)
    roi_bottom = height
    
    # 左右裁剪：切掉两边各 20% 的画面，防止拍到狗腿
    roi_left = int(width * 0.2)
    roi_right = int(width * 0.8)
    
    # 截取这块干干净净的“猫眼”区域
    roi = thresh[roi_top:roi_bottom, roi_left:roi_right]

    # 4. 计算重心 (Moments)
    M = cv2.moments(roi)
    
    if M["m00"] != 0:
        # 计算黑线在 ROI 中的横坐标中心
        line_x_in_roi = int(M["m10"] / M["m00"])
        
        # ⚠️ 关键：把 ROI 里的相对坐标，还原成整张大图上的真实坐标
        line_x = roi_left + line_x_in_roi 
        
        # 计算图像的横坐标中心
        center_x = width // 2
        
        # 计算偏差：负数表示线在相机左边，正数表示在右边
        offset = line_x - center_x
        
        # --- 调试用：在原图上画出识别结果 ---
        # 画出 ROI 的边框（绿框），方便你肉眼确认切得对不对
        cv2.rectangle(frame, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 0), 2)
        # 画出计算出的重心红点
        cv2.circle(frame, (line_x, roi_top + int(M["m01"] / M["m00"])), 5, (0, 0, 255), -1)
        # 画出中心蓝线
        cv2.line(frame, (center_x, 0), (center_x, height), (255, 0, 0), 1) 
        
        return float(offset)
    
    return None

# 下面的 __main__ 调试代码保持你的原样不动即可...
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    img_path = os.path.join(current_dir, 'test_line.jpg')
    print("开始本地测试 line_tracker...")
    print(f"正在读取路径: {img_path}")
    img = cv2.imread(img_path) 
    if img is not None:
        offset = find_line_offset(img, threshold=60)
        print(f"✅ 检测成功！黑线偏移量为: {offset}")
        cv2.imshow("Detection Result", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("错误：图片读取失败，请检查上面的路径对不对！")