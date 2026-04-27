import cv2
import numpy as np
import os

def create_test_image():
    # 1. 创建一张 480(高) x 640(宽) 的纯白图片
    img = np.ones((480, 640, 3), dtype=np.uint8) * 255

    # 2. 在偏左的位置画一条纵向的粗黑线
    cv2.line(img, (200, 0), (200, 480), (0, 0, 0), 40)

    # 3. 获取路径
    current_path = os.path.dirname(os.path.abspath(__file__))
    save_path = os.path.join(current_path, "test_line.jpg")
    
    # 4. 【核心修改】使用 imencode 绕过 OpenCV 对中文路径的限制
    is_success, buffer = cv2.imencode(".jpg", img)
    if is_success:
        buffer.tofile(save_path)
        print(f"破解中文路径成功！测试图片已真正生成在: {save_path}")
    else:
        print("图片生成失败！")

if __name__ == "__main__":
    create_test_image()