# -*- coding: utf-8 -*-
import cv2
import cv2.aruco as aruco
import numpy as np

def main():
    # 1. 初始化摄像头 (本地测用0，实车根据config改)
    cap = cv2.VideoCapture(0) 
    
    # 2. 锁定我们在扫描器里试出来的【真命字典】
    # 既然暴力扫描器说是 4X4_50，我们就死守这个
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    
    # 调高灵敏度：减小自适应阈值的窗口，捕捉更细微的边缘
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    print("🛰️ 纯净版 ArUco 识别器启动... 按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 3. 极简预处理：只转灰度，不搞那些乱七八糟的对比度增强
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 4. 识别
        corners, ids, rejected = detector.detectMarkers(gray)

        # 5. 可视化
        if ids is not None:
            # 只要认出来，就画出绿框和 ID
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                c = corners[i][0]
                center = (int(c[:,0].mean()), int(c[:,1].mean()))
                cv2.putText(frame, f"ID: {ids[i][0]}", (center[0], center[1]-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                print(f"🎯 捕捉到 ID: {ids[i][0]} | 坐标: {center}")
        else:
            # 如果没认出来，可以在屏幕上显示一个红色的提示
            cv2.putText(frame, "Searching...", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("Pure ArUco Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()