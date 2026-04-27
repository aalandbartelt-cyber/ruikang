import cv2
import time

# 这正是你 config.json 里那个已经验证成功的 GStreamer 管道
pipeline = "udpsrc address=230.1.1.1 port=1720 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"

print("正在连接机器狗原生的视频流...")
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("错误：无法打开摄像头！")
else:
    # 稍微空转几帧，让摄像头曝光稳定下来
    for i in range(15):
        cap.read()
        time.sleep(0.03)
        
    # 抓取最清晰的一帧
    ret, frame = cap.read()
    if ret:
        file_name = "dog_eye_raw.jpg"
        cv2.imwrite(file_name, frame)
        print(f"✅ 抓拍成功！你狗眼里的真实画面已保存为: {file_name}")
    else:
        print("错误：抓不到画面！")

cap.release()