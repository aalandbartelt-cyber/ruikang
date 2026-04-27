import cv2

def test_pipeline(i):
    # 使用最宽松的 decodebin 自动解码管道
    pipeline = f"v4l2src device=/dev/video{i} ! decodebin ! videoconvert ! appsink drop=1"
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        return False
    
    ret, frame = cap.read()
    if ret:
        cv2.imwrite(f"test_result_{i}.jpg", frame)
        cap.release()
        return True
    
    cap.release()
    return False

print("🔍 正在扫描可用摄像头节点，请稍候...")
for i in [0, 2, 4]:  # 通常偶数是视频流
    print(f"尝试节点 /dev/video{i} ...")
    if test_pipeline(i):
        print(f"🎯 找到了！节点 /dev/video{i} 可以正常工作！")
        print(f"✅ 已保存预览图到: test_result_{i}.jpg")
        break
else:
    print("❌ 扫描完毕，所有节点均无法通过 GStreamer 打开。")
    print("提示：尝试直接打开数字索引...")
    for i in [0, 1, 2]:
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"🎯 找到了！数字索引 {i} 可以直接打开！")
            cap.release()
            break