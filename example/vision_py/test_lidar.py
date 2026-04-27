# test_lidar.py 简单测试脚本
from hardware.lidar_node import LidarReader
import time

try:
    lidar = LidarReader(port='/dev/ttyUSB0')
    while True:
        dist = lidar.get_obstacle_distance()
        print(f"雷达前方距离: {dist} m")
        time.sleep(0.1)
except Exception as e:
    print(f"报错啦: {e}")