# -*- coding: utf-8 -*-
"""
hardware/realsense_handler.py
Intel RealSense D435i 深度处理模块（方案三 V2.0）

物理参数：
  - 安装位置：狗头上方
  - 朝向：前倾约 45° 朝下
  - 无障碍物时地面距离基线：约 0.66m

避障逻辑：
  - 距离 > 0.55m: 无障碍（可能是地面）
  - 距离 0.20~0.50m: 有障碍物（需触发转弯）
  - 距离 < 0.20m: 太近，可能是噪声或紧急情况
"""
import pyrealsense2 as rs
import numpy as np


class RealSenseHandler:
    def __init__(self, depth_width=848, depth_height=480,
                 color_width=640, color_height=480, fps=30):
        # --- 新增：强制硬件复位防卡死 ---
        import time
        ctx = rs.context()
        if len(ctx.query_devices()) > 0:
            dev = ctx.query_devices()[0]
            print(f"🔄 正在硬件重启 D435i...")
            dev.hardware_reset()
            time.sleep(2.5)  # 必须等待相机重新上线
        # --------------------------------
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 启用深度流和彩色流
        self.config.enable_stream(
            rs.stream.depth, depth_width, depth_height, rs.format.z16, fps
        )
        self.config.enable_stream(
            rs.stream.color, color_width, color_height, rs.format.bgr8, fps
        )

        # 启动 pipeline
        self.profile = self.pipeline.start(self.config)

        # 把深度对齐到彩色坐标系
        self.align = rs.align(rs.stream.color)

        # 获取深度尺度（米/单位）
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"📷 D435i 启动成功，depth_scale={self.depth_scale}")

        # 深度滤波器（内置去噪）
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole_filling = rs.hole_filling_filter()

        # 缓存最新一帧
        self._last_color = None
        self._last_depth = None

    def update(self):
        """主循环每次调用，拉一帧新数据"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
        except Exception as e:
            print(f"[RS] wait_for_frames 失败: {e}")
            return False

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()

        if not depth_frame or not color_frame:
            return False

        # 应用滤波链
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole_filling.process(depth_frame)

        self._last_color = np.asanyarray(color_frame.get_data())
        self._last_depth = np.asanyarray(depth_frame.get_data())
        return True

    def get_color(self):
        """返回当前彩色帧（用于寻迹和显示）"""
        return self._last_color

    def get_depth(self):
        """返回当前深度帧"""
        return self._last_depth

    def get_distance_in_roi(self, roi_x_start, roi_x_end,
                             roi_y_start, roi_y_end):
        """
        在指定 ROI 内计算距离（米）
        策略：取最近 5% 像素的中值，避开远处地面背景
        """
        if self._last_depth is None:
            return 9.99

        roi = self._last_depth[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        # 有效像素：0.20m ~ 5m
        valid = roi[(roi > 200) & (roi < 5000)]

        if valid.size < 50:
            return 9.99

        # 取最近 5% 像素的中值（避开"看到地面"的影响）
        n_top = max(10, valid.size // 20)
        nearest = np.partition(valid, n_top)[:n_top]
        median_mm = np.median(nearest)

        return float(median_mm) * self.depth_scale

    def get_front_distance(self):
        """中央前方距离（用于触发转弯）"""
        if self._last_depth is None:
            return 9.99
        h, w = self._last_depth.shape
        # 中央窄区，避开两侧
        return self.get_distance_in_roi(
            roi_x_start=int(w * 0.40),
            roi_x_end=int(w * 0.60),
            roi_y_start=int(h * 0.30),  # 上半部分（D435i 前倾，上半看更远）
            roi_y_end=int(h * 0.70),
        )

    def get_left_distance(self):
        """左前方距离"""
        if self._last_depth is None:
            return 9.99
        h, w = self._last_depth.shape
        return self.get_distance_in_roi(
            roi_x_start=int(w * 0.10),
            roi_x_end=int(w * 0.30),
            roi_y_start=int(h * 0.30),
            roi_y_end=int(h * 0.70),
        )

    def get_right_distance(self):
        """右前方距离"""
        if self._last_depth is None:
            return 9.99
        h, w = self._last_depth.shape
        return self.get_distance_in_roi(
            roi_x_start=int(w * 0.70),
            roi_x_end=int(w * 0.90),
            roi_y_start=int(h * 0.30),
            roi_y_end=int(h * 0.70),
        )

    def stop(self):
        try:
            self.pipeline.stop()
            print("📷 D435i 已停止")
        except Exception as e:
            print(f"[RS] 停止异常: {e}")