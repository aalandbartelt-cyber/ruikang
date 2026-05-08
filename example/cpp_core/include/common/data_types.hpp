#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <string>

namespace ruikang {

// 视觉与雷达传过来的多模态数据
struct VisionData {
    float line_offset = 0.0f;
    float obstacle_distance = 9.99f;
    int platform_tag = 0;
    std::string warning_sign = "NONE";
    bool aruco_detected = false;
    float aruco_center_x = -1.0f;  // ArUco 码中心 x（像素坐标），-1 表示未检测到
    float aruco_center_y = -1.0f;  // ArUco 码中心 y（像素坐标），-1 表示未检测到

    // ★★ 深度相机数据（D435i）★★
    // 物理基线：D435i 前倾约 45°，无障碍时地面 ≈ 0.66m
    float depth_front = 9.99f;   // 中央前方
    float depth_left  = 9.99f;   // 左前方
    float depth_right = 9.99f;   // 右前方

    // Y 视觉：双切片弯道预判（近切片 vs 远切片 ΔX）
    // >0 = 前方线偏右（预示右弯），<0 = 前方线偏左（预示左弯）
    float turn_trend = 0.0f;

    // 红点检测（State07 检测平台精确定位）
    bool  red_dot_detected  = false;
    float red_dot_center_x  = -1.0f;  // 红点中心 x（像素坐标），-1 表示未检测到
};

} // namespace ruikang

#endif // DATA_TYPES_HPP