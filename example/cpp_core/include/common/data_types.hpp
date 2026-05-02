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

    // ★★ 深度相机数据（D435i）★★
    // 物理基线：D435i 前倾约 45°，无障碍时地面 ≈ 0.66m
    float depth_front = 9.99f;   // 中央前方
    float depth_left  = 9.99f;   // 左前方
    float depth_right = 9.99f;   // 右前方
};

} // namespace ruikang

#endif // DATA_TYPES_HPP