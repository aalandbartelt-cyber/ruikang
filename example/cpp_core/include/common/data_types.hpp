#ifndef DATA_TYPES_HPP
#define DATA_TYPES_HPP

#include <string>

namespace ruikang {

// 视觉与雷达传过来的多模态数据
struct VisionData {
    float line_offset = 0.0f;
    float obstacle_distance = 9.99f; // 🌟 确保有这个字段！默认给个安全距离 9.99米
    int platform_tag = 0;
    std::string warning_sign = "NONE";
    bool aruco_detected = false;
};

} // namespace ruikang

#endif // DATA_TYPES_HPP