#include "control/hal/lidar_handler.hpp"
#include <iostream>

namespace ruikang {
namespace control {

LidarHandler::LidarHandler() : front_min_dist_(9.99f) {}

LidarHandler::~LidarHandler() {
    if (sub_) {
        sub_->CloseChannel();
    }
}

void LidarHandler::init() {
    // 监听官方预处理好的避障距离话题
    sub_.reset(new unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>("rt/utlidar/range_info"));
    sub_->InitChannel(std::bind(&LidarHandler::LidarCallback, this, std::placeholders::_1), 1);
    std::cout << "[LidarHandler] HAL层雷达驱动已启动, 坐标系偏移补偿(0.4m)已激活!" << std::endl;
}

float LidarHandler::get_front_distance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return front_min_dist_;
}

void LidarHandler::LidarCallback(const void* message) {
    auto* msg = static_cast<const geometry_msgs::msg::dds_::PointStamped_*>(message);
    if (!msg) return;

    // 1. 获取官方原始距离（以雷达/身体中心为原点）
    float raw_dist = msg->point().x();
    float calibrated_dist = 9.99f;

    // 2. 🌟 坐标系偏移量补偿逻辑 (Sensor Offset Calibration)
    // 如果测到的距离是正常的有效范围（假设 > 9.0m 都是空气）
    if (raw_dist < 9.0f) {
        // 减去狗身体前半截的物理长度 (0.35m)
        calibrated_dist = raw_dist - 0.35f; 
        
        // 防呆设计：如果贴得比 0.4m 还紧（比如挤压到了雷达），防止出现负数距离
        if (calibrated_dist < 0.0f) {
            calibrated_dist = 0.0f;
        }
    }

    // 3. 更新给状态机的最终数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    front_min_dist_ = calibrated_dist;

    // [调试输出]：方便你对比原始数据和校准后的数据
    /*
    static int print_count = 0;
    if (print_count++ % 50 == 0) {
        std::cout << "[Lidar] 原始中心距离: " << raw_dist 
                  << "m | 校准后(距鼻尖): " << front_min_dist_ << "m" << std::endl;
    }
    */
}

} // namespace control
} // namespace ruikang