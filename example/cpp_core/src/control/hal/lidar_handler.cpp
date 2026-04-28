#include "control/hal/lidar_handler.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

namespace ruikang {
namespace control {

LidarHandler::LidarHandler() : front_min_dist_(9.99f) {}

LidarHandler::~LidarHandler() {
    if (sub_) {
        sub_->CloseChannel();
    }
}

void LidarHandler::init() {
    sub_.reset(new unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>("rt/utlidar/range_info"));
    sub_->InitChannel(std::bind(&LidarHandler::LidarCallback, this, std::placeholders::_1), 1);
    
    // 🌟 醒目的启动日志
    std::cout << "[LidarHandler] HAL雷达启动! 0.35m 补偿 + 15帧中值滤波(防误报抗干扰)已激活!" << std::endl;
}

float LidarHandler::get_front_distance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return front_min_dist_;
}

void LidarHandler::LidarCallback(const void* message) {
    auto* msg = static_cast<const geometry_msgs::msg::dds_::PointStamped_*>(message);
    if (!msg) return;

    // 1. 获取官方原始距离
    float raw_dist = msg->point().x();
    float calibrated_dist = 9.99f;

    // 2. 物理补偿 (狗头 0.35m)
    if (raw_dist < 9.0f) {
        calibrated_dist = raw_dist - 0.35f; 
        if (calibrated_dist < 0.0f) calibrated_dist = 0.0f;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // 🌟 3. 中值滤波核心逻辑 (Median Filter) 🌟
    // 将最新数据放入池子，维持最大容量为 15 帧 (大概 0.15 秒的时间窗口)
    dist_buffer_.push_back(calibrated_dist);
    if (dist_buffer_.size() > 15) {
        dist_buffer_.pop_front();
    }

    // 拷贝一份用于排序，找出最中间的那个“稳定值”
    std::vector<float> sorted_buffer(dist_buffer_.begin(), dist_buffer_.end());
    std::sort(sorted_buffer.begin(), sorted_buffer.end());

    if (!sorted_buffer.empty()) {
        front_min_dist_ = sorted_buffer[sorted_buffer.size() / 2];
    } else {
        front_min_dist_ = calibrated_dist;
    }
}

} // namespace control
} // namespace ruikang