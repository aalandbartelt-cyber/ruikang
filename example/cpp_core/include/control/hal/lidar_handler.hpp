#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include <mutex>
#include <memory>
#include <deque> // 🌟 引入双端队列，用于滤波
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointStamped_.hpp>

namespace ruikang {
namespace control {

class LidarHandler {
public:
    LidarHandler();
    ~LidarHandler();
    
    void init();
    float get_front_distance();

private:
    void LidarCallback(const void* message);

    std::mutex data_mutex_;
    float front_min_dist_;
    
    // 🌟 滤波专用缓冲池
    std::deque<float> dist_buffer_; 

    std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>> sub_;
};

} // namespace control
} // namespace ruikang

#endif // LIDAR_HANDLER_HPP