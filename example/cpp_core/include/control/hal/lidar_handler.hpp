#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include <mutex>
#include <memory>
#include <unitree/robot/channel/channel_subscriber.hpp>
// 🌟 更改为订阅 PointStamped 消息类型，这是宇树雷达官方避障数据的类型
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
    
    // 🌟 更改智能指针的模板参数为 PointStamped_
    std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::PointStamped_>> sub_;
};

} // namespace control
} // namespace ruikang

#endif // LIDAR_HANDLER_HPP