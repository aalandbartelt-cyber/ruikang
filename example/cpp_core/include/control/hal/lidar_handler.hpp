#ifndef LIDAR_HANDLER_HPP
#define LIDAR_HANDLER_HPP

#include <mutex>
#include <memory>
#include <vector>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

namespace ruikang {
namespace control {

// 调试用：剖面统计结果
struct SliceInfo {
    float x_center;
    int   wall_count;
    int   total_count;
};

// ★ 在类外用 using 把模板参数完全展开，避免嵌套模板 >> 解析问题
using PointCloud2Msg = ::sensor_msgs::msg::dds_::PointCloud2_;
using PointCloudSub  = unitree::robot::ChannelSubscriber<PointCloud2Msg>;

class LidarHandler {
public:
    LidarHandler();
    ~LidarHandler();

    // 必须在 ChannelFactory::Init() 之后调用
    void init();

    // ===== State03 主接口 =====
    float get_front_wall_distance();   // 前方挡板距离（米）
    float get_left_side_distance();    // 左侧最近距离（米）
    float get_right_side_distance();   // 右侧最近距离（米）

    // ===== 调试接口 =====
    std::vector<SliceInfo> get_debug_slices();

    // ===== 兼容旧代码 =====
    float get_front_distance() { return get_front_wall_distance(); }

private:
    void cloudCallback(const void* message);

    mutable std::mutex data_mutex_;
    float front_wall_dist_;
    float left_dist_;
    float right_dist_;
    std::vector<SliceInfo> debug_slices_;

    std::shared_ptr<PointCloudSub> sub_;
};

} // namespace control
} // namespace ruikang

#endif // LIDAR_HANDLER_HPP