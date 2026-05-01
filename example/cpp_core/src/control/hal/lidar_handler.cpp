// src/control/hal/lidar_handler.cpp
// V5.0：基于真实通道场景的最终雷达处理器
//
// 设计依据（RViz 实测）：
//   - 狗自身边缘：X 到达 -0.39，Y 到达 +0.28
//   - 狗背高度：Z=+0.06（雷达坐标系）
//   - 障碍物高度：Z=+0.11
//   - 通道侧墙：会出现在 Y=-1.06 等位置，必须严格 Y 过滤
//
#include "control/hal/lidar_handler.hpp"
#include "common/config.hpp"
#include <cstring>
#include <cmath>
#include <iostream>
#include <algorithm>

namespace ruikang {
namespace control {

LidarHandler::LidarHandler()
    : front_wall_dist_(9.99f),
      left_dist_(9.99f),
      right_dist_(9.99f) {}

LidarHandler::~LidarHandler() {
    if (sub_) sub_->CloseChannel();
}

void LidarHandler::init() {
    const std::string TOPIC = "rt/utlidar/cloud";

    sub_ = std::make_shared<PointCloudSub>(TOPIC);
    sub_->InitChannel(
        std::bind(&LidarHandler::cloudCallback, this, std::placeholders::_1),
        1
    );

    std::cout << "================================================" << std::endl;
    std::cout << "[Lidar] PointCloud2 订阅启动 (V5.0)" << std::endl;
    std::cout << "  话题: " << TOPIC << std::endl;
    std::cout << "  挡板 Z: [" << config::s03::WALL_Z_MIN
              << ", " << config::s03::WALL_Z_MAX << "] m" << std::endl;
    std::cout << "  Y 窄条: ±" << config::s03::WALL_Y_HALF << " m" << std::endl;
    std::cout << "  挡板点数阈值: " << config::s03::WALL_POINT_THRESH << std::endl;
    std::cout << "================================================" << std::endl;
}

void LidarHandler::cloudCallback(const void* message) {
    const PointCloud2Msg* msg = reinterpret_cast<const PointCloud2Msg*>(message);
    if (!msg) return;

    static bool first_frame = true;
    if (first_frame) {
        first_frame = false;
        std::cout << "[Lidar] ✅ 第一帧点云到达！"
                  << " point_step=" << msg->point_step()
                  << " width="      << msg->width()
                  << " height="     << msg->height() << std::endl;
    }

    const uint8_t* data = msg->data().data();
    uint32_t point_step = msg->point_step();
    uint32_t total_pts  = msg->height() * msg->width();

    if (point_step < 12 || total_pts == 0) return;

    // ===== 前方剖面扫描参数 =====
    constexpr float SCAN_X_MIN = 0.55f;   // 远离狗鼻子近场噪声
    constexpr float SCAN_X_MAX = 1.50f;
    constexpr float SLICE_STEP = 0.05f;   // 5cm 一个剖面，更精细
    int num_slices = static_cast<int>((SCAN_X_MAX - SCAN_X_MIN) / SLICE_STEP) + 2;

    std::vector<int> wall_counts(num_slices, 0);
    std::vector<int> total_counts(num_slices, 0);

    float left_min  = 9.99f;
    float right_min = 9.99f;

    // ===== 遍历点云 =====
    for (uint32_t i = 0; i < total_pts; ++i) {
        const uint8_t* p = data + i * point_step;
        float x, y, z;
        std::memcpy(&x, p + 0, sizeof(float));
        std::memcpy(&y, p + 4, sizeof(float));
        std::memcpy(&z, p + 8, sizeof(float));

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        // ★ 1. 排除狗自身机身投影（基于实测扩大范围）★
        constexpr float DOG_BODY_X_MIN  = -0.50f;
        constexpr float DOG_BODY_X_MAX  = +0.50f;
        constexpr float DOG_BODY_Y_HALF = 0.35f;
        if (x >= DOG_BODY_X_MIN && x <= DOG_BODY_X_MAX
            && std::abs(y) <= DOG_BODY_Y_HALF) {
            continue;
        }

        // ★ 2. 排除 X<0 的所有点（狗后方任何东西都不关心） ★
        if (x < 0) continue;

        // ★ 3. 高度过滤：只看挡板高度范围 ★
        if (z < config::s03::WALL_Z_MIN || z > config::s03::WALL_Z_MAX) continue;

        // ----- 4. 前方剖面（用于触发转弯）★ Y 严格限制 ★ -----
        // 通道两侧墙在 Y≈±0.30 处，绝不能让它们进入前方判定
        if (x >= SCAN_X_MIN && x <= SCAN_X_MAX
            && std::abs(y) <= config::s03::WALL_Y_HALF) {
            int idx = static_cast<int>((x - SCAN_X_MIN) / SLICE_STEP);
            idx = std::max(0, std::min(idx, num_slices - 1));
            total_counts[idx]++;
            wall_counts[idx]++;
        }

        // ----- 5. 左右侧距（保留计算，State03 暂不使用） -----
        if (x >= config::s03::SIDE_X_MIN && x <= config::s03::SIDE_X_MAX) {
            if (y >= config::s03::SIDE_Y_MIN && y <= config::s03::SIDE_Y_MAX) {
                if (y < left_min) left_min = y;
            }
            if (y <= -config::s03::SIDE_Y_MIN && y >= -config::s03::SIDE_Y_MAX) {
                if ((-y) < right_min) right_min = -y;
            }
        }
    }

    // ===== 找最近的有挡板的剖面 =====
    float nearest = 9.99f;
    std::vector<SliceInfo> new_slices;
    new_slices.reserve(num_slices);

    int max_wall = 0;
    float max_wall_x = 9.99f;

    for (int i = 0; i < num_slices; ++i) {
        float xc = SCAN_X_MIN + i * SLICE_STEP;
        new_slices.push_back({xc, wall_counts[i], total_counts[i]});
        if (wall_counts[i] >= config::s03::WALL_POINT_THRESH && xc < nearest) {
            nearest = xc;
        }
        if (wall_counts[i] > max_wall) {
            max_wall = wall_counts[i];
            max_wall_x = xc;
        }
    }

    // ===== 实时调试打印（每 10 帧） =====
    static int dbg_cnt = 0;
    if (++dbg_cnt % 10 == 0) {
        std::cout << "[Lidar DEBUG] front=" << nearest
                  << "m  max_wall=" << max_wall
                  << "@X=" << max_wall_x << "m"
                  << "  total_pts=" << total_pts << std::endl;
    }

    // ===== 写入共享状态 =====
    std::lock_guard<std::mutex> lock(data_mutex_);
    front_wall_dist_ = nearest;
    left_dist_       = left_min;
    right_dist_      = right_min;
    debug_slices_    = std::move(new_slices);
}

float LidarHandler::get_front_wall_distance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return front_wall_dist_;
}

float LidarHandler::get_left_side_distance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return left_dist_;
}

float LidarHandler::get_right_side_distance() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return right_dist_;
}

std::vector<SliceInfo> LidarHandler::get_debug_slices() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return debug_slices_;
}

} // namespace control
} // namespace ruikang