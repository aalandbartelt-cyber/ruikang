#include "control/locomotion/velocity_ctrl.hpp"

namespace ruikang {
namespace control {

// 台阶专用：恒定低速前进，不转向，防跌落
VelocityCmd VelocityCtrl::getStairClimbVelocity() {
    return {0.06f, 0.0f, 0.0f};
}

// 正常巡线：利用 PID 偏航 + 动态线速度（弯道自动减速）
VelocityCmd VelocityCtrl::getNormalTrackingVelocity(float current_offset, float base_vx) {
    float vyaw = computeYaw(current_offset);

    // 弯道动态减速（安全过弯）
    float vx = base_vx;
    if (std::abs(vyaw) > 0.4f) {
        vx = base_vx * 0.5f;
    } else if (std::abs(vyaw) > 0.2f) {
        vx = base_vx * 0.7f;
    }

    return {vx, 0.0f, vyaw};
}

// 紧急刹车
VelocityCmd VelocityCtrl::getStopVelocity() {
    return {0.0f, 0.0f, 0.0f};
}

} // namespace control
} // namespace ruikang
