#ifndef STATE_04_STAIRS_HPP
#define STATE_04_STAIRS_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State04Stairs : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    enum class Phase {
        APPROACH,          // 巡线直行 + ArUco center_y 判断靠近
        ALIGN_ARUCO,       // 对齐台阶中心
        CLIMB_UP,          // 直爬1s上台阶
        CLIMB_ARC,         // 弧线前进+左转 90°
        CLIMB_DOWN,        // 直下1s
        EXIT_FOLLOW,       // 巡线离开（先巡3s再切步态）
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    float accumulated_yaw_ = 0.0f;  // CLIMB_ARC 角度累计
    bool  gait_switched_   = false; // EXIT_FOLLOW 中是否已切步态

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif
