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
        APPROACH,          // 寻迹直行，向台阶靠近，等待 ArUco
        ALIGN_ARUCO,       // 检测到 ArUco → 对齐台阶中心
        CLIMB_UP,          // 上台阶（State03已是灵动步态，无需切步态）
        TOP_TURN_LEFT,     // ★ 台阶顶部左转 90°，转向下台阶方向
        CLIMB_DOWN,        // 下台阶
        SWITCH_GAIT_DOWN,  // 切回步态：灵动 → 经典
        EXIT_FOLLOW,       // 继续寻迹，等待离开台阶区域
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    // 转弯累计（TOP_TURN_LEFT 用）
    float accumulated_yaw_ = 0.0f;

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif
