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
        APPROACH,          // 寻迹直行，等待 ArUco
        MOVE_TO_STAIRS,    // 检测到 ArUco → 盲走靠近台阶
        ALIGN_ARUCO,       // 对齐台阶中心
        CLIMB_ARC,         // ★ 弧线连贯上下台阶（前进+左转画弧，单一动作）
        SWITCH_GAIT_DOWN,  // 切回步态：灵动 → 经典
        EXIT_FOLLOW,       // 继续寻迹，等待离开台阶区域
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif
