#ifndef STATE_03_AVOIDANCE_HPP
#define STATE_03_AVOIDANCE_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State03Avoidance : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    // 子阶段状态机
    enum class Phase {
        FORWARD,         // 直行，等遇墙
        TURNING,         // 阿克曼转弯
        STABILIZING,     // 转完稳定一下
        FINISHED         // 完成所有 5 次转弯
    };
    
    Phase phase_ = Phase::FORWARD;
    int   turn_count_      = 0;            // 已完成的转弯次数 (0~5)
    float accumulated_yaw_ = 0.0f;         // 当前转弯已累计的角度
    bool  current_turn_left_ = true;       // 本次转弯方向
    
    std::chrono::steady_clock::time_point phase_start_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif