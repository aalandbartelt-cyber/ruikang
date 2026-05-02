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
    enum class Phase {
        FORWARD,
        TURNING,
        STABILIZING,
        FINISHED
    };

    Phase phase_ = Phase::FORWARD;
    int   turn_count_      = 0;
    float accumulated_yaw_ = 0.0f;
    bool  current_turn_left_ = true;

    // ★ 连续帧确认计数器（防抖）
    int   confirm_ticks_ = 0;

    std::chrono::steady_clock::time_point phase_start_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif