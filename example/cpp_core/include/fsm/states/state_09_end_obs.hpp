#ifndef STATE_09_END_OBS_HPP
#define STATE_09_END_OBS_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State09EndObs : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    // ===== 跳跃执行中（阻塞 execute，参考 State02 模式） =====
    bool is_jumping_          = false;
    bool post_jump_following_ = false;

    // ===== 寻迹速度 =====
    float current_vx_ = 0.0f;

    // ===== 里程触发核心 =====
    int   accel_ignore_ticks_ = 0;
    bool  cruise_mode_        = false;
    float traveled_dist_      = 0.0f;
    std::chrono::steady_clock::time_point last_tick_;

    // ===== 跳跃后寻迹进入蓝色启停区 =====
    int post_jump_ticks_ = 0;

    // ===== 调试 =====
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_09_END_OBS_HPP
