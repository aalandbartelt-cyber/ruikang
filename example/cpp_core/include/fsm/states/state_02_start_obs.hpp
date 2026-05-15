#ifndef STATE_02_START_OBS_HPP
#define STATE_02_START_OBS_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State02StartObs : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    // ===== 阶段标志 =====
    bool is_jumping_           = false;   // 跳跃执行中（阻塞 execute）
    bool post_jump_following_  = false;   // 跳跃完成后，进入寻迹收尾
    
    // ===== 寻迹速度 =====
    float current_vx_ = 0.0f;
    
    // ===== 里程触发核心 =====
    int   accel_ignore_ticks_ = 0;
    bool  cruise_mode_        = false;
    float traveled_dist_      = 0.0f;
    std::chrono::steady_clock::time_point last_tick_;
    
    // ===== 跳跃后寻迹收尾 =====
    int post_jump_total_ticks_ = 0;   // 跳跃后已经过的总 ticks
    
    // ===== 调试 =====
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_02_START_OBS_HPP