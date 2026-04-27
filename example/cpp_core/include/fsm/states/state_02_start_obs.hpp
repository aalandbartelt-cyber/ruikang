#ifndef STATE_02_START_OBS_HPP
#define STATE_02_START_OBS_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State02StartObs : public StateBase {
private:
    // 之前用于跳跃逻辑的状态变量（目前闭环寻迹中设为 false）
    bool is_jumping_;
    std::chrono::time_point<std::chrono::steady_clock> action_start_time_;
    
    // 🌟🌟🌟 【核心状态变量】 🌟🌟🌟
    // 用于记录底盘当前的前向速度，实现丝滑的软起动（Ramp-up）和弯道动态加减速
    float current_vx_; 

public:
    // 状态机的标准生命周期函数
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_02_START_OBS_HPP