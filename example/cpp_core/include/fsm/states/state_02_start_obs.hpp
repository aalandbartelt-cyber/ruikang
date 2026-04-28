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
    bool is_jumping_ = false;
    float current_vx_ = 0.0f;
    int obs_confirm_count_ = 0; 
    
    // 🌟 核心：起步保护期倒计时器
    int startup_ignore_ticks_ = 0; 
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_02_START_OBS_HPP