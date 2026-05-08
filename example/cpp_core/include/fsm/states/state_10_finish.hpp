#ifndef STATE_10_FINISH_HPP
#define STATE_10_FINISH_HPP

#include "fsm/state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State10Finish : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    bool locked_ = false;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_10_FINISH_HPP
