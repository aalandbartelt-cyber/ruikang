//状态1：等待起跑发令与环境初始化
#pragma once
#include "../state_base.hpp"
#include <chrono>

namespace ruikang {
namespace fsm {

class State01Init : public StateBase {
private:
    std::chrono::steady_clock::time_point start_time_;
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;
};

}
}