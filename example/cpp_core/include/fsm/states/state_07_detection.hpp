#ifndef STATE_07_DETECTION_HPP
#define STATE_07_DETECTION_HPP

#include "fsm/state_base.hpp"
#include "control/action/action_manager.hpp"
#include <chrono>
#include <string>
#include <memory>

namespace ruikang {
namespace fsm {

class State07Detection : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    enum class Phase {
        APPROACH,        // 寻迹直行，等待红点出现
        RED_DOT_ALIGN,   // 红点居中 → 狗投影精准覆盖检测点
        STOP_AND_READ,   // 停稳，读取警示牌识别结果
        EXECUTE_ACTION,  // 调用 ActionManager 触发动作
        WAIT_ACTION,     // 轮询 ActionManager::isDone()
        EXIT_FOLLOW,     // 继续巡线离开检测点
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    std::string action_to_play_;
    std::unique_ptr<ruikang::control::ActionManager> action_mgr_;

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif
