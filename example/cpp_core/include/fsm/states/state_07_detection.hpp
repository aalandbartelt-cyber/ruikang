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
        APPROACH,         // 寻迹直行，等待红点（弯后短暂boost过双弯）
        MOVE_TO_DOT,      // ★ 红点出现后继续巡线逼近（补偿D435i前倾视角）
        RED_DOT_ALIGN,    // 红点居中 → 狗投影精准覆盖检测点
        TURN_TO_SIGN,     // 左转 90° 面向警示牌
        BACK_AWAY,        // 后退拉开距离（离警示牌太近）
        STOP_AND_READ,    // 停稳，读取警示牌识别结果
        EXECUTE_ACTION,   // 调用 ActionManager 触发动作
        WAIT_ACTION,      // 轮询 ActionManager::isDone()
        TURN_BACK,        // 右转 90° 回正，面向巡线方向
        EXIT_FOLLOW,      // 继续巡线离开检测点
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    std::string action_to_play_;
    std::unique_ptr<ruikang::control::ActionManager> action_mgr_;
    float accumulated_yaw_ = 0.0f;  // 转弯角度累积

    // 弯后短暂boost：过完急弯后50帧内vyaw翻倍，帮助捕捉连续第二个弯
    bool was_in_turn_      = false;
    int  post_turn_boost_  = 0;

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
};

} // namespace fsm
} // namespace ruikang

#endif
