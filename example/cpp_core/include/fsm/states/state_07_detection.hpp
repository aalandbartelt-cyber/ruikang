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
        APPROACH,         // 寻迹直行，等待红点出现
        SECOND_TURN,      // 双弯：第一个弯完成后硬转90°补第二个弯
        MOVE_TO_DOT,      // ★ 红点出现后继续巡线逼近（补偿D435i前倾视角）
        RED_DOT_ALIGN,   // 红点居中 → 狗投影精准覆盖检测点
        TURN_TO_SIGN,    // 左转 90° 面向警示牌
        BACK_AWAY,       // 后退拉开距离（离警示牌太近）
        STOP_AND_READ,   // 停稳，读取警示牌识别结果
        EXECUTE_ACTION,  // 调用 ActionManager 触发动作
        WAIT_ACTION,     // 轮询 ActionManager::isDone()
        TURN_BACK,       // 右转 90° 回正，面向巡线方向
        EXIT_FOLLOW,     // 继续巡线离开检测点
        FINISHED
    };

    Phase phase_ = Phase::APPROACH;

    std::string action_to_play_;
    std::unique_ptr<ruikang::control::ActionManager> action_mgr_;
    float accumulated_yaw_ = 0.0f;  // 转弯角度累积

    std::chrono::steady_clock::time_point phase_start_;
    std::chrono::steady_clock::time_point state_enter_time_;
    int log_tick_ = 0;
    int   sharp_ticks_   = 0;     // 连续急弯帧计数
    float second_turn_dir_ = 0.0f; // 第二个弯转向（+右 -左）
    bool  second_turn_done_ = false; // 只补一次
};

} // namespace fsm
} // namespace ruikang

#endif
