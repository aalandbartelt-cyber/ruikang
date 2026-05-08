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
        FORWARD,       // 直行 + 等待触发
        TURNING,       // 阿克曼转弯
        STABILIZING,   // 稳定缓冲 + 姿态校正
        FINISHED       // 完成
    };

    Phase phase_ = Phase::FORWARD;
    int   turn_count_        = 0;
    float accumulated_yaw_   = 0.0f;
    bool  current_turn_left_ = true;

    // 连续帧确认计数器
    int   confirm_ticks_       = 0;
    int   side_gap_confirm_    = 0;   // 门禁 3 计数器

    // 深度滑动窗口
    static constexpr int DEPTH_HISTORY_SIZE = 5;
    float depth_history_[DEPTH_HISTORY_SIZE] = {0.0f};
    int   depth_history_idx_  = 0;
    int   depth_history_fill_ = 0;

    // 侧向深度历史（用于开口检测）
    float prev_side_depth_ = 0.0f;
    bool  prev_side_valid_ = false;

    std::chrono::steady_clock::time_point phase_start_;
    int log_tick_ = 0;

    // 辅助函数
    float getSmoothedDepth();
    float getSideDepthForDirection(bool turn_left);
    bool  isDepthValid(float d);
};

} // namespace fsm
} // namespace ruikang

#endif
