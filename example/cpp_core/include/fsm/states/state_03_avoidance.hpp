//状态3：避障区无碰撞穿行
#ifndef STATE_03_AVOIDANCE_HPP
#define STATE_03_AVOIDANCE_HPP

#include "fsm/state_base.hpp" // 🌟 已经修正为你们正确的基类头文件

namespace ruikang {
namespace fsm {

// 注意：这里假设你们的基类叫 StateBase，如果报错说找不到 StateBase，
// 请把它改回 public State
class State03Avoidance : public StateBase {
public:
    void enter(StateMachine* sm) override;
    void execute(StateMachine* sm) override;
    void exit(StateMachine* sm) override;

private:
    int turn_count_ = 0;      // 记录转弯的次数，用于决定左右方向
    bool is_turning_ = false; // 状态锁：防止转弯期间重复触发雷达判定
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_03_AVOIDANCE_HPP