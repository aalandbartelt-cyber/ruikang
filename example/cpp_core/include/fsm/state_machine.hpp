#pragma once // 🌟 防死循环终极魔法
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "control/hal/robot_driver.hpp"
#include "common/udp_receiver.hpp"
#include "control/locomotion/velocity_ctrl.hpp"

// ❌ 绝对不要在这里 include state_base.hpp ！

namespace ruikang {
namespace fsm {

class StateBase; // ✅ 只需要这一句“前向声明”就足够了

class StateMachine {
public:
    control::RobotDriver* robot_driver;
    StateBase* current_state;
    VisionData vision_data;
    control::VelocityCtrl vel_ctrl;

    StateMachine(control::RobotDriver* driver, StateBase* initial_state);
    ~StateMachine();

    void update(const VisionData& data);
    void changeState(StateBase* new_state);
};

} // namespace fsm
} // namespace ruikang

#endif // STATE_MACHINE_HPP