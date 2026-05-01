// include/fsm/state_machine.hpp
#pragma once
#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "control/hal/robot_driver.hpp"
#include "control/hal/lidar_handler.hpp"   // ← 新增
#include "common/udp_receiver.hpp"
#include "control/locomotion/velocity_ctrl.hpp"

namespace ruikang {
namespace fsm {

class StateBase;

class StateMachine {
public:
    control::RobotDriver*  robot_driver;
    control::LidarHandler* lidar_handler;  // ← 新增
    StateBase*             current_state;
    VisionData             vision_data;
    control::VelocityCtrl  vel_ctrl;
    
    // ★ 跨状态共享：选哪号放置平台（State05 写入，State08 读取）
    int memorized_tag = 0;

    StateMachine(control::RobotDriver* driver,
                 control::LidarHandler* lidar,
                 StateBase* initial_state);
    ~StateMachine();

    void update(const VisionData& data);
    void changeState(StateBase* new_state);
};

} // namespace fsm
} // namespace ruikang

#endif