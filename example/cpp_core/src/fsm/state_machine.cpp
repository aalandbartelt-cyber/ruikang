#include "fsm/state_machine.hpp"
#include "fsm/state_base.hpp"
#include <iostream>

namespace ruikang {
namespace fsm {

StateMachine::StateMachine(control::RobotDriver* driver,
                           control::LidarHandler* lidar,
                           StateBase* initial_state)
    : robot_driver(driver),
      lidar_handler(lidar),
      current_state(nullptr) {
    if (initial_state) changeState(initial_state);
}

StateMachine::~StateMachine() {
    if (current_state) {
        current_state->exit(this);
        delete current_state;
    }
}

void StateMachine::changeState(StateBase* new_state) {
    if (current_state) {
        current_state->exit(this);
        delete current_state;
    }
    current_state = new_state;
    if (current_state) current_state->enter(this);
}

void StateMachine::update(const VisionData& data) {
    vision_data = data;
    if (current_state) current_state->execute(this);
}

} // namespace fsm
} // namespace ruikang