#include "fsm/state_machine.hpp"
#include "fsm/state_base.hpp" // 🌟 必须在这里引入！
#include <iostream>

namespace ruikang {
namespace fsm {

StateMachine::StateMachine(control::RobotDriver* driver, StateBase* initial_state)
    : robot_driver(driver), current_state(nullptr) { 
    if (initial_state != nullptr) {
        changeState(initial_state);
    }
}

StateMachine::~StateMachine() {
    if (current_state != nullptr) {
        current_state->exit(this);
        delete current_state;
    }
}

void StateMachine::changeState(StateBase* new_state) {
    if (current_state != nullptr) {
        current_state->exit(this);
        delete current_state;
    }
    
    current_state = new_state;
    
    if (current_state != nullptr) {
        current_state->enter(this);
    }
}

void StateMachine::update(const VisionData& data) {
    vision_data = data;
    
    if (current_state != nullptr) {
        current_state->execute(this);
    }
}

} 
}