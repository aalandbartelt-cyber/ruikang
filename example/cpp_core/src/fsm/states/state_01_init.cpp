//state_01_init.cpp  
#include "fsm/states/state_01_init.hpp"
#include "fsm/states/state_02_start_obs.hpp" // 包含下一个状态的头文件
#include "fsm/state_machine.hpp"
#include <iostream>

namespace ruikang {
namespace fsm {

void State01Init::enter(StateMachine* sm) {
    std::cout << "[FSM] Enter STATE_01_INIT: System Booting. Waiting 3 seconds..." << std::endl;
    // 记录进入该状态的时间点
    start_time_ = std::chrono::steady_clock::now();
    
    // 确保狗在原地不要动
    sm->robot_driver->move(0.0f, 0.0f, 0.0f);
}

void State01Init::execute(StateMachine* sm) {
    // 计算已经经过的时间
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(now - start_time_).count();

    // 假设：等待3秒后，或者收到了来自遥控器的开始指令（这里演示时间触发）
    if (elapsed_seconds > 3.0) {
        std::cout << "[FSM] Ready to Go!" << std::endl;
        // 【核心操作】：触发状态切换！
        sm->changeState(new State02StartObs());
    }
}

void State01Init::exit(StateMachine* sm) {
    std::cout << "[FSM] Exit STATE_01_INIT: Starting the race." << std::endl;
}

}
}