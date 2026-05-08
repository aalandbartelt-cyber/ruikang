// src/fsm/states/state_10_finish.cpp
// State10: 蓝色启停区 — 稳定停靠 + 完全锁死电机
//
// 流程：
//   enter   → 刹车停稳
//   execute → 等待稳定 → lockMotors() 切断力矩进入阻尼态 → 任务完成
// =====================================================
#include "fsm/states/state_10_finish.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>

namespace ruikang {
namespace fsm {

void State10Finish::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_10: 终点停靠" << std::endl;
    std::cout << "[FSM] 任务完成！进入蓝色启停区，准备锁死电机" << std::endl;

    locked_  = false;
    log_tick_ = 0;
    state_enter_time_ = std::chrono::steady_clock::now();

    if (sm->robot_driver) {
        sm->robot_driver->move(0, 0, 0);
    }
}

void State10Finish::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - state_enter_time_).count();

    // 全局超时保护
    if (elapsed > config::s10::TOTAL_TIMEOUT) {
        if (!locked_) {
            std::cout << "[FSM] State10 超时，强制锁死" << std::endl;
            sm->robot_driver->lockMotors();
            locked_ = true;
        }
        return;
    }

    // 等待稳定后再锁死
    if (!locked_) {
        if (elapsed < config::s10::STOP_DELAY) {
            sm->robot_driver->move(0, 0, 0);

            if (++log_tick_ % 30 == 0) {
                std::cout << "[终点][STOP] 停稳中... " << elapsed << "s / "
                          << config::s10::STOP_DELAY << "s" << std::endl;
            }
            return;
        }

        std::cout << "[FSM] 电机锁死！进入阻尼态" << std::endl;
        sm->robot_driver->lockMotors();
        locked_ = true;
    }

    // 锁死后什么都不做，安静等待
    if (++log_tick_ % 100 == 0) {
        std::cout << "[FSM] 任务已完成，机器人已安全锁死" << std::endl;
    }
}

void State10Finish::exit(StateMachine* sm) {
    // 确保无论如何都是锁死状态
    if (sm->robot_driver && !locked_) {
        sm->robot_driver->lockMotors();
    }
    std::cout << "[FSM] <<< 退出 STATE_10（任务结束）" << std::endl;
}

} // namespace fsm
} // namespace ruikang
