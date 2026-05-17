// src/fsm/states/state_10_finish.cpp
// State10: 蓝色启停区 — 稳定停靠 + SDK 站立锁定
//
// 流程：
//   enter   → 刹车停稳
//   execute → 等待稳定 → BalanceStand() 站立锁定 → 任务完成
// =====================================================
#include "fsm/states/state_10_finish.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>

namespace ruikang {
namespace fsm {

void State10Finish::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_10: 终点停靠" << std::endl;
    std::cout << "[FSM] 任务完成！进入蓝色启停区，准备 SDK 站立锁定" << std::endl;

    standing_locked_  = false;
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
        if (!standing_locked_) {
            std::cout << "[FSM] State10 超时，强制站立锁定" << std::endl;
            sm->robot_driver->balanceStand();
            standing_locked_ = true;
        }
        return;
    }

    // 停稳阶段：持续发零速让狗站稳
    if (!standing_locked_) {
        sm->robot_driver->move(0, 0, 0);

        if (elapsed < config::s10::STOP_DELAY) {
            if (++log_tick_ % 30 == 0) {
                std::cout << "[终点][STOP] 停稳中... " << elapsed << "s / "
                          << config::s10::STOP_DELAY << "s" << std::endl;
            }
            return;
        }

        // SDK BalanceStand: 主动平衡站立，电机保持力矩不卸力
        sm->robot_driver->balanceStand();
        standing_locked_ = true;
    }

    // 站立锁定后无需再发指令，BalanceStand 是持续模式
    if (++log_tick_ % 100 == 0) {
        std::cout << "[FSM] 任务已完成，机器人已 SDK 站立锁定" << std::endl;
    }
}

void State10Finish::exit(StateMachine* sm) {
    if (sm->robot_driver) {
        sm->robot_driver->move(0, 0, 0);
    }
    std::cout << "[FSM] <<< 退出 STATE_10（任务结束）" << std::endl;
}

} // namespace fsm
} // namespace ruikang
