// src/fsm/states/state_09_end_obs.cpp
// State09: 终点障碍跳跃（纯里程触发）
//
// 流程：
//   enter          → 初始化里程累加器
//   阶段A：里程累加 → traveled_dist_ >= D_JUMP_TRIGGER 触发跳跃
//   阶段B：跳跃执行 → 刹车 + jump() + 恢复
//   阶段C：跳后寻迹 → 短距离直行进入蓝色启停区 → 切入 State10
// =====================================================
#include "fsm/states/state_09_end_obs.hpp"
#include "fsm/states/state_10_finish.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <unistd.h>

namespace ruikang {
namespace fsm {

void State09EndObs::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_09: 终点障碍跳跃" << std::endl;
    std::cout << "[FSM] D_JUMP_TRIGGER = "
              << config::s09::D_JUMP_TRIGGER << " m" << std::endl;

    is_jumping_            = false;
    post_jump_following_   = false;
    current_vx_            = 0.0f;
    accel_ignore_ticks_    = config::s09::ACCEL_IGNORE_TICKS;
    traveled_dist_         = 0.0f;
    post_jump_ticks_       = 0;
    log_tick_              = 0;
    last_tick_             = std::chrono::steady_clock::now();

    cruise_mode_ = (accel_ignore_ticks_ == 0);
    if (cruise_mode_) {
        std::cout << "[FSM] 跳过加速段，立即开始累计里程" << std::endl;
    }

    sm->vel_ctrl.reset();
}

void State09EndObs::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    // ===== 跳跃进行中，阻塞 execute =====
    if (is_jumping_) return;

    // ============================================================
    // 阶段 C：跳跃后寻迹进入蓝色启停区 → State10
    // ============================================================
    if (post_jump_following_) {
        post_jump_ticks_++;

        if (post_jump_ticks_ > config::s09::POST_JUMP_TICKS) {
            std::cout << "[FSM] 进入蓝色启停区，切入 STATE_10" << std::endl;
            sm->changeState(new State10Finish());
            return;
        }

        float offset = sm->vision_data.line_offset;
        float vyaw   = sm->vel_ctrl.computeYaw(offset);
        sm->robot_driver->move(config::s09::CRUISE_VX, 0.0f, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[跳后寻迹] " << post_jump_ticks_ << " / "
                      << config::s09::POST_JUMP_TICKS << " ticks"
                      << " offset=" << offset << std::endl;
        }
        return;
    }

    // ============================================================
    // 阶段 A：寻迹 + 里程累加
    // ============================================================
    float offset = sm->vision_data.line_offset;
    auto now = std::chrono::steady_clock::now();

    // A.1 加速段倒计时
    if (accel_ignore_ticks_ > 0) {
        accel_ignore_ticks_--;
        if (accel_ignore_ticks_ == 0) {
            cruise_mode_   = true;
            last_tick_     = now;
            traveled_dist_ = 0.0f;
            std::cout << "[FSM] 加速段结束，里程累计开始..." << std::endl;
        }
    }
    // A.2 累计里程 + 检查触发
    else if (cruise_mode_) {
        float dt = std::chrono::duration<float>(now - last_tick_).count();
        last_tick_ = now;
        traveled_dist_ += current_vx_ * dt;

        // ★ 触发跳跃
        if (traveled_dist_ >= config::s09::D_JUMP_TRIGGER) {
            std::cout << "\n[FSM] 达到触发距离！traveled = "
                      << traveled_dist_ << "m, target = "
                      << config::s09::D_JUMP_TRIGGER << "m" << std::endl;

            is_jumping_ = true;

            if (config::debug::DRY_RUN_NO_JUMP) {
                std::cout << "[FSM] DRY_RUN：仅刹车不跳跃" << std::endl;
                sm->robot_driver->move(0, 0, 0);
                sleep(5);
                return;
            }

            std::cout << "[FSM] 刹车 " << config::s09::BRAKE_BEFORE_JUMP << "s ..." << std::endl;
            sm->robot_driver->move(0, 0, 0);
            usleep(static_cast<int>(config::s09::BRAKE_BEFORE_JUMP * 1e6));

            std::cout << "[FSM] ACTION: JUMP!" << std::endl;
            sm->robot_driver->jump();

            usleep(static_cast<int>(config::s09::RECOVER_AFTER_JUMP * 1e6));

            std::cout << "[FSM] 跨栏完成，寻迹进入蓝色启停区..." << std::endl;
            post_jump_following_ = true;
            is_jumping_          = false;
            post_jump_ticks_     = 0;
            log_tick_            = 0;
            sm->vel_ctrl.reset();
            return;
        }
    }

    // A.3 寻迹速度计算
    float target_vx = config::s09::CRUISE_VX;
    if (std::abs(offset) > 80.0f) target_vx = config::s09::SLOW_VX;

    if (current_vx_ < target_vx - 0.006f)      current_vx_ += 0.005f;
    else if (current_vx_ > target_vx + 0.016f) current_vx_ -= 0.015f;
    else                                        current_vx_  = target_vx;

    float vyaw = sm->vel_ctrl.computeYaw(offset);
    if (current_vx_ < 0.15f && std::abs(offset) < 80.0f) vyaw *= 0.5f;

    sm->robot_driver->move(current_vx_, 0.0f, vyaw);

    if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
        std::cout << "[终点]" << (cruise_mode_ ? "[匀速]" : "[加速]")
                  << " traveled=" << traveled_dist_ << "m"
                  << " vx=" << current_vx_
                  << " offset=" << offset
                  << " (target=" << config::s09::D_JUMP_TRIGGER << "m)"
                  << std::endl;
    }
}

void State09EndObs::exit(StateMachine* sm) {
    if (sm->robot_driver) sm->robot_driver->move(0, 0, 0);
    std::cout << "[FSM] <<< 退出 STATE_09" << std::endl;
}

} // namespace fsm
} // namespace ruikang
