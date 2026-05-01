// src/fsm/states/state_02_start_obs.cpp
// State02：纯里程触发跨栏 + 跳后寻迹收尾
//
// 时间线：
//   1) enter           初始化里程累加器
//   2) 阶段A：里程累加  → traveled_dist_ ≥ D_JUMP_TRIGGER 触发跳跃
//   3) 阶段B：跳跃执行  → 刹车 + jump() + 恢复
//   4) 阶段C：跳后寻迹  → 满足下列任一条件后切 State03：
//                        - 寻迹超过 POST_JUMP_MIN_TICKS 且连续 NO_LINE_TICKS_TO_EXIT 帧无线
//                        - 总 ticks 超过 POST_JUMP_TIMEOUT_TICKS（兜底超时）
//
#include "fsm/states/state_02_start_obs.hpp"
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <cmath>
#include <unistd.h>

namespace ruikang {
namespace fsm {

// =====================================================
// enter
// =====================================================
void State02StartObs::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_02: 寻迹 + 里程触发跨栏" << std::endl;
    std::cout << "[FSM] D_JUMP_TRIGGER = "
              << config::s02::D_JUMP_TRIGGER << " m" << std::endl;
    
    // 重置所有状态
    is_jumping_              = false;
    post_jump_following_     = false;
    current_vx_              = 0.0f;
    accel_ignore_ticks_      = config::s02::ACCEL_IGNORE_TICKS;
    traveled_dist_           = 0.0f;
    post_jump_total_ticks_   = 0;
    no_line_ticks_           = 0;
    log_tick_                = 0;
    last_tick_               = std::chrono::steady_clock::now();
    
    // ★ 关键：ACCEL_IGNORE_TICKS == 0 时直接进入累计模式
    cruise_mode_ = (accel_ignore_ticks_ == 0);
    
    if (cruise_mode_) {
        std::cout << "[FSM] ⚡ 跳过加速段，立即开始累计里程" << std::endl;
    } else {
        std::cout << "[FSM] ⏳ 加速段忽略 "
                  << accel_ignore_ticks_ << " ticks ("
                  << accel_ignore_ticks_ * 10 << "ms)" << std::endl;
    }
    
    sm->vel_ctrl.reset();
}

// =====================================================
// execute
// =====================================================
void State02StartObs::execute(StateMachine* sm) {
    if (is_jumping_) return;
    
    // ============================================================
    // 阶段 C：跳跃完成后，继续寻迹直到出线 or 超时
    // ============================================================
    if (post_jump_following_) {
        post_jump_total_ticks_++;
        float offset = sm->vision_data.line_offset;
        
        // 出线判定：连续 N 帧 |offset| 异常
        if (std::abs(offset) > config::s02::NO_LINE_OFFSET_THRESH) {
            no_line_ticks_++;
        } else {
            no_line_ticks_ = 0;
        }
        
        // ----- 退出条件 1：满足"最小寻迹时长 + 连续无线" -----
        bool min_time_passed = (post_jump_total_ticks_ >= config::s02::POST_JUMP_MIN_TICKS);
        bool no_line_enough  = (no_line_ticks_ >= config::s02::NO_LINE_TICKS_TO_EXIT);
        if (min_time_passed && no_line_enough) {
            std::cout << "[FSM] ✅ 出线判定达成（已寻迹 "
                      << post_jump_total_ticks_ * 10 << "ms，连续无线 "
                      << no_line_ticks_ * 10 << "ms）→ 切 STATE_03" << std::endl;
            sm->changeState(new State03Avoidance());
            return;
        }
        
        // ----- 退出条件 2：兜底超时 -----
        if (post_jump_total_ticks_ >= config::s02::POST_JUMP_TIMEOUT_TICKS) {
            std::cout << "[FSM] ⚠️ 跳后寻迹超时（"
                      << config::s02::POST_JUMP_TIMEOUT_TICKS * 10
                      << "ms），强制切 STATE_03" << std::endl;
            sm->changeState(new State03Avoidance());
            return;
        }
        
        // 正常寻迹（保持小速度匀速）
        float vyaw = sm->vel_ctrl.computeYaw(offset);
        if (sm->robot_driver) {
            sm->robot_driver->move(config::s02::CRUISE_VX, 0.0f, vyaw);
        }
        
        if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
            std::cout << "[跳后寻迹] total="
                      << post_jump_total_ticks_ * 10 << "ms"
                      << " offset=" << offset
                      << " no_line=" << no_line_ticks_
                      << "/" << config::s02::NO_LINE_TICKS_TO_EXIT
                      << " (min_pass=" << (min_time_passed ? "Y" : "N") << ")"
                      << std::endl;
        }
        return;
    }
    
    // ============================================================
    // 阶段 A：寻迹 + 里程累加
    // ============================================================
    float offset = sm->vision_data.line_offset;
    auto now = std::chrono::steady_clock::now();
    
    // ----- A.1 加速段倒计时（如果有） -----
    if (accel_ignore_ticks_ > 0) {
        accel_ignore_ticks_--;
        if (accel_ignore_ticks_ == 0) {
            cruise_mode_   = true;
            last_tick_     = now;
            traveled_dist_ = 0.0f;
            std::cout << "[FSM] ✅ 加速段结束，里程累计开始..." << std::endl;
        }
    }
    // ----- A.2 累计里程 + 检查触发 -----
    else if (cruise_mode_) {
        float dt = std::chrono::duration<float>(now - last_tick_).count();
        last_tick_ = now;
        traveled_dist_ += current_vx_ * dt;
        
        // ★ 触发跳跃 ★
        if (traveled_dist_ >= config::s02::D_JUMP_TRIGGER) {
            std::cout << "\n[FSM] 🎯 达到触发距离！traveled = "
                      << traveled_dist_ << "m, target = "
                      << config::s02::D_JUMP_TRIGGER << "m" << std::endl;
            
            is_jumping_ = true;
            
            // DRY_RUN 模式：只刹车不跳，方便用卷尺标定
            if (config::debug::DRY_RUN_NO_JUMP) {
                std::cout << "[FSM] 🧪 DRY_RUN：仅刹车不跳跃" << std::endl;
                if (sm->robot_driver) sm->robot_driver->move(0, 0, 0);
                sleep(5);
                return;
            }
            
            // 正式跳跃流程
            if (sm->robot_driver) {
                std::cout << "[FSM] 🛑 刹车 "
                          << config::s02::BRAKE_BEFORE_JUMP << "s ..." << std::endl;
                sm->robot_driver->move(0, 0, 0);
                usleep(static_cast<int>(config::s02::BRAKE_BEFORE_JUMP * 1e6));
                
                std::cout << "[FSM] 🐕 ACTION: JUMP!" << std::endl;
                sm->robot_driver->jump();
                
                usleep(static_cast<int>(config::s02::RECOVER_AFTER_JUMP * 1e6));
                
                // ★ 进入跳后寻迹阶段，不立刻切 State03
                std::cout << "[FSM] ✅ 跨栏完成，进入跳后寻迹阶段（最小 "
                          << config::s02::POST_JUMP_MIN_TICKS * 10 << "ms）" << std::endl;
                post_jump_following_   = true;
                is_jumping_            = false;
                post_jump_total_ticks_ = 0;
                no_line_ticks_         = 0;
                log_tick_              = 0;
                sm->vel_ctrl.reset();
                return;
            }
        }
    }
    
    // ----- A.3 寻迹速度计算 -----
    float target_vx = config::s02::CRUISE_VX;
    if (std::abs(offset) > 80.0f) target_vx = config::s02::SLOW_VX;
    
    if (current_vx_ < target_vx - 0.006f)      current_vx_ += 0.005f;
    else if (current_vx_ > target_vx + 0.016f) current_vx_ -= 0.015f;
    else                                        current_vx_  = target_vx;
    
    float vyaw = sm->vel_ctrl.computeYaw(offset);
    if (current_vx_ < 0.15f && std::abs(offset) < 80.0f) vyaw *= 0.5f;
    
    if (sm->robot_driver) sm->robot_driver->move(current_vx_, 0.0f, vyaw);
    
    // ----- A.4 调试打印 -----
    if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
        std::string phase = cruise_mode_ ? "[匀速]" : "[加速]";
        std::cout << phase
                  << " traveled=" << traveled_dist_ << "m"
                  << " vx=" << current_vx_
                  << " offset=" << offset
                  << " (target=" << config::s02::D_JUMP_TRIGGER << "m)"
                  << std::endl;
    }
}

// =====================================================
// exit
// =====================================================
void State02StartObs::exit(StateMachine* sm) {
    if (sm->robot_driver) sm->robot_driver->move(0, 0, 0);
    std::cout << "[FSM] <<< 退出 STATE_02" << std::endl;
}

} // namespace fsm
} // namespace ruikang