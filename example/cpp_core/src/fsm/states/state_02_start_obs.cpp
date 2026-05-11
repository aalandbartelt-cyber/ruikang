// src/fsm/states/state_02_start_obs.cpp
// State02：纯里程触发跨栏 + 跳后寻迹 2 秒  + 逆时针自转 3° 切出
//
// 时间线：
//   1) enter          初始化里程累加器
//   2) 阶段A：里程累加  → traveled_dist_ ≥ D_JUMP_TRIGGER 触发跳跃
//   3) 阶段B：跳跃执行  → 刹车 + jump() + 恢复
//   4) 阶段C：跳后动作  → 寻迹 2 秒 (200 ticks) -> 原地逆时针转 3° (26 ticks) -> 切入 State03
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
    current_vx_              = config::s02::CRUISE_VX;  // 直接以目标速度起步，跳过斜坡防止不动
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
    // 阶段 C：跳跃完成后，固定寻迹 2 秒 -> 逆时针自转 3° -> 进 State03
    // ============================================================
    if (post_jump_following_) {
        post_jump_total_ticks_++;
        
        const int TRACK_TICKS = 200; // 寻迹时长：200 帧 (2秒)
        const int SPIN_TICKS  = 17;  // 自转时长：17 帧 (0.17秒，配合 0.2rad/s 转约 2°)
        
        // C.1 寻迹阶段 (0 ~ 2秒)
        if (post_jump_total_ticks_ <= TRACK_TICKS) {
            float offset = sm->vision_data.line_offset;
            float vyaw = sm->vel_ctrl.computeYaw(offset);
            
            if (sm->robot_driver) {
                sm->robot_driver->move(config::s02::CRUISE_VX, 0.0f, vyaw);
            }
            
            if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
                std::cout << "[跳后寻迹] total="
                          << post_jump_total_ticks_ * 10 << "ms / 2000ms"
                          << " offset=" << offset
                          << std::endl;
            }
        }
        // C.2 自转微调阶段 (2秒 ~ 2.26秒)
        else if (post_jump_total_ticks_ <= TRACK_TICKS + SPIN_TICKS) {
            if (post_jump_total_ticks_ == TRACK_TICKS + 1) {
                std::cout << "[FSM] 🌀 寻迹完成，开始逆时针自转 3° (0.2rad/s, 0.26s)..." << std::endl;
            }

            if (sm->robot_driver) {
                sm->robot_driver->move(0.0f, 0.0f, 0.2f);
            }
        }
        // C.3 切换阶段
        else {
            std::cout << "[FSM] ✅ 3°微调完成，刹车→切换步态: 经典→灵动，切入 STATE_03" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_AGILE);
            sm->changeState(new State03Avoidance());
            return;
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
                
                // ★ 进入跳后寻迹阶段
                std::cout << "[FSM] ✅ 跨栏完成，开始执行跳后寻迹（固定 2 秒）..." << std::endl;
                post_jump_following_   = true;
                is_jumping_            = false;
                post_jump_total_ticks_ = 0;
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