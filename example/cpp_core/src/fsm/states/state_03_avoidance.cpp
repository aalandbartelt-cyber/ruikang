// src/fsm/states/state_03_avoidance.cpp
// V3.0：禁用侧墙警戒（噪声大），只用前方距离触发
//
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <cmath>

namespace ruikang {
namespace fsm {

void State03Avoidance::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_03: 避障区（5 次转弯）" << std::endl;
    std::cout << "[FSM] 转弯顺序: ";
    for (int i = 0; i < config::s03::TOTAL_TURNS; ++i) {
        std::cout << (config::s03::TURN_DIRECTIONS[i] ? "左 " : "右 ");
    }
    std::cout << std::endl;
    
    phase_           = Phase::FORWARD;
    turn_count_      = 0;
    accumulated_yaw_ = 0.0f;
    log_tick_        = 0;
    phase_start_     = std::chrono::steady_clock::now();
}

void State03Avoidance::execute(StateMachine* sm) {
    if (!sm->lidar_handler || !sm->robot_driver) return;
    
    auto now = std::chrono::steady_clock::now();
    float wall_dist = sm->lidar_handler->get_front_wall_distance();

    // ===== 子阶段 1：直行 =====
    if (phase_ == Phase::FORWARD) {
        // 完成 5 次转弯就退出
        if (turn_count_ >= config::s03::TOTAL_TURNS) {
            std::cout << "[迷宫] 🎉 完成 " << config::s03::TOTAL_TURNS 
                      << " 次转弯，退出 State03" << std::endl;
            phase_ = Phase::FINISHED;
            sm->robot_driver->move(0, 0, 0);
            return;
        }
        
        // 检查前方挡板
        if (wall_dist > 0.10f && wall_dist <= config::s03::TURN_TRIGGER_DIST) {
            current_turn_left_ = config::s03::TURN_DIRECTIONS[turn_count_];
            std::cout << "[迷宫] 🛑 第 " << (turn_count_ + 1) 
                      << "/" << config::s03::TOTAL_TURNS 
                      << " 次转弯触发，方向: " 
                      << (current_turn_left_ ? "左" : "右")
                      << "，前墙: " << wall_dist << "m" << std::endl;
            
            phase_           = Phase::TURNING;
            accumulated_yaw_ = 0.0f;
            phase_start_     = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }
        
        // 直行（侧墙警戒已禁用）
        sm->robot_driver->move(config::s03::CRUISE_VX, 0, 0);
        
        if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
            std::cout << "[迷宫][直行] front=" << wall_dist 
                      << "m (" << turn_count_ << "/" 
                      << config::s03::TOTAL_TURNS << " done)" << std::endl;
        }
        return;
    }
    
    // ===== 子阶段 2：阿克曼转弯 =====
    if (phase_ == Phase::TURNING) {
        float dt = std::chrono::duration<float>(now - phase_start_).count();
        accumulated_yaw_ = dt * config::s03::TURN_VYAW;
        
        if (accumulated_yaw_ >= config::s03::TURN_TARGET) {
            std::cout << "[迷宫] ✅ 第 " << (turn_count_ + 1) 
                      << " 次转弯完成 (耗时 " << dt << "s)" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            turn_count_++;
            phase_       = Phase::STABILIZING;
            phase_start_ = now;
            return;
        }
        
        float vyaw = config::s03::TURN_VYAW * (current_turn_left_ ? 1.0f : -1.0f);
        sm->robot_driver->move(config::s03::TURN_VX, 0, vyaw);
        
        if (++log_tick_ % 30 == 0) {
            std::cout << "[迷宫][转弯] yaw=" 
                      << (accumulated_yaw_ * 180.0f / 3.14159f) 
                      << "° / 90°" << std::endl;
        }
        return;
    }
    
    // ===== 子阶段 3：稳定 =====
    if (phase_ == Phase::STABILIZING) {
        float dt = std::chrono::duration<float>(now - phase_start_).count();
        if (dt >= config::s03::STABILIZE_AFTER_TURN) {
            std::cout << "[迷宫] ▶️ 稳定完成，继续直行" << std::endl;
            phase_       = Phase::FORWARD;
            phase_start_ = now;
            log_tick_    = 0;
        } else {
            sm->robot_driver->move(0, 0, 0);
        }
        return;
    }
    
    // ===== 子阶段 4：完成 =====
    if (phase_ == Phase::FINISHED) {
        sm->robot_driver->move(0, 0, 0);
    }
}

void State03Avoidance::exit(StateMachine* sm) {
    if (sm->robot_driver) sm->robot_driver->move(0, 0, 0);
    std::cout << "[FSM] <<< 退出 STATE_03" << std::endl;
}

} // namespace fsm
} // namespace ruikang