#include "fsm/states/state_02_start_obs.hpp"
#include "fsm/states/state_03_avoidance.hpp" 
#include "fsm/state_machine.hpp"
#include <iostream>
#include <cmath>
#include <unistd.h>

namespace ruikang {
namespace fsm {

void State02StartObs::enter(StateMachine* sm) {
    std::cout << "[FSM] >>> 进入 STATE_02: 寻迹 + 跨栏监测" << std::endl;
    is_jumping_ = false;
    current_vx_ = 0.0f;
    obs_confirm_count_ = 0; 
    startup_ignore_ticks_ = 150; // 1.5秒起步保护期
    sm->vel_ctrl.reset(); 
}

void State02StartObs::execute(StateMachine* sm) {
    if (is_jumping_) return;

    float offset = sm->vision_data.line_offset;
    float obs_dist = sm->vision_data.obstacle_distance;

    // ==========================================
    // 🌟 0. 物理预热保护期 
    // ==========================================
    if (startup_ignore_ticks_ > 0) {
        startup_ignore_ticks_--;
        obs_confirm_count_ = 0; 
        
        if (startup_ignore_ticks_ == 0) {
            std::cout << "[FSM] 🛡️ 1.5秒起步保护期结束，跨栏监测正式激活！" << std::endl;
        }
    } 
    // ==========================================
    // 🌟 1. 终极物理修复：精准架设黄金捕获网
    // ==========================================
    else {
        // 核心修改：无视 0.20m 以下的所有地面底噪！
        // 只捕捉 0.20m ~ 0.35m 这个没有任何干扰的黄金起跳区！
        if (obs_dist >= 0.10f && obs_dist < 0.30f) {
            obs_confirm_count_++; 
        } else {
            obs_confirm_count_ = 0; 
        }

        // 3 帧确信起跳
        if (obs_confirm_count_ >= 3) {
            std::cout << "[FSM] !!! 连续 3 帧锁定目标 (0.20~0.35m), 紧急制动起跳 !!!" << std::endl;
            is_jumping_ = true;

            if (sm->robot_driver) {
                sm->robot_driver->move(0.0f, 0.0f, 0.0f);
                std::cout << "[FSM] 正在刹车，等待底盘绝对静止..." << std::endl;
                
                usleep(1500000); // 1.5秒抱死刹车
                
                std::cout << "[FSM] 底盘锁定，ACTION: JUMP!" << std::endl;
                sm->robot_driver->jump();
                
                sleep(3); // 落地缓冲
                
                sm->changeState(new State03Avoidance());
                return;
            }
        }
    }

    // ==========================================
    // 2. 正常寻迹与速度控制
    // ==========================================
    float target_vx = 0.22f; 
    if (std::abs(offset) > 80.0f) {
        target_vx = 0.18f; 
    }

    if (current_vx_ < target_vx - 0.006f) {
        current_vx_ += 0.005f; 
    } else if (current_vx_ > target_vx + 0.016f) {
        current_vx_ -= 0.015f; 
    } else {
        current_vx_ = target_vx; 
    }

    float vyaw = sm->vel_ctrl.computeYaw(offset); 
    if (current_vx_ < 0.15f && std::abs(offset) < 80.0f) {
        vyaw *= 0.5f; 
    }

    // 🌟 动态日志：只要看到障碍物（计数器>0），强制逐帧打印！
    static int print_count = 0;
    if (obs_confirm_count_ > 0 || print_count++ % 10 == 0) {
        std::cout << "Vx: " << current_vx_ << " | Offset: " << offset 
                  << " | Vyaw: " << vyaw << " | Radar: " << obs_dist << "m" 
                  << (startup_ignore_ticks_ > 0 ? " [保护期屏蔽中]" : " | Confirm: " + std::to_string(obs_confirm_count_) + "/3") 
                  << std::endl;
    }

    if (sm->robot_driver) {
        sm->robot_driver->move(current_vx_, 0.0f, vyaw);
    }
}

void State02StartObs::exit(StateMachine* sm) {
    if (sm->robot_driver) {
        sm->robot_driver->move(0.0f, 0.0f, 0.0f);
    }
}

} 
}