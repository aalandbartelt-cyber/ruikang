#include "fsm/states/state_02_start_obs.hpp"
#include "fsm/state_machine.hpp"
#include <iostream>
#include <cmath>

namespace ruikang {
namespace fsm {

void State02StartObs::enter(StateMachine* sm) {
    std::cout << "[FSM] Enter STATE_02_START_OBS: 条件积分纯转头 + 动态控速" << std::endl;
    is_jumping_ = false;
    sm->vel_ctrl.reset(); 
    current_vx_ = 0.0f; 
}

void State02StartObs::execute(StateMachine* sm) {
    if (!is_jumping_) {
        // 🌟 获取视觉偏移量
        float offset = sm->vision_data.line_offset;
        
        // 🌟🌟🌟 修复点：在这里把雷达距离提取出来！ 🌟🌟🌟
        float obs_dist = sm->vision_data.obstacle_distance;

        // 🌟 弯道动态减速（维持 0.18 底线防僵死）
        float target_vx = 0.22f; 
        if (std::abs(offset) > 80.0f) {
            target_vx = 0.18f; 
        }

        // 🌟 解决 Vx 触电抖动
        if (current_vx_ < target_vx - 0.006f) {
            current_vx_ += 0.005f; 
        } else if (current_vx_ > target_vx + 0.016f) {
            current_vx_ -= 0.015f; 
        } else {
            current_vx_ = target_vx; 
        }

        // 获取纯转头速度
        float vyaw = sm->vel_ctrl.computeYaw(offset); 

        // 🚨 极限防反向校验（狗如果往黑线外跑，解开下面这行的注释）
        // vyaw = -vyaw;

        // 起步抑制 (仅在直道起步阶段生效)
        if (current_vx_ < 0.15f && std::abs(offset) < 80.0f) {
            vyaw *= 0.5f; 
        }

        // 🌟 现在这里打印 obs_dist 就绝对不会报错了！
        std::cout << "Vx: " << current_vx_ 
                  << " | Offset: " << offset 
                  << " | Vyaw: " << vyaw 
                  << " | Radar_Dist: " << obs_dist << "m" << std::endl;

        if (sm->robot_driver) {
            // 🌟 彻底抛弃 vy，中间项固定为 0.0f
            sm->robot_driver->move(current_vx_, 0.0f, vyaw);
        }
    }
}

void State02StartObs::exit(StateMachine* sm) {
    if (sm->robot_driver) {
        sm->robot_driver->move(0.0f, 0.0f, 0.0f);
    }
}

} 
}