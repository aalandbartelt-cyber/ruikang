//state_03_avoidance.cpp
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/state_machine.hpp"
#include <iostream>
#include <unistd.h>

namespace ruikang {
namespace fsm {

void State03Avoidance::enter(StateMachine* sm) {
    std::cout << "[FSM] >>> 进入 STATE_03: 迷宫避障模式 (Avoidance)" << std::endl;
    turn_count_ = 0;
    is_turning_ = false;
}

void State03Avoidance::execute(StateMachine* sm) {
    // 🌟 如果正在转弯，直接跳过雷达判定，专心把弯转完！
    if (is_turning_) return; 

    float obs_dist = sm->vision_data.obstacle_distance;

    // 🌟 1. 遇到死胡同判断 (距离小于 0.45 米，且大于 0.1 米防底噪误触)
    if (obs_dist > 0.1f && obs_dist <= 0.45f) {
        is_turning_ = true; // 上锁
        turn_count_++;
        
        std::cout << "[迷宫逻辑] 🛑 遇墙！前方距离: " << obs_dist 
                  << "m, 准备执行第 " << turn_count_ << " 次 90度原地转弯" << std::endl;

        if (sm->robot_driver) {
            // 步骤 A: 发现墙壁，瞬间刹车，防止撞墙
            sm->robot_driver->move(0.0f, 0.0f, 0.0f); 
            usleep(300000); // 停稳 0.3秒，防止带惯性漂移导致转角不准

            // 步骤 B: 根据计数器决定左转还是右转
            // 默认逻辑：奇数次左转，偶数次右转。（具体根据你们赛道的 U型 或 Z型 走法调整！）
            bool turn_left = (turn_count_ % 2 != 0); 
            
            // 步骤 C: 🌟 调用 X 同学写的底层原地 90度 转向函数
            // 注意：这个函数内部必须是“阻塞的”（转完90度才 return）
            sm->robot_driver->turn90Degree(turn_left); 

            // 步骤 D: 转弯完成后，稍微停顿一下，让雷达重新扫描前方新视野
            usleep(200000); 
        }
        
        is_turning_ = false; // 解锁，恢复直线探测模式
    } 
    // 🌟 2. 前方开阔，稳步推进
    else {
        if (sm->robot_driver) {
            // 迷宫里不能走太快，给雷达留反应时间，0.15m/s 是一个很稳健的速度
            sm->robot_driver->move(0.15f, 0.0f, 0.0f);
        }
    }
}

void State03Avoidance::exit(StateMachine* sm) {
    std::cout << "[FSM] <<< 退出 STATE_03 (Avoidance)" << std::endl;
    if (sm->robot_driver) {
        sm->robot_driver->move(0.0f, 0.0f, 0.0f); // 确保退出状态时安全停下
    }
}

} // namespace fsm
} // namespace ruikang