// src/fsm/states/state_03_avoidance.cpp
// V2.0（方案三）：基于 D435i 深度相机的避障逻辑
//
// 物理基线：D435i 前倾约 45°
//   - 无障碍时：看到地面 ≈ 0.66m
//   - 有挡板：距离突然 < 0.50m
//
// 触发条件：depth_front < TURN_TRIGGER_DIST
//          且持续 CONFIRM_FRAMES 帧（防抖）
//
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/states/state_04_stairs.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <cmath>

namespace ruikang {
namespace fsm {

void State03Avoidance::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_03: 避障区（D435i 深度方案）" << std::endl;
    std::cout << "[FSM] 转弯顺序: ";
    for (int i = 0; i < config::s03::TOTAL_TURNS; ++i) {
        std::cout << (config::s03::TURN_DIRECTIONS[i] ? "左 " : "右 ");
    }
    std::cout << std::endl;

    // ★ 已经换回单一触发距离
    std::cout << "[FSM] 触发距离: " << config::s03::TURN_TRIGGER_DIST
              << "m (连续 " << config::s03::CONFIRM_FRAMES << " 帧)" << std::endl;

    phase_           = Phase::FORWARD;
    turn_count_      = 0;
    accumulated_yaw_ = 0.0f;
    confirm_ticks_   = 0;
    log_tick_        = 0;
    phase_start_     = std::chrono::steady_clock::now();
}

void State03Avoidance::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();

    // ★ 数据源：D435i 深度相机
    float depth_front = sm->vision_data.depth_front;
    float depth_left  = sm->vision_data.depth_left;
    float depth_right = sm->vision_data.depth_right;

    // ===== 子阶段 1：直行 + 触发判定 =====
    if (phase_ == Phase::FORWARD) {

        // 完成所有转弯，退出
        if (turn_count_ >= config::s03::TOTAL_TURNS) {
            std::cout << "[迷宫] 🎉 完成 " << config::s03::TOTAL_TURNS
                      << " 次转弯，退出 State03" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State04Stairs());
            return;
        }

        // 常规触发判定
        bool valid_data = (depth_front >= config::s03::DEPTH_MIN_VALID
                        && depth_front <= config::s03::DEPTH_MAX_VALID);
        bool below_trigger = valid_data
                         && depth_front < config::s03::TURN_TRIGGER_DIST;

        // ★★★ 核心修改：特定转弯完成后的"盲走"逻辑
        float blind_walk_time = 0.0f;

        if (turn_count_ == 2) {
            // 前 4 次（全是左转）完成后，准备进入第 5 次（第一个右转）前
            blind_walk_time = 0.1f;  // 保留你之前调好的 1.9 秒
        } else if (turn_count_ == 3) {
            // ★ 新增：第 5 次（也就是第一个右转）完成后，准备进入第 6 次前
            blind_walk_time = 0.3f;  // 往前盲走 1.0 秒
        }

        // 如果当前阶段需要盲走
        if (blind_walk_time > 0.0f) {
            // 计算进入直行状态后经过的时间
            float dt_forward = std::chrono::duration<float>(now - phase_start_).count();

            if (dt_forward < blind_walk_time) {
                below_trigger = false; // 强行关闭触发（装作没看见墙）

                if (config::debug::VERBOSE_LOG && (log_tick_ % 50 == 0)) {
                    std::cout << "[迷宫] 盲走中... 忽略避障 ("
                              << dt_forward << "s / " << blind_walk_time << "s)" << std::endl;
                }
            }
        }

        if (below_trigger) {
            confirm_ticks_++;
            if (confirm_ticks_ >= config::s03::CONFIRM_FRAMES) {
                // 触发转弯
                current_turn_left_ = config::s03::TURN_DIRECTIONS[turn_count_];
                std::cout << "[迷宫] 🛑 第 " << (turn_count_ + 1) << "/"
                          << config::s03::TOTAL_TURNS << " 次转弯！"
                          << " 方向: " << (current_turn_left_ ? "左" : "右")
                          << " 前方距离: " << depth_front << "m"
                          << " 目标角度: " << (config::s03::TURN_TARGETS[turn_count_] * 180.0f / 3.14159f) << "°"
                          << " (连续 " << confirm_ticks_ << " 帧确认)"
                          << std::endl;

                phase_           = Phase::TURNING;
                accumulated_yaw_ = 0.0f;
                phase_start_     = now;
                confirm_ticks_   = 0;
                sm->robot_driver->move(0, 0, 0);
                return;
            }
        } else {
            confirm_ticks_ = 0;  // 重置
        }

        // 没触发，继续直行
        sm->robot_driver->move(config::s03::CRUISE_VX, 0, 0);

        // 调试打印（每 50 帧 = 0.5s）
        if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
            std::cout << "[迷宫][直行] depth_front=" << depth_front
                      << "m  L=" << depth_left
                      << "m  R=" << depth_right
                      << "  确认计数=" << confirm_ticks_
                      << "/" << config::s03::CONFIRM_FRAMES
                      << "  (" << turn_count_ << "/"
                      << config::s03::TOTAL_TURNS << " done)"
                      << std::endl;
        }
        return;
    }

    // ===== 子阶段 2：阿克曼转弯（边走边转） =====
    if (phase_ == Phase::TURNING) {
        float dt = std::chrono::duration<float>(now - phase_start_).count();
        accumulated_yaw_ = dt * config::s03::TURN_VYAW;
        if (accumulated_yaw_ >= config::s03::TURN_TARGETS[turn_count_]) {
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
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / "
                      << (config::s03::TURN_TARGETS[turn_count_] * 180.0f / 3.14159f) << "°"
                      << "  depth_front=" << depth_front << "m"
                      << std::endl;
        }
        return;
    }

    // ===== 子阶段 3：稳定 =====
    if (phase_ == Phase::STABILIZING) {
        float dt = std::chrono::duration<float>(now - phase_start_).count();

        // ★ 核心修改：动态读取刚才那次转弯专属的缓冲时间
        // 注意：因为在阶段 2 转弯结束时，turn_count_ 已经执行了 +1，
        // 所以当前检查的是第 (turn_count_ - 1) 次转弯后的稳定时间。
        float target_wait_time = config::s03::STABILIZE_TIMES[turn_count_ - 1];

        if (dt >= target_wait_time) {
            std::cout << "[迷宫] ▶️ 缓冲完成 (" << target_wait_time << "s)，重心已恢复，继续直行" << std::endl;
            phase_         = Phase::FORWARD;
            phase_start_   = now;
            log_tick_      = 0;
            confirm_ticks_ = 0;
        } else {
            // 持续发送 0 速度，让底层算法原地踏步调平姿态
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
