// src/fsm/states/state_04_stairs.cpp
// State04: 上下台阶（ArUco 辅助定位 + 步态切换）
// 翻越式：上台阶 → 顶部左转 90° → 下台阶 → 继续寻迹
//
// vs 骨架版的修正：
//   1. 加全局超时检查（防卡死）
//   2. ALIGN_ARUCO 闭环对齐（aruco_center_x）
//   3. SWITCH_GAIT 已接通 RobotDriver::setGait()
//   4. 新增 TOP_TURN_LEFT（上完台阶后左转 90°）
// =====================================================
#include "fsm/states/state_04_stairs.hpp"
#include "fsm/states/state_07_detection.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <cmath>

namespace ruikang {
namespace fsm {

void State04Stairs::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_04: 台阶区" << std::endl;
    std::cout << "[FSM] 流程: APPROACH → ALIGN → 经典→灵动 → 上台阶 → 顶部左转90° → 下台阶 → 灵动→经典 → EXIT" << std::endl;

    phase_            = Phase::APPROACH;
    accumulated_yaw_  = 0.0f;
    log_tick_         = 0;
    phase_start_      = std::chrono::steady_clock::now();
    state_enter_time_ = phase_start_;
}

void State04Stairs::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();

    // ===== ★ 修复点 1：全局超时检查 =====
    float total_elapsed = std::chrono::duration<float>(now - state_enter_time_).count();
    if (total_elapsed > config::s04::TOTAL_TIMEOUT) {
        std::cout << "[FSM] ⏰ State04 全局超时 (" << total_elapsed
                  << "s)！强制退出，确保后续任务不卡死" << std::endl;
        sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_CLASSIC);
        sm->robot_driver->move(0, 0, 0);
        sm->changeState(new State07Detection());
        return;
    }

    float dt_phase      = std::chrono::duration<float>(now - phase_start_).count();
    float line_offset   = sm->vision_data.line_offset;
    bool  aruco_detected = sm->vision_data.aruco_detected;
    float depth_front   = sm->vision_data.depth_front;

    // ================================================================
    // 阶段 1：APPROACH — 寻迹直行，等待 ArUco
    // ================================================================
    if (phase_ == Phase::APPROACH) {
        if (aruco_detected) {
            std::cout << "[FSM] 🎯 检测到 ArUco！进入对齐" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::ALIGN_ARUCO;
            phase_start_ = now;
            return;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s04::APPROACH_VX, 0, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[台阶][APPROACH] offset=" << line_offset
                      << " aruco=" << (aruco_detected ? "YES" : "no") << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 2：ALIGN_ARUCO — 对齐台阶中心
    // ================================================================
    if (phase_ == Phase::ALIGN_ARUCO) {
        float cx      = sm->vision_data.aruco_center_x;
        float error_x = cx - config::s04::IMAGE_CENTER_X;

        // 超时保护
        if (dt_phase > config::s04::ALIGN_TIMEOUT) {
            std::cout << "[FSM] ⏰ 对齐超时 (" << dt_phase
                      << "s)，进入步态切换" << std::endl;
            phase_       = Phase::SWITCH_GAIT_UP;
            phase_start_ = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        if (cx < 0) {  // -1 = 当前帧未检测到 ArUco，原地等待
            sm->robot_driver->move(0, 0, 0);
        } else if (std::abs(error_x) < config::s04::ALIGN_CENTER_TOL_PX) {
            std::cout << "[FSM] ✅ ArUco 对齐完成 (cx=" << cx
                      << " error=" << error_x << "px)" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::SWITCH_GAIT_UP;
            phase_start_ = now;
            return;
        } else {
            // ArUco 偏右 (error_x > 0) → 狗需右转 (vyaw 为负，取决于坐标系)
            float vyaw = (error_x > 0) ? -config::s04::ALIGN_VYAW
                                       : +config::s04::ALIGN_VYAW;
            sm->robot_driver->move(0, 0, vyaw);
        }

        if (++log_tick_ % 30 == 0) {
            std::cout << "[台阶][ALIGN] cx=" << cx << " error=" << error_x
                      << "px  dt=" << dt_phase << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 3：SWITCH_GAIT_UP — 经典 → 灵动
    // ================================================================
    if (phase_ == Phase::SWITCH_GAIT_UP) {
        std::cout << "[FSM] 🦿 步态切换: 经典 → 灵动" << std::endl;
        sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_AGILE);
        phase_       = Phase::CLIMB_UP;
        phase_start_ = now;
        sm->robot_driver->move(0, 0, 0);
        return;
    }

    // ================================================================
    // 阶段 4：CLIMB_UP — 上台阶
    // ================================================================
    if (phase_ == Phase::CLIMB_UP) {
        if (dt_phase > config::s04::CLIMB_UP_DURATION) {
            std::cout << "[FSM] ✅ 上台阶完成 (" << dt_phase << "s)" << std::endl;
            phase_            = Phase::TOP_TURN_LEFT;  // ★ → 顶部左转 90°
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::CLIMB_VX, 0, 0);

        if (++log_tick_ % 30 == 0) {
            std::cout << "[台阶][CLIMB_UP] " << dt_phase << "s / "
                      << config::s04::CLIMB_UP_DURATION << "s"
                      << "  DF=" << depth_front << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 5：TOP_TURN_LEFT — 台阶顶部左转 90°
    // ================================================================
    if (phase_ == Phase::TOP_TURN_LEFT) {
        float vyaw_mag    = config::s04::TOP_TURN_VYAW;
        accumulated_yaw_  = dt_phase * vyaw_mag;  // dt 方式，与 state_03 一致

        if (accumulated_yaw_ >= config::s04::TOP_TURN_TARGET) {
            std::cout << "[FSM] ✅ 顶部左转 90° 完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°)" << std::endl;
            phase_       = Phase::CLIMB_DOWN;
            phase_start_ = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::TOP_TURN_VX, 0, vyaw_mag);

        if (++log_tick_ % 10 == 0) {
            std::cout << "[台阶][TOP_TURN] 左转中... "
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / 90°" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 6：CLIMB_DOWN — 下台阶
    // ================================================================
    if (phase_ == Phase::CLIMB_DOWN) {
        if (dt_phase > config::s04::CLIMB_DOWN_DURATION) {
            std::cout << "[FSM] ✅ 下台阶完成 (" << dt_phase << "s)" << std::endl;
            phase_       = Phase::SWITCH_GAIT_DOWN;
            phase_start_ = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::CLIMB_VX, 0, 0);

        if (++log_tick_ % 30 == 0) {
            std::cout << "[台阶][CLIMB_DOWN] " << dt_phase << "s / "
                      << config::s04::CLIMB_DOWN_DURATION << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 7：SWITCH_GAIT_DOWN — 灵动 → 经典
    // ================================================================
    if (phase_ == Phase::SWITCH_GAIT_DOWN) {
        std::cout << "[FSM] 🦿 步态恢复: 灵动 → 经典" << std::endl;
        sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_CLASSIC);
        phase_       = Phase::EXIT_FOLLOW;
        phase_start_ = now;
        sm->robot_driver->move(0, 0, 0);
        return;
    }

    // ================================================================
    // 阶段 8：EXIT_FOLLOW — 继续寻迹，离开台阶区域
    // ================================================================
    if (phase_ == Phase::EXIT_FOLLOW) {
        if (dt_phase > config::s04::EXIT_FOLLOW_DURATION) {
            std::cout << "[FSM] 🎉 台阶区完成！退出 State04" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State07Detection());
            return;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s04::EXIT_FOLLOW_VX, 0, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[台阶][EXIT] 寻迹离开 " << dt_phase << "s / "
                      << config::s04::EXIT_FOLLOW_DURATION << "s"
                      << " offset=" << line_offset << std::endl;
        }
        return;
    }

    if (phase_ == Phase::FINISHED) {
        sm->robot_driver->move(0, 0, 0);
    }
}

void State04Stairs::exit(StateMachine* sm) {
    if (sm->robot_driver) {
        sm->robot_driver->move(0, 0, 0);
        sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_CLASSIC);
    }
    std::cout << "[FSM] <<< 退出 STATE_04" << std::endl;
}

} // namespace fsm
} // namespace ruikang
