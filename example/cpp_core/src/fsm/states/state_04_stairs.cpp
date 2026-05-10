// src/fsm/states/state_04_stairs.cpp
// State04: 上下台阶（ArUco 辅助定位 + 步态切换）
//
// V5.10 改动：
//   - 三段式上下台阶：直爬1s → 弧线左转90° → 直下1s
//   - 巡线离开后先巡3s再切回经典步态
//   - ArUco center_y 阈值判断靠近
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
    std::cout << "[FSM] 流程: APPROACH(cy>" << config::s04::ARUCO_NEAR_Y_THRESHOLD
              << ") → ALIGN → 直爬" << config::s04::CLIMB_UP_DURATION
              << "s → 弧线" << (config::s04::CLIMB_ARC_TARGET * 180.0f / 3.14159f)
              << "° → 直下" << config::s04::CLIMB_DOWN_DURATION
              << "s → EXIT(巡" << config::s04::GAIT_SWITCH_DELAY << "s后切步态)" << std::endl;

    phase_            = Phase::APPROACH;
    accumulated_yaw_  = 0.0f;
    gait_switched_    = false;
    log_tick_         = 0;
    phase_start_      = std::chrono::steady_clock::now();
    state_enter_time_ = phase_start_;
}

void State04Stairs::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();

    // ===== 全局超时检查 =====
    float total_elapsed = std::chrono::duration<float>(now - state_enter_time_).count();
    if (total_elapsed > config::s04::TOTAL_TIMEOUT) {
        std::cout << "[FSM] ⏰ State04 全局超时 (" << total_elapsed
                  << "s)！强制退出" << std::endl;
        sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_CLASSIC);
        sm->robot_driver->move(0, 0, 0);
        sm->changeState(new State07Detection());
        return;
    }

    float dt_phase      = std::chrono::duration<float>(now - phase_start_).count();
    float line_offset   = sm->vision_data.line_offset;
    bool  aruco_detected = sm->vision_data.aruco_detected;
    float aruco_cy      = sm->vision_data.aruco_center_y;

    // ================================================================
    // 阶段 1：APPROACH — 巡线直行，ArUco center_y 判断是否靠近台阶
    // ================================================================
    if (phase_ == Phase::APPROACH) {
        if (aruco_detected && aruco_cy > config::s04::ARUCO_NEAR_Y_THRESHOLD) {
            std::cout << "[FSM] 🎯 ArUco已靠近 (cy=" << aruco_cy
                      << " > " << config::s04::ARUCO_NEAR_Y_THRESHOLD << ") → 对齐台阶" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::ALIGN_ARUCO;
            phase_start_ = now;
            return;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s04::APPROACH_VX, 0, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[台阶][APPROACH] offset=" << line_offset
                      << " aruco=" << (aruco_detected ? "YES" : "no");
            if (aruco_detected)
                std::cout << " cy=" << aruco_cy << "/" << config::s04::ARUCO_NEAR_Y_THRESHOLD;
            std::cout << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 2：ALIGN_ARUCO — 对齐台阶中心
    // ================================================================
    if (phase_ == Phase::ALIGN_ARUCO) {
        float cx      = sm->vision_data.aruco_center_x;
        float error_x = cx - config::s04::IMAGE_CENTER_X;

        if (dt_phase > config::s04::ALIGN_TIMEOUT) {
            std::cout << "[FSM] ⏰ 对齐超时 (" << dt_phase << "s)，直接爬台阶" << std::endl;
            phase_            = Phase::CLIMB_UP;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        if (cx < 0) {
            sm->robot_driver->move(0, 0, 0);
        } else if (std::abs(error_x) < config::s04::ALIGN_CENTER_TOL_PX) {
            std::cout << "[FSM] ✅ ArUco 对齐完成 (cx=" << cx
                      << " error=" << error_x << "px) → 直爬" << config::s04::CLIMB_UP_DURATION << "s" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::CLIMB_UP;
            phase_start_ = now;
            return;
        } else {
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
    // 阶段 3：CLIMB_UP — 直爬上台阶
    // ================================================================
    if (phase_ == Phase::CLIMB_UP) {
        if (dt_phase > config::s04::CLIMB_UP_DURATION) {
            std::cout << "[FSM] ✅ 直爬完成 (" << dt_phase << "s) → 弧线左转"
                      << (config::s04::CLIMB_ARC_TARGET * 180.0f / 3.14159f) << "°" << std::endl;
            phase_            = Phase::CLIMB_ARC;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::CLIMB_UP_VX, 0, 0);

        if (++log_tick_ % 20 == 0) {
            std::cout << "[台阶][CLIMB_UP] " << dt_phase << "s / "
                      << config::s04::CLIMB_UP_DURATION << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 4：CLIMB_ARC — 弧线前进 + 左转 90°
    // ================================================================
    if (phase_ == Phase::CLIMB_ARC) {
        accumulated_yaw_ = dt_phase * config::s04::CLIMB_ARC_VYAW;

        if (accumulated_yaw_ >= config::s04::CLIMB_ARC_TARGET) {
            std::cout << "[FSM] ✅ 弧线完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°) → 直下"
                      << config::s04::CLIMB_DOWN_DURATION << "s" << std::endl;
            phase_       = Phase::CLIMB_DOWN;
            phase_start_ = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::CLIMB_ARC_VX, 0, config::s04::CLIMB_ARC_VYAW);

        if (++log_tick_ % 15 == 0) {
            std::cout << "[台阶][CLIMB_ARC] "
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / "
                      << (config::s04::CLIMB_ARC_TARGET * 180.0f / 3.14159f) << "°" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 5：CLIMB_DOWN — 直下台阶
    // ================================================================
    if (phase_ == Phase::CLIMB_DOWN) {
        if (dt_phase > config::s04::CLIMB_DOWN_DURATION) {
            std::cout << "[FSM] ✅ 直下完成 → 巡线离开（" << config::s04::GAIT_SWITCH_DELAY << "s后切回经典）" << std::endl;
            phase_       = Phase::EXIT_FOLLOW;
            phase_start_ = now;
            gait_switched_ = false;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        sm->robot_driver->move(config::s04::CLIMB_DOWN_VX, 0, 0);

        if (++log_tick_ % 20 == 0) {
            std::cout << "[台阶][CLIMB_DOWN] " << dt_phase << "s / "
                      << config::s04::CLIMB_DOWN_DURATION << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 6：EXIT_FOLLOW — 巡线离开（先巡几秒再切步态）
    // ================================================================
    if (phase_ == Phase::EXIT_FOLLOW) {
        // 巡线满 GAIT_SWITCH_DELAY 秒后切回经典步态
        if (!gait_switched_ && dt_phase > config::s04::GAIT_SWITCH_DELAY) {
            std::cout << "[FSM] 🦿 巡线" << config::s04::GAIT_SWITCH_DELAY << "s → 步态恢复: 灵动 → 经典" << std::endl;
            sm->robot_driver->setGait(ruikang::control::GaitType::GAIT_CLASSIC);
            gait_switched_ = true;
        }

        if (dt_phase > config::s04::EXIT_FOLLOW_DURATION) {
            std::cout << "[FSM] 🎉 台阶区完成！退出 State04" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State07Detection());
            return;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s04::EXIT_FOLLOW_VX, 0, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[台阶][EXIT] 巡线 " << dt_phase << "s / "
                      << config::s04::EXIT_FOLLOW_DURATION << "s"
                      << " 步态=" << (gait_switched_ ? "经典" : "灵动")
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
