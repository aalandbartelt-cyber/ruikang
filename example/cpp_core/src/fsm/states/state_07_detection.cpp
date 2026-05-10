// src/fsm/states/state_07_detection.cpp
// State07: 检测平台（警示牌识别 + 指定动作执行）
//
// 流程：
//   APPROACH      → 寻迹直行，等待红点出现
//   MOVE_TO_DOT   → 红点出现后继续巡线逼近（补偿D435i前倾视角）
//   RED_DOT_ALIGN → 红点居中 → 狗投影精准覆盖检测点
//   TURN_TO_SIGN  → 原地左转 90° 面向警示牌
//   BACK_AWAY     → 后退 1s 拉开距离（离警示牌太近）
//   STOP_AND_READ → 停稳，读取 warning_sign 并映射动作
//   EXECUTE_ACTION→ ActionManager::triggerAction() 非阻塞下发
//   WAIT_ACTION   → 轮询 ActionManager::isDone()（超时兜底）
//   TURN_BACK     → 原地右转 90° 回正，面向巡线方向
//   EXIT_FOLLOW   → 继续巡线，离开检测点
// =====================================================
#include "fsm/states/state_07_detection.hpp"
#include "fsm/states/state_09_end_obs.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <string>

namespace ruikang {
namespace fsm {

void State07Detection::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_07: 检测平台" << std::endl;
    std::cout << "[FSM] 流程: APPROACH(急弯后退2.5s) → MOVE_TO_DOT(盲巡3.6s) → TURN_TO_SIGN(左90°) → BACK_AWAY(退1s) → STOP&READ → EXECUTE → WAIT → TURN_BACK(右90°) → EXIT" << std::endl;

    phase_            = Phase::APPROACH;
    action_to_play_   = "";
    accumulated_yaw_  = 0.0f;
    action_mgr_.reset(new ruikang::control::ActionManager(*sm->robot_driver));
    log_tick_         = 0;
    phase_start_      = std::chrono::steady_clock::now();
    state_enter_time_ = phase_start_;
    prev_offset_      = 0.0f;
    sharp_turn_ticks_ = 0;
    post_turn_backup_done_ = false;
}

void State07Detection::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();

    // ===== ★ 全局超时检查 =====
    float total_elapsed = std::chrono::duration<float>(now - state_enter_time_).count();
    if (total_elapsed > config::s07::TOTAL_TIMEOUT) {
        std::cout << "[FSM] ⏰ State07 全局超时 (" << total_elapsed
                  << "s)！强制退出" << std::endl;
        sm->robot_driver->move(0, 0, 0);
        sm->changeState(new State09EndObs());
        return;
    }

    float dt_phase    = std::chrono::duration<float>(now - phase_start_).count();
    float line_offset = sm->vision_data.line_offset;

    // ================================================================
    // 阶段 1：APPROACH — 寻迹直行，等待红点出现
    // ================================================================
    if (phase_ == Phase::APPROACH) {
        bool red_dot_seen = sm->vision_data.red_dot_detected;

        // 兜底超时：红点一直没出现
        if (dt_phase > config::s07::APPROACH_DURATION) {
            std::cout << "[FSM] ⏰ 红点未出现，超时兜底 (寻迹 " << dt_phase << "s) → 左转面向警示牌" << std::endl;
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        // 检测到红点 → 继续巡线逼近（补偿 D435i 前倾视角）
        if (red_dot_seen) {
            std::cout << "[FSM] 🔴 检测到红点！(cx="
                      << sm->vision_data.red_dot_center_x
                      << ") → 巡线逼近 " << config::s07::RED_DOT_FORWARD_DURATION << "s" << std::endl;
            phase_       = Phase::MOVE_TO_DOT;
            phase_start_ = now;
            return;
        }

        auto cmd = sm->vel_ctrl.getNormalTrackingVelocity(line_offset, config::s07::APPROACH_VX);
        sm->robot_driver->move(cmd.vx, cmd.vy, cmd.vyaw);

        // ★ 急弯恢复检测：|offset|>60 持续>15帧（真急弯）→ 恢复时后退
        if (std::abs(line_offset) > 60.0f) {
            sharp_turn_ticks_++;
        } else if (std::abs(line_offset) < 10.0f) {
            if (!post_turn_backup_done_ && sharp_turn_ticks_ > 15) {
                std::cout << "[FSM] 🔙 急弯恢复 (持续" << sharp_turn_ticks_
                          << "帧)，后退 2.5s 拉远视角" << std::endl;
                phase_       = Phase::POST_TURN_BACKUP;
                phase_start_ = now;
                sm->robot_driver->move(0, 0, 0);
                sharp_turn_ticks_ = 0;
                return;
            }
            sharp_turn_ticks_ = 0;
        }
        prev_offset_ = line_offset;

        if (++log_tick_ % 50 == 0) {
            std::cout << "[检测][APPROACH] " << dt_phase << "s / "
                      << config::s07::APPROACH_DURATION << "s"
                      << " offset=" << line_offset
                      << " red_dot=" << (red_dot_seen ? "YES" : "no") << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 2：POST_TURN_BACKUP — 急弯后后退 2.5s，拉开视角看清下个弯
    // ================================================================
    if (phase_ == Phase::POST_TURN_BACKUP) {
        if (dt_phase > 2.5f) {
            std::cout << "[FSM] ✅ 后退完成 → 继续巡线" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_                  = Phase::APPROACH;
            phase_start_            = now;
            post_turn_backup_done_  = true;
            prev_offset_            = 0.0f;
            sharp_turn_ticks_       = 0;
            return;
        }

        sm->robot_driver->move(-0.12f, 0, 0);

        if (++log_tick_ % 20 == 0) {
            std::cout << "[检测][BACKUP] 退后拉视角 " << dt_phase << "s / 2.5s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 3：MOVE_TO_DOT — 继续巡线逼近红点（补偿 D435i 45°前倾视角）
    // ================================================================
    if (phase_ == Phase::MOVE_TO_DOT) {
        if (dt_phase > config::s07::RED_DOT_FORWARD_DURATION) {
            std::cout << "[FSM] ✅ 盲巡逼近完成 (" << dt_phase << "s) → 左转面向警示牌" << std::endl;
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        auto cmd = sm->vel_ctrl.getNormalTrackingVelocity(line_offset, config::s07::APPROACH_VX);
        sm->robot_driver->move(cmd.vx, cmd.vy, cmd.vyaw);

        if (++log_tick_ % 30 == 0) {
            std::cout << "[检测][MOVE] 逼近红点 " << dt_phase << "s / "
                      << config::s07::RED_DOT_FORWARD_DURATION << "s"
                      << " offset=" << line_offset << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 4：RED_DOT_ALIGN — 精确定位：狗投影覆盖红点
    // ================================================================
    if (phase_ == Phase::RED_DOT_ALIGN) {
        float rx      = sm->vision_data.red_dot_center_x;
        float error_x = rx - config::s07::IMAGE_CENTER_X;

        // 对齐超时保护
        if (dt_phase > config::s07::RED_DOT_ALIGN_TIMEOUT) {
            std::cout << "[FSM] ⏰ 红点对齐超时 (" << dt_phase
                      << "s)，当前误差=" << error_x << "px，继续左转面向警示牌" << std::endl;
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        // 红点丢失（可能已经走过头）
        if (rx < 0) {
            sm->robot_driver->move(0, 0, 0);
            if (++log_tick_ % 20 == 0) {
                std::cout << "[检测][ALIGN] 红点丢失，原地等待... dt=" << dt_phase << "s" << std::endl;
            }
            return;
        }

        // 已在容忍范围内 → 左转面向警示牌！
        if (std::abs(error_x) < config::s07::RED_DOT_CENTER_TOL_PX) {
            std::cout << "[FSM] ✅ 红点对齐完成 (cx=" << rx
                      << " error=" << error_x << "px) → 左转 90° 面向警示牌" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            return;
        }

        // 闭环对齐：红点偏右 → 狗右转，偏左 → 左转
        float vyaw = (error_x > 0) ? -config::s07::RED_DOT_ALIGN_VYAW
                                   : +config::s07::RED_DOT_ALIGN_VYAW;
        sm->robot_driver->move(0, 0, vyaw);

        if (++log_tick_ % 20 == 0) {
            std::cout << "[检测][ALIGN] rx=" << rx << " error=" << error_x
                      << "px  dt=" << dt_phase << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 5：TURN_TO_SIGN — 原地左转 90° 面向警示牌
    // ================================================================
    if (phase_ == Phase::TURN_TO_SIGN) {
        float vyaw_mag   = config::s07::TURN_TO_SIGN_VYAW;
        accumulated_yaw_ = dt_phase * vyaw_mag;

        if (accumulated_yaw_ >= config::s07::TURN_TO_SIGN_TARGET) {
            std::cout << "[FSM] ✅ 左转 90° 完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°) → 后退 1s 拉开距离" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::BACK_AWAY;
            phase_start_ = now;
            return;
        }

        sm->robot_driver->move(0, 0, vyaw_mag);  // 正 vyaw = 左转

        if (++log_tick_ % 10 == 0) {
            std::cout << "[检测][TURN_SIGN] 左转中... "
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / 90°" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 6：BACK_AWAY — 后退拉开距离（离警示牌太近）
    // ================================================================
    if (phase_ == Phase::BACK_AWAY) {
        if (dt_phase > config::s07::BACK_AWAY_DURATION) {
            std::cout << "[FSM] ✅ 后退完成 (" << dt_phase << "s) → 读取警示牌" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::STOP_AND_READ;
            phase_start_ = now;
            return;
        }

        sm->robot_driver->move(config::s07::BACK_AWAY_VX, 0, 0);

        if (++log_tick_ % 20 == 0) {
            std::cout << "[检测][BACK] 后退中... " << dt_phase << "s / "
                      << config::s07::BACK_AWAY_DURATION << "s" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 7：STOP_AND_READ — 停稳，读取警示牌识别结果
    // ================================================================
    if (phase_ == Phase::STOP_AND_READ) {
        sm->robot_driver->move(0, 0, 0);

        if (dt_phase < config::s07::STOP_DURATION) {
            if (++log_tick_ % 20 == 0) {
                std::cout << "[检测][STOP] 停稳中... " << dt_phase << "s" << std::endl;
            }
            return;
        }

        // 读取警示牌
        const std::string& sign = sm->vision_data.warning_sign;
        std::cout << "[FSM] 🔍 警示牌识别结果: " << sign << std::endl;

        // 映射动作
        const char* action = config::s07::action_for_sign(sign);
        if (action) {
            action_to_play_ = action;
            std::cout << "[FSM] 🎬 映射动作: " << action_to_play_ << std::endl;
            phase_       = Phase::EXECUTE_ACTION;
            phase_start_ = now;
        } else {
            std::cout << "[FSM] ⚠️ 无匹配动作 (sign=" << sign << ")，跳过 → 右转回正" << std::endl;
            phase_            = Phase::TURN_BACK;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
        }
        return;
    }

    // ================================================================
    // 阶段 8：EXECUTE_ACTION — 调用 ActionManager 触发动作
    // ================================================================
    if (phase_ == Phase::EXECUTE_ACTION) {
        std::cout << "[FSM] 🎬 执行动作: " << action_to_play_ << std::endl;
        action_mgr_->triggerAction(action_to_play_);
        phase_       = Phase::WAIT_ACTION;
        phase_start_ = now;
        return;
    }

    // ================================================================
    // 阶段 9：WAIT_ACTION — 等待动作完成
    // ================================================================
    if (phase_ == Phase::WAIT_ACTION) {
        sm->robot_driver->move(0, 0, 0);

        // ★ ActionManager 非阻塞轮询：动作完成立即离开
        if (!action_mgr_->isDone()) {
            // 兜底超时保护
            if (dt_phase > config::s07::ACTION_TIMEOUT) {
                std::cout << "[FSM] ⏰ 动作超时 (" << dt_phase << "s)，右转回正" << std::endl;
                phase_            = Phase::TURN_BACK;
                accumulated_yaw_  = 0.0f;
                phase_start_      = now;
                return;
            }

            if (++log_tick_ % 30 == 0) {
                std::cout << "[检测][WAIT] 动作进行中: " << action_to_play_
                          << " " << dt_phase << "s" << std::endl;
            }
            return;
        }

        std::cout << "[FSM] ✅ 动作完成: " << action_to_play_ << " → 右转 90° 回正" << std::endl;
        phase_            = Phase::TURN_BACK;
        accumulated_yaw_  = 0.0f;
        phase_start_      = now;
        return;
    }

    // ================================================================
    // 阶段 10：TURN_BACK — 原地右转 90° 回正，面向巡线方向
    // ================================================================
    if (phase_ == Phase::TURN_BACK) {
        float vyaw_mag   = config::s07::TURN_BACK_VYAW;
        accumulated_yaw_ = dt_phase * vyaw_mag;

        if (accumulated_yaw_ >= config::s07::TURN_BACK_TARGET) {
            std::cout << "[FSM] ✅ 右转 90° 回正完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°) → 继续巡线" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_       = Phase::EXIT_FOLLOW;
            phase_start_ = now;
            log_tick_    = 0;
            return;
        }

        sm->robot_driver->move(0, 0, -vyaw_mag);  // 负 vyaw = 右转

        if (++log_tick_ % 10 == 0) {
            std::cout << "[检测][TURN_BACK] 右转中... "
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / 90°" << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 11：EXIT_FOLLOW — 继续巡线，离开检测点
    // ================================================================
    if (phase_ == Phase::EXIT_FOLLOW) {
        if (dt_phase > config::s07::EXIT_FOLLOW_DURATION) {
            std::cout << "[FSM] 🎉 检测平台完成！退出 State07" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State09EndObs());
            return;
        }

        auto cmd = sm->vel_ctrl.getNormalTrackingVelocity(line_offset, config::s07::EXIT_FOLLOW_VX);
        sm->robot_driver->move(cmd.vx, cmd.vy, cmd.vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[检测][EXIT] 离开检测点 " << dt_phase << "s / "
                      << config::s07::EXIT_FOLLOW_DURATION << "s"
                      << " offset=" << line_offset << std::endl;
        }
        return;
    }

    if (phase_ == Phase::FINISHED) {
        sm->robot_driver->move(0, 0, 0);
    }
}

void State07Detection::exit(StateMachine* sm) {
    if (sm->robot_driver) {
        sm->robot_driver->move(0, 0, 0);
    }
    action_mgr_.reset();
    std::cout << "[FSM] <<< 退出 STATE_07" << std::endl;
}

} // namespace fsm
} // namespace ruikang
