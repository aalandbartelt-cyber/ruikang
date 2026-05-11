// src/fsm/states/state_07_detection.cpp
// State07: 检测平台（警示牌识别 + 指定动作执行）
//
// 流程：
//   前半段（180°掉头前）：高速（0.37）+ 强弯中增强（×2.54），过普通直角弯 + 双急弯
//   后半段（180°掉头后）：稳定（0.18）+ 弱弯中增强（×1.5），正常巡线到红点
//   MOVE_TO_DOT   → 红点出现后继续巡线逼近（补偿D435i前倾视角）
//   TURN_TO_SIGN  → 原地左转 90° 面向警示牌
//   BACK_AWAY     → 后退 1s 拉开距离
//   STOP_AND_READ → 停稳，读取 warning_sign 并映射动作
//   EXECUTE_ACTION→ ActionManager::triggerAction() 非阻塞下发
//   WAIT_ACTION   → 轮询 ActionManager::isDone()（超时兜底）
//   TURN_BACK     → 原地右转 90° 回正
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
    std::cout << "[FSM] 流程: 前半段高速(" << config::s07::APPROACH_VX
              << "m/s,×2.54过弯) → 平台检测→TURN_180 → 后半段稳定("
              << config::s07::AFTER180_VX
              << "m/s,×1.5过弯) → MOVE_TO_DOT(盲巡" << config::s07::RED_DOT_FORWARD_DURATION
              << "s) → TURN_TO_SIGN(左90°) → BACK_AWAY(退"
              << config::s07::BACK_AWAY_DURATION << "s) → STOP&READ → EXECUTE → WAIT → TURN_BACK(右90°) → EXIT" << std::endl;

    phase_            = Phase::APPROACH;
    action_to_play_   = "";
    accumulated_yaw_  = 0.0f;
    action_mgr_.reset(new ruikang::control::ActionManager(*sm->robot_driver));
    sm->vel_ctrl.reset();
    log_tick_         = 0;
    was_in_turn_          = false;
    post_turn_boost_      = 0;
    boost_dir_            = 0.0f;
    after_turn_180_       = false;
    platform_confirm_cnt_ = 0;
    phase_start_          = std::chrono::steady_clock::now();
    state_enter_time_ = phase_start_;
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
    // ★ 弯中增强 + 弯后boost：弯中vyaw×1.5确保普通直角弯转够，弯后×1.5持续0.5s捕捉连续弯
    // ================================================================
    if (phase_ == Phase::APPROACH) {
        // ===== ★ 平台深度检测（连续帧确认，防噪声漏判） =====
        // 黑色平台吸收红外光 → D435i 可能返回 0 或极低值
        // 去掉 >0.10 的最低有效限制，让 depth=0 也能触发
        // 仅保留 <9.0 防传感器异常大值
        float d = sm->vision_data.depth_front;
        bool sees_platform = (d < 9.0f) && (d < config::s07::OBSTACLE_TRIGGER_DIST);

        if (sees_platform) {
            platform_confirm_cnt_++;
        } else {
            platform_confirm_cnt_ = 0;  // 一旦丢失立即清零
        }

        // 调试：每10帧打印深度值，方便现场观察
        if (log_tick_ % 10 == 0) {
            std::cout << "[DEBUG] depth_front=" << d
                      << " valid=" << (d > 0.10f && d < 9.0f ? "Y" : "N")
                      << " plat_cnf=" << platform_confirm_cnt_ << std::endl;
        }

        if (platform_confirm_cnt_ >= config::s07::PLATFORM_CONFIRM_FRAMES) {
            std::cout << "[FSM] ⚠️ 检测到平台！距离=" << sm->vision_data.depth_front
                      << "m (连续" << platform_confirm_cnt_ << "帧) → 紧急180°掉头!" << std::endl;
            phase_               = Phase::TURN_180;
            accumulated_yaw_     = 0.0f;
            platform_confirm_cnt_ = 0;
            phase_start_         = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        bool red_dot_seen = sm->vision_data.red_dot_detected;

        // 兜底超时：红点一直没出现
        if (dt_phase > config::s07::APPROACH_DURATION) {
            std::cout << "[FSM] ⏰ 红点未出现，超时兜底 (巡线 " << dt_phase << "s) → 左转面向警示牌" << std::endl;
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

        // ===== 弯后boost逻辑 =====
        if (post_turn_boost_ > 0) {
            post_turn_boost_--;
        }

        // 检测是否进入急弯 → 记录方向
        if (std::abs(line_offset) > 60.0f) {
            was_in_turn_ = true;
            boost_dir_   = (line_offset > 0) ? 1.0f : -1.0f;
        }
        // 弯结束：offset回到20以内 → 清turn_memory（防偏右撞台阶）+ 启动boost
        if (was_in_turn_ && std::abs(line_offset) < 20.0f) {
            was_in_turn_ = false;
            sm->vel_ctrl.turn_memory = 0.0f;
            sm->vel_ctrl.error_sum   = 0.0f;
            post_turn_boost_ = 50;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);

        if (after_turn_180_) {
            // 后半段：稳定巡线，弱增强
            if (std::abs(line_offset) > 60.0f) {
                vyaw *= 1.5f;
            } else if (post_turn_boost_ > 0) {
                vyaw *= 1.5f;
            }
        } else {
            // 前半段：高速过弯，强增强（普通直角弯+双急弯）
            if (std::abs(line_offset) > 50.0f) {
                vyaw *= 2.54f;
            } else if (post_turn_boost_ > 0) {
                vyaw *= 1.5f;
            }
        }

        float current_vx = after_turn_180_ ? config::s07::AFTER180_VX
                                          : config::s07::APPROACH_VX;
        sm->robot_driver->move(current_vx, 0, vyaw);

        if (++log_tick_ % 50 == 0) {
            std::cout << "[检测][APPROACH] " << dt_phase << "s / "
                      << config::s07::APPROACH_DURATION << "s"
                      << " offset=" << line_offset
                      << " boost=" << (post_turn_boost_ > 0 ? "ON" : "off")
                      << " red_dot=" << (red_dot_seen ? "YES" : "no") << std::endl;
        }
        return;
    }
    // =====================================================
    // ★ 阶段：TURN_180 — 遇到平台边走边转180°掉头（角度累积）
    // =====================================================
    if (phase_ == Phase::TURN_180) {
        float vyaw_mag   = config::s07::TURN_180_VYAW;
        accumulated_yaw_ = dt_phase * vyaw_mag;

        if (accumulated_yaw_ >= config::s07::TURN_180_TARGET) {
            std::cout << "[FSM] 🔄 180度掉头完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°) → 切换后半段稳定模式(vx="
                      << config::s07::AFTER180_VX << ")" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            // ★ 掉头后重置巡线状态 + 切为稳定模式
            sm->vel_ctrl.reset();
            was_in_turn_      = false;
            post_turn_boost_  = 0;
            boost_dir_        = 0.0f;
            after_turn_180_   = true;
            platform_confirm_cnt_ = 0;
            phase_       = Phase::APPROACH;
            phase_start_ = now;
            return;
        }

        sm->robot_driver->move(config::s07::TURN_180_VX, 0, vyaw_mag);  // 边走边转

        if (++log_tick_ % 10 == 0) {
            std::cout << "[检测][TURN_180] 掉头中... "
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "° / 180°" << std::endl;
        }
        return;
    }
    // ================================================================
    // 阶段 2：MOVE_TO_DOT — 继续巡线逼近红点（补偿 D435i 45°前倾视角）
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

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s07::AFTER180_VX, 0, vyaw);

        if (++log_tick_ % 30 == 0) {
            std::cout << "[检测][MOVE] 逼近红点 " << dt_phase << "s / "
                      << config::s07::RED_DOT_FORWARD_DURATION << "s"
                      << " offset=" << line_offset << std::endl;
        }
        return;
    }

    // ================================================================
    // 阶段 3：RED_DOT_ALIGN — 精确定位：狗投影覆盖红点
    // ================================================================
    if (phase_ == Phase::RED_DOT_ALIGN) {
        float rx      = sm->vision_data.red_dot_center_x;
        float error_x = rx - config::s07::IMAGE_CENTER_X;

        if (dt_phase > config::s07::RED_DOT_ALIGN_TIMEOUT) {
            std::cout << "[FSM] ⏰ 红点对齐超时 (" << dt_phase
                      << "s)，当前误差=" << error_x << "px，继续左转面向警示牌" << std::endl;
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        if (rx < 0) {
            sm->robot_driver->move(0, 0, 0);
            if (++log_tick_ % 20 == 0) {
                std::cout << "[检测][ALIGN] 红点丢失，原地等待... dt=" << dt_phase << "s" << std::endl;
            }
            return;
        }

        if (std::abs(error_x) < config::s07::RED_DOT_CENTER_TOL_PX) {
            std::cout << "[FSM] ✅ 红点对齐完成 (cx=" << rx
                      << " error=" << error_x << "px) → 左转 90° 面向警示牌" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            phase_            = Phase::TURN_TO_SIGN;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            return;
        }

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
    // 阶段 4：TURN_TO_SIGN — 原地左转 90° 面向警示牌
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
    // 阶段 5：BACK_AWAY — 后退拉开距离（离警示牌太近）
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
    // 阶段 6：STOP_AND_READ — 停稳，读取警示牌识别结果
    // ================================================================
    if (phase_ == Phase::STOP_AND_READ) {
        sm->robot_driver->move(0, 0, 0);

        if (dt_phase < config::s07::STOP_DURATION) {
            if (++log_tick_ % 20 == 0) {
                std::cout << "[检测][STOP] 停稳中... " << dt_phase << "s" << std::endl;
            }
            return;
        }

        const std::string& sign = sm->vision_data.warning_sign;
        std::cout << "[FSM] 🔍 警示牌识别结果: " << sign << std::endl;

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
    // 阶段 7：EXECUTE_ACTION — 调用 ActionManager 触发动作
    // ================================================================
    if (phase_ == Phase::EXECUTE_ACTION) {
        std::cout << "[FSM] 🎬 执行动作: " << action_to_play_ << std::endl;
        action_mgr_->triggerAction(action_to_play_);
        phase_       = Phase::WAIT_ACTION;
        phase_start_ = now;
        return;
    }

    // ================================================================
    // 阶段 8：WAIT_ACTION — 等待动作完成
    // ================================================================
    if (phase_ == Phase::WAIT_ACTION) {
        sm->robot_driver->move(0, 0, 0);

        if (!action_mgr_->isDone()) {
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
    // 阶段 9：TURN_BACK — 原地右转 90° 回正，面向巡线方向
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
    // 阶段 10：EXIT_FOLLOW — 继续巡线，离开检测点
    // ================================================================
    if (phase_ == Phase::EXIT_FOLLOW) {
        if (dt_phase > config::s07::EXIT_FOLLOW_DURATION) {
            std::cout << "[FSM] 🎉 检测平台完成！退出 State07" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State09EndObs());
            return;
        }

        float vyaw = sm->vel_ctrl.computeYaw(line_offset);
        sm->robot_driver->move(config::s07::EXIT_FOLLOW_VX, 0, vyaw);

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
