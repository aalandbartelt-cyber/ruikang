// src/fsm/states/state_03_avoidance.cpp
// V3.2：方案 A（深度感知优化版）
//
// 关键设计：
//   1. 深度 5 帧移动平均：抗深度跳变
//   2. 侧向开口三道门禁：前向距离 + 直行时间 + 连续帧确认
//   3. 方向感知姿态校正：每次转弯后根据下次转向决定偏移
//   4. 入口对齐：State02 5° 粗对齐 + State03 闭环精对齐
// =====================================================
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/states/state_04_stairs.hpp"
#include "fsm/state_machine.hpp"
#include "common/config.hpp"
#include <iostream>
#include <cmath>

namespace ruikang {
namespace fsm {

// ============================================================
// 辅助函数
// ============================================================
bool State03Avoidance::isDepthValid(float d) {
    return d >= config::s03::DEPTH_MIN_VALID && d <= config::s03::DEPTH_MAX_VALID;
}

float State03Avoidance::getSmoothedDepth() {
    float sum = 0.0f;
    int count = 0;
    int window = (depth_history_fill_ < DEPTH_HISTORY_SIZE)
                 ? depth_history_fill_ : DEPTH_HISTORY_SIZE;
    if (window == 0) return 999.0f;

    for (int i = 0; i < window; i++) {
        int idx = (depth_history_idx_ - 1 - i + DEPTH_HISTORY_SIZE) % DEPTH_HISTORY_SIZE;
        if (isDepthValid(depth_history_[idx])) {
            sum += depth_history_[idx];
            count++;
        }
    }
    return (count > 0) ? (sum / count) : 999.0f;
}

// ============================================================
// enter
// ============================================================
void State03Avoidance::enter(StateMachine* sm) {
    std::cout << "\n[FSM] >>> 进入 STATE_03: 避障区 V3.2（方案 A 优化版）" << std::endl;
    std::cout << "[FSM] 转弯序列: ";
    for (int i = 0; i < config::s03::TOTAL_TURNS; ++i) {
        std::cout << (config::s03::TURN_DIRECTIONS[i] ? "左 " : "右 ");
    }
    std::cout << std::endl;
    std::cout << "[FSM] 触发=" << config::s03::TURN_TRIGGER_DIST
              << "m | 减速=" << config::s03::TURN_WARNING_DIST
              << "m | 平滑=" << DEPTH_HISTORY_SIZE << "帧" << std::endl;
    std::cout << "[FSM] 侧向三门禁: 前向<" << config::s03::SIDE_GAP_FRONT_MAX
              << "m && 直行>" << config::s03::SIDE_GAP_MIN_STRAIGHT_TIME
              << "s && 连续>=" << config::s03::SIDE_GAP_CONFIRM_FRAMES << "帧" << std::endl;

    phase_            = Phase::STABILIZING;  // 入口对齐使用 STABILIZING
    turn_count_       = 0;
    accumulated_yaw_  = 0.0f;
    confirm_ticks_    = 0;
    side_gap_confirm_ = 0;
    log_tick_         = 0;
    depth_history_idx_  = 0;
    depth_history_fill_ = 0;
    prev_side_depth_    = 0.0f;
    prev_side_valid_    = false;

    for (int i = 0; i < DEPTH_HISTORY_SIZE; i++) depth_history_[i] = 0.0f;

    phase_start_ = std::chrono::steady_clock::now();

    std::cout << "[FSM] 🎯 入口精对准 (target="
              << config::s03::ENTRY_ALIGN_TARGET << "m, timeout="
              << config::s03::ENTRY_ALIGN_TIMEOUT << "s)" << std::endl;
}

// ============================================================
// execute
// ============================================================
void State03Avoidance::execute(StateMachine* sm) {
    if (!sm->robot_driver) return;

    auto now = std::chrono::steady_clock::now();

    // ----- 更新深度滑动窗口 -----
    float raw_depth = sm->vision_data.depth_front;
    depth_history_[depth_history_idx_ % DEPTH_HISTORY_SIZE] = raw_depth;
    depth_history_idx_++;
    if (depth_history_fill_ < DEPTH_HISTORY_SIZE) depth_history_fill_++;

    float depth_smoothed = getSmoothedDepth();
    float depth_left     = sm->vision_data.depth_left;
    float depth_right    = sm->vision_data.depth_right;

    // ================================================================
    // 子阶段 1：直行 + 触发判定
    // ================================================================
    if (phase_ == Phase::FORWARD) {
        if (turn_count_ >= config::s03::TOTAL_TURNS) {
            std::cout << "[迷宫] 🎉 完成全部 " << config::s03::TOTAL_TURNS
                      << " 次转弯，退出避障区！" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            sm->changeState(new State04Stairs());
            return;
        }

        float dt_straight = std::chrono::duration<float>(now - phase_start_).count();

        // ----- 盲走逻辑（第 5、6 段直行） -----
        float blind_walk_time = 0.0f;
        if (turn_count_ == 4)      blind_walk_time = config::s03::BLIND_WALK_T4;
        else if (turn_count_ == 5) blind_walk_time = config::s03::BLIND_WALK_T5;

        bool is_blind_walking = false;
        if (blind_walk_time > 0.0f && dt_straight < blind_walk_time) {
            is_blind_walking = true;
            if (config::debug::VERBOSE_LOG && (log_tick_ % 50 == 0)) {
                std::cout << "[迷宫] 盲走... (" << dt_straight << "s / "
                          << blind_walk_time << "s)" << std::endl;
            }
        }

        // ----- 触发判定 -----
        bool should_trigger = false;
        bool side_gap_gated = false;  // 提前声明，方便日志使用

        if (!is_blind_walking) {
            // ── 方式 1：前向深度触发（平滑后） ──
            bool front_trigger = isDepthValid(depth_smoothed)
                              && depth_smoothed < config::s03::TURN_TRIGGER_DIST;

            if (front_trigger) {
                confirm_ticks_++;
            } else {
                if (confirm_ticks_ > 0
                    && depth_smoothed >= config::s03::TURN_TRIGGER_DIST + 0.10f) {
                    confirm_ticks_ = 0;
                }
            }

            // ── 方式 2：侧向开口提前触发（三道门禁） ──
            //
            //   ★ 门禁 1（空间）：前向必须已 < 0.55m
            //   ★ 门禁 2（时间）：直行已走 > 1.0s
            //   ★ 门禁 3（频次）：侧向开口需连续 3 帧
            //
            side_gap_gated = (isDepthValid(depth_smoothed)
                          && depth_smoothed < config::s03::SIDE_GAP_FRONT_MAX
                          && dt_straight > config::s03::SIDE_GAP_MIN_STRAIGHT_TIME);

            if (side_gap_gated) {
                bool turn_left   = config::s03::TURN_DIRECTIONS[turn_count_];
                float side_depth = turn_left ? depth_left : depth_right;
                bool  side_valid = isDepthValid(side_depth);

                if (prev_side_valid_ && side_valid) {
                    float side_gap = side_depth - prev_side_depth_;
                    if (side_gap > config::s03::SIDE_GAP_THRESHOLD) {
                        side_gap_confirm_++;   // 门禁 3 累计
                    } else {
                        side_gap_confirm_ = 0;
                    }
                }
                prev_side_depth_ = side_depth;
                prev_side_valid_ = side_valid;
            } else {
                // 门禁 1 或 2 未通过 → 重置门禁 3
                side_gap_confirm_ = 0;
                bool turn_left   = config::s03::TURN_DIRECTIONS[turn_count_];
                float side_depth = turn_left ? depth_left : depth_right;
                prev_side_depth_ = side_depth;
                prev_side_valid_ = isDepthValid(side_depth);
            }

            // ── 合并判定 ──
            if (confirm_ticks_ >= config::s03::CONFIRM_FRAMES) {
                should_trigger = true;
                std::cout << "[迷宫] 🔔 前向触发 (平滑=" << depth_smoothed
                          << "m, raw=" << raw_depth << "m)" << std::endl;
            } else if (side_gap_confirm_ >= config::s03::SIDE_GAP_CONFIRM_FRAMES) {
                should_trigger = true;
                std::cout << "[迷宫] 🔔 侧向开口三门禁全通过！(前向=" << depth_smoothed
                          << "m, 直行=" << dt_straight << "s)" << std::endl;
            }
        }

        if (should_trigger) {
            current_turn_left_ = config::s03::TURN_DIRECTIONS[turn_count_];
            std::cout << "[迷宫] 🛑 第 " << (turn_count_ + 1) << "/"
                      << config::s03::TOTAL_TURNS << " 次转弯！方向: "
                      << (current_turn_left_ ? "左" : "右")
                      << " 目标: " << (config::s03::TURN_TARGETS[turn_count_] * 180.0f / 3.14159f) << "°"
                      << std::endl;

            phase_            = Phase::TURNING;
            accumulated_yaw_  = 0.0f;
            phase_start_      = now;
            confirm_ticks_    = 0;
            side_gap_confirm_ = 0;
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        // ----- 预警减速 -----
        float vx = config::s03::CRUISE_VX;
        if (!is_blind_walking && isDepthValid(depth_smoothed)
            && depth_smoothed < config::s03::TURN_WARNING_DIST) {
            vx = config::s03::SLOW_VX;
        }
        sm->robot_driver->move(vx, 0, 0);

        if (config::debug::VERBOSE_LOG && (++log_tick_ % 50 == 0)) {
            std::cout << "[迷宫][直行] raw=" << raw_depth
                      << " smooth=" << depth_smoothed
                      << " L=" << depth_left << " R=" << depth_right
                      << " 前触=" << confirm_ticks_ << "/" << config::s03::CONFIRM_FRAMES
                      << " 侧隙=" << side_gap_confirm_ << "/" << config::s03::SIDE_GAP_CONFIRM_FRAMES
                      << " 三门禁=" << (side_gap_gated ? "开" : "关")
                      << " vx=" << vx
                      << std::endl;
        }
        return;
    }

    // ================================================================
    // 子阶段 2：阿克曼转弯
    // ================================================================
    if (phase_ == Phase::TURNING) {
        float vyaw_mag = config::s03::TURN_VYAW_ARRAY[turn_count_];
        float dt       = std::chrono::duration<float>(now - phase_start_).count();
        accumulated_yaw_ = dt * vyaw_mag;

        // 早退条件：转过 70% 角度且看到通道开阔
        bool min_angle_passed = (accumulated_yaw_ >= config::s03::TURN_TARGETS[turn_count_] * 0.7f);
        bool see_open_path    = isDepthValid(depth_smoothed) && depth_smoothed >= 0.55f;
        if (min_angle_passed && see_open_path) {
            accumulated_yaw_ = config::s03::TURN_TARGETS[turn_count_];
        }

        if (accumulated_yaw_ >= config::s03::TURN_TARGETS[turn_count_]) {
            std::cout << "[迷宫] ✅ 第 " << (turn_count_ + 1) << " 次转弯完成 ("
                      << (accumulated_yaw_ * 180.0f / 3.14159f) << "°)" << std::endl;
            sm->robot_driver->move(0, 0, 0);
            turn_count_++;
            phase_       = Phase::STABILIZING;
            phase_start_ = now;
            return;
        }

        float vyaw = vyaw_mag * (current_turn_left_ ? 1.0f : -1.0f);
        sm->robot_driver->move(config::s03::TURN_VX, 0, vyaw);
        return;
    }

    // ================================================================
    // 子阶段 3：稳定缓冲 + 方向感知姿态校正
    // ================================================================
    if (phase_ == Phase::STABILIZING) {
        float dt = std::chrono::duration<float>(now - phase_start_).count();

        // A. 入口对齐（turn_count_ == 0）
        if (turn_count_ == 0) {
            if (dt > config::s03::ENTRY_ALIGN_TIMEOUT) {
                std::cout << "[FSM] ⏰ 入口对齐超时 (" << dt << "s)，直接进入直行" << std::endl;
                phase_       = Phase::FORWARD;
                phase_start_ = now;
                return;
            }

            if (isDepthValid(depth_left) && isDepthValid(depth_right)) {
                float diff  = depth_left - depth_right;
                float error = diff - config::s03::ENTRY_ALIGN_TARGET;

                if (std::abs(error) < config::s03::ENTRY_ALIGN_TOLERANCE) {
                    std::cout << "[FSM] ✅ 入口精对准完成 (DL-DR=" << diff
                              << " 误差=" << error << " 耗时 " << dt << "s)" << std::endl;
                    sm->robot_driver->move(0, 0, 0);
                    phase_       = Phase::FORWARD;
                    phase_start_ = now;
                    log_tick_    = 0;
                    return;
                }

                float vyaw = (error > 0) ? config::s03::ENTRY_ALIGN_VYAW
                                         : -config::s03::ENTRY_ALIGN_VYAW;
                sm->robot_driver->move(0, 0, vyaw);
            } else {
                sm->robot_driver->move(0, 0, 0);
            }

            if (++log_tick_ % 30 == 0) {
                std::cout << "[FSM][入口精对准] DL=" << depth_left
                          << " DR=" << depth_right << std::endl;
            }
            return;
        }

        // B. 转弯后缓冲 + 方向感知校正
        int just_finished = turn_count_ - 1;
        float wait_time   = config::s03::STABILIZE_TIMES[just_finished];

        if (dt < wait_time) {
            sm->robot_driver->move(0, 0, 0);
            return;
        }

        bool need_align = (just_finished >= 0 && just_finished < 8)
                       && config::s03::ENABLE_POST_TURN_ALIGN[just_finished];

        if (!need_align) {
            std::cout << "[迷宫] ▶️ 缓冲完成（无校正），继续直行" << std::endl;
            phase_            = Phase::FORWARD;
            phase_start_      = now;
            log_tick_         = 0;
            confirm_ticks_    = 0;
            side_gap_confirm_ = 0;
            return;
        }

        float align_elapsed = dt - wait_time;

        if (align_elapsed > config::s03::POST_TURN_ALIGN_TIMEOUT) {
            std::cout << "[迷宫] ⏰ 姿态校正超时 (" << align_elapsed << "s)，继续直行" << std::endl;
            phase_            = Phase::FORWARD;
            phase_start_      = now;
            log_tick_         = 0;
            confirm_ticks_    = 0;
            side_gap_confirm_ = 0;
            return;
        }

        if (isDepthValid(depth_left) && isDepthValid(depth_right)) {
            float diff   = depth_left - depth_right;
            float target = config::s03::POST_TURN_ALIGN_TARGETS[just_finished];
            float error  = diff - target;

            const char* next_dir = "退出";
            if (just_finished + 1 < config::s03::TOTAL_TURNS) {
                next_dir = config::s03::TURN_DIRECTIONS[just_finished + 1] ? "左" : "右";
            }

            if (std::abs(error) < config::s03::POST_TURN_ALIGN_TOLERANCE) {
                std::cout << "[迷宫] ✅ 姿态校正完成 (目标=" << target
                          << " 实际=" << diff << " 误差=" << error
                          << " 下次转" << next_dir
                          << " 耗时 " << align_elapsed << "s)" << std::endl;
                sm->robot_driver->move(0, 0, 0);
                phase_            = Phase::FORWARD;
                phase_start_      = now;
                log_tick_         = 0;
                confirm_ticks_    = 0;
                side_gap_confirm_ = 0;
                return;
            }

            float vyaw = (error > 0) ? config::s03::POST_TURN_ALIGN_VYAW
                                     : -config::s03::POST_TURN_ALIGN_VYAW;
            sm->robot_driver->move(0, 0, vyaw);

            if (++log_tick_ % 20 == 0) {
                std::cout << "[迷宫][校正] 目标=" << target << " 实际=" << diff
                          << " 误差=" << error << " 下次转" << next_dir << std::endl;
            }
        } else {
            sm->robot_driver->move(0, 0, 0);
        }
        return;
    }

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
