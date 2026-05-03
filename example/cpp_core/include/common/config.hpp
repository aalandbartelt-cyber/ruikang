//全局静态参数 (狗的极速限制、PID参数、坐标阈值等)
// include/common/config.hpp
// 全局静态配置参数 - 所有需要现场调的数字都放这里
#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace ruikang {
namespace config {

// =====================================================
// State02: 起点跨栏（纯里程触发）+ 跳跃后寻迹
// =====================================================
namespace s02 {
    // ===== 寻迹基础速度 =====
    constexpr float CRUISE_VX = 0.08f;
    constexpr float SLOW_VX   = 0.06f;
    
    // ===== 加速段忽略 ticks =====
    constexpr int ACCEL_IGNORE_TICKS = 0;
    
    // ===== 物理参考量（注释用，便于复算） =====
    constexpr float D_JUMP_REACH        = 1.03f;
    constexpr float DOG_HALF_LENGTH     = 0.30f;
    constexpr float TARGET_FOOT_TO_OBS  = 0.165f;
    
    // ===== ★ 核心触发距离 ★ =====
    // 5.1 现场标定值：D_JUMP_TRIGGER = 0.40m 时 d_stop ≈ 16cm，完美
    constexpr float D_JUMP_TRIGGER = 0.40f;
    
    // ===== 跳跃前后时序 =====
    constexpr float BRAKE_BEFORE_JUMP  = 0.8f;
    constexpr float RECOVER_AFTER_JUMP = 0.8f;
    
    // ===== ★ 新增：跳跃后寻迹阶段参数 ★ =====
    // 跳跃后至少寻迹这么多 ticks 才考虑切走（防落地抖动误触）
    // 100 ticks × 10ms = 1 秒
    constexpr int POST_JUMP_MIN_TICKS = 100;
    
    // 视觉端"无线"判定阈值：
    //   - 若 Y 视觉无线时返回特殊值（如 999），改成 > 500
    //   - 若 Y 视觉无线时返回大幅 offset，用 200 即可
    //   ⚠️ 5.1 现场需和 Y 视觉端约定，先用 200 占位
    constexpr float NO_LINE_OFFSET_THRESH = 120.0f;
    
    // 连续多少帧"无线"才认定走完黑线
    // 50 ticks × 10ms = 0.5 秒
    constexpr int NO_LINE_TICKS_TO_EXIT = 50;
    
    // 兜底超时：跳跃后寻迹超过这么多 ticks 强制退出（避免视觉异常导致死循环）
    // 1500 ticks × 10ms = 15 秒
    constexpr int POST_JUMP_TIMEOUT_TICKS = 1500;
}

// =====================================================
// State03: 避障区（D435i 深度相机方案）
// 物理基线：D435i 前倾约 45°
//   - 无障碍：看到地面，距离 ≈ 0.66m
//   - 有障碍：距离突然 < 0.50m
// =====================================================
namespace s03 {
    // ===== 直行参数 =====
    constexpr float CRUISE_VX = 0.12f;
    
    
    // ===== 避障核心阈值 =====
    constexpr float TURN_TRIGGER_DIST = 0.40f;

    // 连续帧数确认（防止单帧抖动误触）
    constexpr int CONFIRM_FRAMES = 5;  // 5 帧 ≈ 50ms

    // ===== 阿克曼转弯参数 =====
    constexpr float TURN_VX     = 0.10f;
    constexpr float TURN_VYAW   = 0.60f;
    // ===== 8 次转弯的目标角度数组（弧度制） =====
    constexpr float TURN_TARGETS[8] = {
        1.5708f,  // 第 1 次：90度
        1.5708f,  // 第 2 次：90度
        0.8550f,   // 第 3 次：小于60度 (← 在这里把数值改小)
        0.0850f,  // 第 4 次：小于7度  （修正偏角）
        1.5708f,  // 第 5 次：90度
        1.047f,  // 第 6 次：60度
        0.7500f,  // 第 7 次：大于40度    (修正偏角)
        1.5708f   // 第 8 次：90度
    };

    // ===== 8 次转弯方向（左左左左右右右左） =====
    constexpr bool TURN_DIRECTIONS[8] = {true, true, true, true, false, false, false, true};
    constexpr int  TOTAL_TURNS = 8;

    // ===== 次转弯后的稳定/停顿缓冲时间（秒） =====
    // 发送速度 0，让机器狗原地踏步恢复重心
    constexpr float STABILIZE_TIMES[8] = {
        0.3f,  // 第 1 次转完后
        0.3f,  // 第 2 次转完后
        0.5f,  // 第 3 次转完后
        0.5f,  // 第 4 次转完后
        0.4f,  // 第 5 次转完后：这里加长到 0.5 秒，强制恢复重心！
        0.3f,  // 第 6 次转完后
        0.4f,  // 第 7 次转完后
        0.4f   // 第 8 次转完后
    };

    // ===== 数据有效性检查 =====
    // 如果 depth_front < 0.10m，认为是数据异常（太近不可能），忽略这帧
    constexpr float DEPTH_MIN_VALID = 0.10f;
    // 如果 depth_front >= 9.0m，认为是 D435i 数据缺失，忽略
    constexpr float DEPTH_MAX_VALID = 9.0f;
}
// =====================================================
// State09: 终点跨栏（纯里程触发，参数与 State02 同结构）
// =====================================================
namespace s09 {
    constexpr float CRUISE_VX        = 0.20f;
    constexpr int   ACCEL_IGNORE_TICKS = 100;
    
    // 【★ 5.1 标定 ★】放置平台→终点障碍物的距离
    constexpr float D_JUMP_TRIGGER     = 1.20f;  // 占位
    
    constexpr float BRAKE_BEFORE_JUMP  = 1.5f;
    constexpr float RECOVER_AFTER_JUMP = 0.5f;
}

// =====================================================
// 全局：调试开关
// =====================================================
namespace debug {
    // true: 详细打印里程 / 雷达数据；false: 只打印关键事件
    constexpr bool VERBOSE_LOG = true;
    
    // true: 不下发跳跃指令，只打印日志（dry-run，标定 D_JUMP_TRIGGER 用）
    constexpr bool DRY_RUN_NO_JUMP = false;
}

} // namespace config
} // namespace ruikang

#endif // CONFIG_HPP