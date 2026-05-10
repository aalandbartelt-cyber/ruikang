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
    constexpr float CRUISE_VX = 0.50f;


    // ===== 避障核心阈值 =====
    constexpr float TURN_TRIGGER_DIST = 0.40f;

    // 连续帧数确认（防止单帧抖动误触）
    constexpr int CONFIRM_FRAMES = 5;  // 5 帧 ≈ 50ms

    // ===== 阿克曼转弯参数 =====
    constexpr float TURN_VX     = 0.10f;
    constexpr float TURN_VYAW   = 0.60f;
    // ===== 5 次转弯的目标角度数组（弧度制） =====
    constexpr float TURN_TARGETS[5] = {
        1.5708f,  // 第 1 次：90度
        1.6708f,  // 第 2 次：90度
        1.9708f,  // 第 5 次：90度
        2.3708f,  // 第 6 次：60度
        1.9708f   // 第 8 次：90度
    };

    // ===== 5 次转弯方向（左左右右左） =====
    constexpr bool TURN_DIRECTIONS[5] = {true, true,  false, false,  true};
    constexpr int  TOTAL_TURNS = 5;

    // ===== 次转弯后的稳定/停顿缓冲时间（秒） =====
    // 发送速度 0，让机器狗原地踏步恢复重心
    constexpr float STABILIZE_TIMES[5] = {
        0.4f,  // 第 1 次转完后
        0.4f,  // 第 2 次转完后
        0.4f,  // 第 5 次转完后：这里加长到 0.5 秒，强制恢复重心！
        0.4f,  // 第 6 次转完后
        0.4f   // 第 8 次转完后
    };

    // ===== 数据有效性检查 =====
    // 如果 depth_front < 0.10m，认为是数据异常（太近不可能），忽略这帧
    constexpr float DEPTH_MIN_VALID = 0.10f;
    // 如果 depth_front >= 9.0m，认为是 D435i 数据缺失，忽略
    constexpr float DEPTH_MAX_VALID = 9.0f;
}
// =====================================================
// State04: 台阶区（ArUco 辅助定位 + 步态切换）
// 流程：寻迹→检测ArUco→对齐→切步态→上台阶→下台阶→切回步态→继续寻迹
// =====================================================
namespace s04 {
    // ===== APPROACH：寻迹靠近 =====
    constexpr float APPROACH_VX = 0.45f;  // 灵动步态下需要较高速度才能动

    // ===== ArUco 靠近判断：center_y 超过此阈值认为已走近台阶 =====
    // 前摄 640×480，ArUco 远时 cy≈240，越近越靠下(cy↑)
    constexpr float ARUCO_NEAR_Y_THRESHOLD = 350.0f;  // cy>350 认为靠近，需现场标定

    // ===== ALIGN_ARUCO：对齐台阶 =====
    constexpr float ALIGN_TIMEOUT        = 3.0f;   // 对齐最长等待时间 (s)
    constexpr float ALIGN_CENTER_TOL_PX  = 30.0f;  // ArUco 中心偏差容忍（像素）
    constexpr float ALIGN_VYAW           = 0.25f;  // 对齐转速
    constexpr int   IMAGE_CENTER_X       = 320;    // 图像宽度一半（640x480）

    // ===== CLIMB_ARC：弧线连贯上下台阶 =====
    // 狗尺寸大无法四腿站第三阶，上到第二阶后边前进边左转画弧
    // 单一连贯动作替代原来的 上台阶→顶部转90°→下台阶
    constexpr float CLIMB_ARC_VX       = 0.42f;   // 弧线前进速度 (m/s)
    constexpr float CLIMB_ARC_VYAW     = 0.18f;   // 弧线左转角速度 (rad/s)，8s≈82°
    constexpr float CLIMB_ARC_DURATION = 8.0f;    // 弧线总时长 (s)，需现场标定

    // ===== EXIT_FOLLOW：离开台阶 =====
    constexpr float EXIT_FOLLOW_VX       = 0.18f;
    constexpr float EXIT_FOLLOW_DURATION = 15.0f;

    // ===== 安全保护 =====
    constexpr float TOTAL_TIMEOUT = 60.0f;
}

// =====================================================
// State07: 检测平台（警示牌识别 + 指定动作执行）
// 流程：寻迹走近→停靠→读警示牌→执行动作→等待完成→继续巡线
// =====================================================
namespace s07 {
    // ===== APPROACH：寻迹靠近检测点 =====
    constexpr float APPROACH_VX       = 0.18f;   // 巡线速度
    constexpr float APPROACH_DURATION = 18.0f;   // 寻迹超时 (s)，过弯后长直道需足够时间

    // ===== RED_DOT_FORWARD：检测到红点后继续巡线逼近（D435i 前倾 45°，需补偿） =====
    constexpr float RED_DOT_FORWARD_DURATION = 3.6f;   // 红点出现后盲巡 3.6s 站到红点上（5.9实测）

    // ===== BACK_AWAY：左转后离警示牌太近，后退拉开距离 =====
    constexpr float BACK_AWAY_VX       = -0.06f;  // 后退速度（负 = 后退）
    constexpr float BACK_AWAY_DURATION = 1.0f;    // 后退时长 (s)

    // ===== RED_DOT_ALIGN：红点精确定位 =====
    constexpr float RED_DOT_CENTER_TOL_PX = 25.0f;  // 红点中心偏差容忍（像素）
    constexpr float RED_DOT_ALIGN_VYAW     = 0.20f; // 对齐转速
    constexpr float RED_DOT_ALIGN_TIMEOUT  = 5.0f;  // 对齐超时 (s)
    constexpr int   IMAGE_CENTER_X         = 320;   // 图像水平中心（640x480）

    // ===== TURN_TO_SIGN：红点定位后左转 90° 面向警示牌 =====
    constexpr float TURN_TO_SIGN_VYAW   = 0.50f;   // 左转角速度 (rad/s)
    constexpr float TURN_TO_SIGN_TARGET = 1.5708f; // 目标 90°

    // ===== STOP_AND_READ：停稳读取 =====
    constexpr float STOP_DURATION     = 1.0f;    // 停稳确认时间 (s)

    // ===== WAIT_ACTION：等待动作完成 =====
    constexpr float ACTION_TIMEOUT    = 3.0f;

    // ===== TURN_BACK：动作完成后右转 90° 回正，面向巡线方向 =====
    constexpr float TURN_BACK_VYAW   = 0.50f;   // 右转角速度 (rad/s)
    constexpr float TURN_BACK_TARGET = 1.5708f; // 目标 90°

    // ===== EXIT_FOLLOW：继续巡线离开 =====
    constexpr float EXIT_FOLLOW_VX       = 0.08f;
    constexpr float EXIT_FOLLOW_DURATION = 3.0f;

    // ===== 安全保护 =====
    constexpr float TOTAL_TIMEOUT     = 45.0f;   // 含两次原地转弯，适当放宽

    // ★ 警示牌 → 动作映射（由 ActionManager::triggerAction 非阻塞执行）
    //   伸懒腰(stretch) / 打招呼(greet) / 闪灯(flash_lights) — 三项均为非阻塞，≤3s
    inline const char* action_for_sign(const std::string& sign) {
        if (sign == "ELECTRIC")  return "stretch";
        if (sign == "OXIDANT")   return "greet";
        if (sign == "RADIATION") return "flash_lights";
        if (sign == "FIRE")      return "greet";  // 当心强氧化物 → 打招呼
        if (sign == "TOXIC")     return "stretch";
        return nullptr;  // NONE / 无法识别 → 不做动作
    }
}

// =====================================================
// State09: 终点跨栏（纯里程触发，参数与 State02 同结构）
// =====================================================
namespace s09 {
    constexpr float CRUISE_VX        = 0.08f;  // 标定阶段与 State02 同速，DRY_RUN 后逐步提
    constexpr float SLOW_VX          = 0.06f;
    constexpr int   ACCEL_IGNORE_TICKS = 100;

    // 【★ 5.1 标定 ★】放置平台→终点障碍物的距离
    constexpr float D_JUMP_TRIGGER     = 1.20f;  // 5.9 现场标定

    constexpr float BRAKE_BEFORE_JUMP  = 1.5f;
    constexpr float RECOVER_AFTER_JUMP = 0.5f;

    // 跳跃后寻迹进入蓝色启停区 (ticks, 10ms/tick)
    constexpr int POST_JUMP_TICKS = 150;  // 1.5s，需 5.10 标定
}

// =====================================================
// State10: 蓝色启停区 — 稳定停靠 + 完全锁死电机
// =====================================================
namespace s10 {
    constexpr float STOP_DELAY   = 1.5f;   // 进区后稳定时间 (s)
    constexpr float TOTAL_TIMEOUT = 15.0f;  // 全局超时保护
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