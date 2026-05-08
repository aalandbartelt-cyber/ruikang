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
// State03: 避障区（D435i 深度相机方案 V3.2）
//
// V3.1 → V3.2 改动：
//   - 侧向开口"三道门禁"概念显式标注，参数本身不变
//   - 准备 5.9 现场对照测试（删 5° vs 保留 5°）
// =====================================================
namespace s03 {
    // ===== 直行参数 =====
    constexpr float CRUISE_VX = 0.12f;
    constexpr float SLOW_VX   = 0.06f;   // 接近障碍时减速

    // ===== 避障核心阈值 =====
    constexpr float TURN_TRIGGER_DIST  = 0.45f;  // 前向触发
    constexpr float TURN_WARNING_DIST  = 0.50f;  // 减速阈值

    // 前向触发：连续帧数确认（防单帧抖动）
    constexpr int CONFIRM_FRAMES = 5;

    // ===== 深度滑动窗口滤波 =====
    constexpr int DEPTH_SMOOTH_WINDOW = 5;       // 5 帧移动平均

    // ===== 侧向开口提前触发：三道门禁 =====
    //
    // 门禁 1（空间）：前向必须接近 → 屏蔽入口空旷区误触发
    constexpr float SIDE_GAP_FRONT_MAX = 0.55f;
    //
    // 门禁 2（时间）：直行已走过最低时间 → 屏蔽姿态未稳误触发
    constexpr float SIDE_GAP_MIN_STRAIGHT_TIME = 1.0f;
    //
    // 门禁 3（频次）：侧向开口需连续 N 帧确认 → 抗深度跳变误触发
    constexpr int   SIDE_GAP_CONFIRM_FRAMES = 3;
    //
    // 阈值：侧向距离单帧跃升超过此值即视为"看到开口"
    constexpr float SIDE_GAP_THRESHOLD = 0.25f;

    // ===== 阿克曼转弯参数 =====
    constexpr float TURN_VX     = 0.10f;
    constexpr float TURN_VYAW   = 0.60f;

    // ===== 8 次转弯目标角度（弧度） =====
    constexpr float TURN_TARGETS[8] = {
        1.5708f,  // 第1次：90°
        1.5708f,  // 第2次：90°
        0.8650f,  // 第3次：~49°
        0.2000f,  // 第4次：~11°（修正偏角）
        1.5708f,  // 第5次：90°
        1.0470f,  // 第6次：60°
        0.7500f,  // 第7次：~43°（修正偏角）
        1.5708f   // 第8次：90°
    };

    constexpr bool TURN_DIRECTIONS[8] = {
        true, true, true, true,    // 4 次左
        false, false, false,       // 3 次右
        true                       // 最后 1 次左
    };
    constexpr int TOTAL_TURNS = 8;

    // 每段独立转弯角速度（小角度需较长时间才能起作用）
    constexpr float TURN_VYAW_ARRAY[8] = {
        0.60f, 0.60f, 0.50f, 0.20f,
        0.60f, 0.50f, 0.30f, 0.60f
    };

    // 每次转弯后稳定缓冲时间
    constexpr float STABILIZE_TIMES[8] = {
        0.3f, 0.3f, 0.4f, 0.5f, 0.4f, 0.4f, 0.5f, 0.4f
    };

    // ==========================================================
    // ★ 方向感知姿态修正
    // ==========================================================
    constexpr float ENTRY_ALIGN_TARGET     = -0.05f;  // 入口偏左 5cm
    constexpr float ENTRY_ALIGN_TOLERANCE  = 0.08f;
    constexpr float ENTRY_ALIGN_VYAW       = 0.30f;
    constexpr float ENTRY_ALIGN_TIMEOUT    = 1.5f;

    // 每次转弯后的目标偏移：根据下一次转弯方向决定
    //   下次左转 → 偏左（target<0），给屁股留右侧空间
    //   下次右转 → 偏右（target>0），给屁股留左侧空间
    constexpr float POST_TURN_ALIGN_TARGETS[8] = {
        -0.05f,  // [0] 第1次后 → 第2次左 → 偏左
        -0.05f,  // [1] 第2次后 → 第3次左 → 偏左
        -0.05f,  // [2] 第3次后 → 第4次左 → 偏左
         0.05f,  // [3] 第4次后 → 第5次右 → 偏右
         0.05f,  // [4] 第5次后 → 第6次右 → 偏右
         0.05f,  // [5] 第6次后 → 第7次右 → 偏右
        -0.05f,  // [6] 第7次后 → 第8次左 → 偏左
         0.00f,  // [7] 第8次后 → 退出，不使用
    };

    constexpr float POST_TURN_ALIGN_TOLERANCE = 0.08f;
    constexpr float POST_TURN_ALIGN_VYAW      = 0.25f;
    constexpr float POST_TURN_ALIGN_TIMEOUT   = 1.0f;

    constexpr bool ENABLE_POST_TURN_ALIGN[8] = {
        true,   // [0] 第1次后
        true,   // [1] 第2次后
        true,   // [2] 第3次后（关键：防第4次微调失效）
        false,  // [3] 第4次后（本身是微调）
        true,   // [4] 第5次后
        true,   // [5] 第6次后
        true,   // [6] 第7次后
        false,  // [7] 第8次后（已出区）
    };

    // ===== 数据有效性检查 =====
    constexpr float DEPTH_MIN_VALID = 0.10f;
    constexpr float DEPTH_MAX_VALID = 9.0f;

    // ===== 盲走参数 =====
    constexpr float BLIND_WALK_T4 = 1.95f;
    constexpr float BLIND_WALK_T5 = 1.65f;
}
// =====================================================
// State04: 台阶区（ArUco 辅助定位 + 步态切换）
// 流程：寻迹→检测ArUco→对齐→切步态→上台阶→下台阶→切回步态→继续寻迹
// =====================================================
namespace s04 {
    // ===== APPROACH：寻迹靠近 =====
    constexpr float APPROACH_VX = 0.08f;

    // ===== ALIGN_ARUCO：对齐台阶 =====
    constexpr float ALIGN_TIMEOUT        = 3.0f;   // 对齐最长等待时间 (s)
    constexpr float ALIGN_CENTER_TOL_PX  = 30.0f;  // ArUco 中心偏差容忍（像素）
    constexpr float ALIGN_VYAW           = 0.25f;  // 对齐转速
    constexpr int   IMAGE_CENTER_X       = 320;    // 图像宽度一半（640x480）

    // ===== CLIMB：爬台阶 =====
    constexpr float CLIMB_VX             = 0.06f;  // 爬台阶速度 (m/s)
    constexpr float CLIMB_UP_DURATION    = 5.0f;   // 上台阶时长 (s)，需 5.10 标定
    constexpr float CLIMB_DOWN_DURATION  = 5.0f;   // 下台阶时长 (s)，需 5.10 标定

    // ===== TOP_TURN_LEFT：台阶顶部左转 90° =====
    // 翻越式台阶：上到顶部后左转 90° 对准下台阶方向
    constexpr float TOP_TURN_VX     = 0.0f;        // 原地转
    constexpr float TOP_TURN_VYAW   = 0.50f;       // 转弯角速度 (rad/s)
    constexpr float TOP_TURN_TARGET = 1.5708f;     // 目标 90°（弧度）

    // ===== EXIT_FOLLOW：离开台阶 =====
    constexpr float EXIT_FOLLOW_VX       = 0.08f;
    constexpr float EXIT_FOLLOW_DURATION = 3.0f;

    // ===== 安全保护 =====
    constexpr float TOTAL_TIMEOUT = 60.0f;
}

// =====================================================
// State07: 检测平台（警示牌识别 + 指定动作执行）
// 流程：寻迹走近→停靠→读警示牌→执行动作→等待完成→继续巡线
// =====================================================
namespace s07 {
    // ===== APPROACH：寻迹靠近检测点 =====
    constexpr float APPROACH_VX       = 0.08f;   // 巡线速度
    constexpr float APPROACH_DURATION = 8.0f;    // 寻迹超时 (s)，兜底保护

    // ===== RED_DOT_ALIGN：红点精确定位 =====
    constexpr float RED_DOT_CENTER_TOL_PX = 25.0f;  // 红点中心偏差容忍（像素）
    constexpr float RED_DOT_ALIGN_VYAW     = 0.20f; // 对齐转速
    constexpr float RED_DOT_ALIGN_TIMEOUT  = 5.0f;  // 对齐超时 (s)
    constexpr int   IMAGE_CENTER_X         = 320;   // 图像水平中心（640x480）

    // ===== STOP_AND_READ：停稳读取 =====
    constexpr float STOP_DURATION     = 1.0f;    // 停稳确认时间 (s)

    // ===== WAIT_ACTION：等待动作完成 =====
    // ★ TODO（X 实现 isActionDone() 后，用轮询替代固定超时）：
    //   当前：固定超时 3s，之后无论是否完成都继续
    constexpr float ACTION_TIMEOUT    = 3.0f;

    // ===== EXIT_FOLLOW：继续巡线离开 =====
    constexpr float EXIT_FOLLOW_VX       = 0.08f;
    constexpr float EXIT_FOLLOW_DURATION = 3.0f;

    // ===== 安全保护 =====
    constexpr float TOTAL_TIMEOUT     = 30.0f;

    // ★ 警示牌 → 动作映射（由 ActionManager::triggerAction 非阻塞执行）
    //   伸懒腰(stretch) / 打招呼(greet) / 闪灯(flash_lights) — 三项均为非阻塞，≤3s
    inline const char* action_for_sign(const std::string& sign) {
        if (sign == "ELECTRIC")  return "stretch";
        if (sign == "OXIDANT")   return "greet";
        if (sign == "RADIATION") return "flash_lights";
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