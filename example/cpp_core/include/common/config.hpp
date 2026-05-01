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
// State03: 避障区（点云 + 阿克曼转弯）
// V5.0：基于 RViz 实测 + 真实通道场景调优
// 实测：狗背 Z=+0.06，前方挡板 Z=+0.11
//       右侧通道墙 Y=-1.06（说明侧墙也会被扫到，必须收紧 Y）
// =====================================================
namespace s03 {
    // ===== 直行参数 =====
    constexpr float CRUISE_VX = 0.12f;
    
    // ===== 触发转弯的前方距离（米） =====
    constexpr float TURN_TRIGGER_DIST = 0.40f;
    
    // ===== ★ 挡板高度范围 ★ =====
    // 实测：狗背 Z=+0.06，挡板上端 Z 在 +0.10~+0.30 之间
    constexpr float WALL_Z_MIN = +0.05f;   // 略高于狗背
    constexpr float WALL_Z_MAX = +0.30f;   // 包含 45cm 挡板
    
    // ===== ★ 前方扫描的 Y 窄条（必须严格） ★ =====
    // 通道宽 60cm，狗中心到墙 30cm
    // 必须严格限制 Y_HALF ≤ 0.20，否则会扫到通道左右两侧的墙
    constexpr float WALL_Y_HALF = 0.20f;
    
    // ===== ★ 挡板点数阈值（白色塑料板反射弱，不能设太高）★ =====
    constexpr int WALL_POINT_THRESH = 3;
    
    // ===== 阿克曼转弯参数 =====
    constexpr float TURN_VX     = 0.10f;
    constexpr float TURN_VYAW   = 0.60f;
    constexpr float TURN_TARGET = 1.5708f;
    
    // ===== 5 次转弯方向（左左右右左） =====
    constexpr bool TURN_DIRECTIONS[5] = {true, true, false, false, true};
    constexpr int  TOTAL_TURNS = 5;
    
    // ===== 侧距参数（保留但 State03 暂不使用） =====
    constexpr float SIDE_WARN_DIST = 0.12f;
    constexpr float SIDE_X_MIN = 0.30f;
    constexpr float SIDE_X_MAX = 0.70f;
    constexpr float SIDE_Y_MIN = 0.30f;
    constexpr float SIDE_Y_MAX = 0.50f;
    
    // ===== 转弯后稳定时间 =====
    constexpr float STABILIZE_AFTER_TURN = 0.3f;
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