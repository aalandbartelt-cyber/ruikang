# -*- coding: utf-8 -*-
"""
test_state09_10.py — State09 终点跳跃 + State10 终点停靠 仿真测试
===============================================================
验证：
  1. State09 里程触发跳跃（D_JUMP_TRIGGER = 1.20m）
  2. 跳跃后寻迹进入蓝色启停区（POST_JUMP_TICKS = 150）
  3. State10 停稳 + lockMotors() 锁死

用法: python test_state09_10.py
"""
import sys
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ============================================================
# 复刻 config.hpp
# ============================================================
class S09:
    CRUISE_VX          = 0.20
    SLOW_VX            = 0.12
    ACCEL_IGNORE_TICKS = 100
    D_JUMP_TRIGGER     = 1.20
    BRAKE_BEFORE_JUMP  = 1.5
    RECOVER_AFTER_JUMP = 0.5
    POST_JUMP_TICKS    = 150

class S10:
    STOP_DELAY   = 1.5
    TOTAL_TIMEOUT = 15.0

# ============================================================
# 模拟 RobotDriver（记录调用）
# ============================================================
class MockDriver:
    def __init__(self):
        self.log = []
        self.locked = False
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

    def move(self, vx, vy, vyaw):
        self.vx, self.vy, self.vyaw = vx, vy, vyaw

    def jump(self):
        self.log.append("jump()")

    def lockMotors(self):
        self.log.append("lockMotors() -> emergencyDamp()")
        self.locked = True

# ============================================================
# 模拟 State09
# ============================================================
def simulate_state09(verbose=True):
    driver = MockDriver()
    dt = 0.01  # 10ms tick

    # State09 内部状态
    is_jumping = False
    post_jump_following = False
    current_vx = 0.0
    accel_ignore_ticks = S09.ACCEL_IGNORE_TICKS
    cruise_mode = (accel_ignore_ticks == 0)
    traveled_dist = 0.0
    post_jump_ticks = 0
    last_tick_time = 0.0
    tick = 0
    total_time = 0.0

    # Mock PID (simple)
    line_offset = 5.0  # 假设基本在线上

    while True:
        total_time += dt
        tick += 1

        if is_jumping:
            # 跳跃阻塞阶段（刹车+跳跃+恢复）
            continue

        # --- 阶段 C: 跳后寻迹 ---
        if post_jump_following:
            post_jump_ticks += 1

            if post_jump_ticks > S09.POST_JUMP_TICKS:
                if verbose:
                    print(f"  [State09] 跳后寻迹完成 ({post_jump_ticks} ticks)")
                    print(f"  [State09] -> changeState(State10Finish)")
                return driver, total_time, "->State10"

            driver.move(S09.CRUISE_VX, 0, 0.02)  # 微小偏航修正
            continue

        # --- 阶段 A: 里程累加 ---
        if accel_ignore_ticks > 0:
            accel_ignore_ticks -= 1
            if accel_ignore_ticks == 0:
                cruise_mode = True
                traveled_dist = 0.0
                last_tick_time = total_time
        elif cruise_mode:
            dt_real = total_time - last_tick_time
            last_tick_time = total_time
            traveled_dist += current_vx * dt_real

            if traveled_dist >= S09.D_JUMP_TRIGGER:
                if verbose:
                    print(f"  [State09] 触发跳跃! traveled={traveled_dist:.2f}m "
                          f">= D_JUMP_TRIGGER={S09.D_JUMP_TRIGGER}m")
                    print(f"  [State09] 刹车 {S09.BRAKE_BEFORE_JUMP}s -> jump() "
                          f"-> 恢复 {S09.RECOVER_AFTER_JUMP}s")
                is_jumping = True

                # 模拟跳跃阻塞时间
                jump_block_time = S09.BRAKE_BEFORE_JUMP + 2.5 + S09.RECOVER_AFTER_JUMP
                total_time += jump_block_time

                driver.jump()
                post_jump_following = True
                is_jumping = False
                post_jump_ticks = 0
                continue

        # 寻迹速度
        target_vx = S09.CRUISE_VX
        if abs(line_offset) > 80:
            target_vx = S09.SLOW_VX

        if current_vx < target_vx - 0.006:
            current_vx += 0.005
        elif current_vx > target_vx + 0.016:
            current_vx -= 0.015
        else:
            current_vx = target_vx

        driver.move(current_vx, 0, 0.01)  # 微小平稳偏航

    return driver, total_time, "timeout"


# ============================================================
# 模拟 State10
# ============================================================
def simulate_state10(verbose=True):
    driver = MockDriver()
    dt = 0.01
    locked = False
    elapsed = 0.0

    # enter
    driver.move(0, 0, 0)
    if verbose:
        print(f"  [State10] enter: 刹车停稳, 等待 {S10.STOP_DELAY}s 后锁死")

    while True:
        elapsed += dt

        if elapsed > S10.TOTAL_TIMEOUT:
            if not locked:
                if verbose:
                    print(f"  [State10] 超时 {elapsed:.1f}s, 强制锁死")
                driver.lockMotors()
                locked = True
            break

        if not locked:
            if elapsed < S10.STOP_DELAY:
                driver.move(0, 0, 0)  # 停稳中
            else:
                if verbose:
                    print(f"  [State10] 稳定 {S10.STOP_DELAY}s -> lockMotors()")
                driver.lockMotors()
                locked = True
                if verbose:
                    print(f"  [State10] 电机已锁死，进入阻尼态。任务完成。")

        if locked:
            break

    return driver, elapsed, locked


# ============================================================
# 主测试
# ============================================================
if __name__ == "__main__":
    print("State09 + State10 Integration Test")
    print("=" * 55)

    # Test 1: State09
    print("\n--- Test 1: State09 (终点障碍跳跃) ---")
    driver_s9, t9, result9 = simulate_state09()
    expected_log = ["jump()"]
    ok9 = (driver_s9.log == expected_log) and (result9 == "->State10")
    print(f"  Driver calls: {driver_s9.log}")
    print(f"  Result: {result9}")
    print(f"  Total time: {t9:.1f}s")
    print(f"  {'OK' if ok9 else 'FAIL'}")

    # Compute expected State09 time
    # ACCEL_IGNORE: 100 ticks * 10ms = 1.0s
    # Cruise to 1.20m at 0.20 m/s = 6.0s (approx, with ramp-up)
    # Jump block: 1.5 + 2.5 + 0.5 = 4.5s
    # Post-jump: 150 ticks * 10ms = 1.5s
    # Total: ~1.0 + 6.0 + 4.5 + 1.5 = ~13s
    print(f"  Expected: ~13s (1s accel + 6s cruise + 4.5s jump + 1.5s post)")

    # Test 2: State10
    print("\n--- Test 2: State10 (终点停靠锁死) ---")
    driver_s10, t10, locked = simulate_state10()
    ok10 = ("lockMotors() -> emergencyDamp()" in driver_s10.log) and locked
    print(f"  Driver calls: {driver_s10.log}")
    print(f"  Locked: {locked}")
    print(f"  Time to lock: {t10:.1f}s")
    print(f"  Expected: {S10.STOP_DELAY}s delay + lock")
    print(f"  {'OK' if ok10 else 'FAIL'}")

    # Summary
    print(f"\n{'='*55}")
    print(f"  State09 (jump trigger + post-follow): {'OK' if ok9 else 'FAIL'}")
    print(f"  State10 (stop + lock motors):         {'OK' if ok10 else 'FAIL'}")
    print(f"  {'='*55}")
    all_ok = ok9 and ok10
    print(f"  Conclusion: {'ALL PASSED' if all_ok else 'SOME FAILED'}")
