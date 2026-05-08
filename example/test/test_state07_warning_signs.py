# -*- coding: utf-8 -*-
"""
test_state07_warning_signs.py — 三种警示牌响应仿真测试
=====================================================
模拟 State07 完整流程，验证:
  ELECTRIC  -> stretch (非阻塞伸懒腰)
  OXIDANT   -> greet   (非阻塞打招呼)
  RADIATION -> flash_lights (非阻塞闪灯)
  NONE      -> 跳过动作，直接离开

用法: python test_state07_warning_signs.py
"""
import sys
import os

# 强制 UTF-8 输出，避免 GBK 终端报错
if sys.platform == "win32":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# ============================================================
# 1:1 复刻 config.hpp s07 参数
# ============================================================
class S07Config:
    APPROACH_DURATION  = 4.0
    STOP_DURATION      = 1.0
    ACTION_TIMEOUT     = 3.0
    EXIT_FOLLOW_DURATION = 3.0
    TOTAL_TIMEOUT      = 30.0

    @staticmethod
    def action_for_sign(sign):
        """config.hpp s07::action_for_sign() 精确复刻"""
        if sign == "ELECTRIC":   return "stretch"
        if sign == "OXIDANT":    return "greet"
        if sign == "RADIATION":  return "flash_lights"
        return None

# ============================================================
# 2: 模拟 ActionManager (对应 action_manager.cpp)
# ============================================================
class MockActionManager:
    def __init__(self):
        self.current_action = "none"
        self.in_progress = False
        self.elapsed = 0.0
        self.call_log = []

    def triggerAction(self, action_name):
        if self.in_progress:
            print(f"    [ActionManager] BLOCKED: refused duplicate trigger "
                  f"(current: {self.current_action})")
            return
        self.current_action = action_name
        self.elapsed = 0.0
        self.in_progress = True
        self.call_log.append(action_name)
        print(f"    [ActionManager] triggerAction(\"{action_name}\") -> non-blocking OK")

        # 模拟实际底层调用 (对应 robot_driver.cpp)
        calls = {
            "stretch":      "RobotDriver::stretch()      -> sport_client->Stretch()",
            "greet":        "RobotDriver::greet()        -> sport_client->Hello()",
            "flash_lights": "RobotDriver::flashLights()  -> light control",
        }
        print(f"       -> {calls.get(action_name, 'RobotDriver::playSpecialAction() [fallback]')}")

    def tick(self, dt: float) -> bool:
        """模拟时间推进，返回 isDone()"""
        if not self.in_progress:
            return True
        self.elapsed += dt
        if self.elapsed >= 3.0:
            self.in_progress = False
            self.current_action = "none"
            return True
        return False

    def get_elapsed(self) -> float:
        return self.elapsed

# ============================================================
# 3: 模拟 State07 核心逻辑 (对应 state_07_detection.cpp)
# ============================================================
class Phase:
    APPROACH       = "APPROACH"
    STOP_AND_READ  = "STOP_AND_READ"
    EXECUTE_ACTION = "EXECUTE_ACTION"
    WAIT_ACTION    = "WAIT_ACTION"
    EXIT_FOLLOW    = "EXIT_FOLLOW"
    FINISHED       = "FINISHED"

def simulate_state07(sign: str):
    """用模拟时间 (dt=0.01s steps) 跑完 State07 全流程"""
    phase = Phase.APPROACH
    action_mgr = MockActionManager()
    phase_elapsed = 0.0
    total_elapsed = 0.0
    dt = 0.01  # 10ms tick
    action_name = ""
    log = []

    while True:
        total_elapsed += dt
        phase_elapsed += dt

        # 全局超时
        if total_elapsed > S07Config.TOTAL_TIMEOUT:
            log.append(f"TIMEOUT at {total_elapsed:.1f}s")
            break

        # --- APPROACH ---
        if phase == Phase.APPROACH:
            if phase_elapsed > S07Config.APPROACH_DURATION:
                phase = Phase.STOP_AND_READ
                phase_elapsed = 0.0

        # --- STOP_AND_READ ---
        elif phase == Phase.STOP_AND_READ:
            if phase_elapsed > S07Config.STOP_DURATION:
                action = S07Config.action_for_sign(sign)
                if action:
                    action_name = action
                    phase = Phase.EXECUTE_ACTION
                else:
                    phase = Phase.EXIT_FOLLOW
                phase_elapsed = 0.0

        # --- EXECUTE_ACTION ---
        elif phase == Phase.EXECUTE_ACTION:
            action_mgr.triggerAction(action_name)
            phase = Phase.WAIT_ACTION
            phase_elapsed = 0.0

        # --- WAIT_ACTION ---
        elif phase == Phase.WAIT_ACTION:
            action_mgr.tick(dt)
            done = action_mgr.get_elapsed() >= 3.0  # isDone() -> true after 3s
            if done:
                log.append(f"action_done: {action_name} at {total_elapsed:.1f}s")
                phase = Phase.EXIT_FOLLOW
                phase_elapsed = 0.0
            elif phase_elapsed > S07Config.ACTION_TIMEOUT:
                log.append(f"action_timeout at {total_elapsed:.1f}s")
                phase = Phase.EXIT_FOLLOW
                phase_elapsed = 0.0

        # --- EXIT_FOLLOW ---
        elif phase == Phase.EXIT_FOLLOW:
            if phase_elapsed > S07Config.EXIT_FOLLOW_DURATION:
                phase = Phase.FINISHED
                break

        elif phase == Phase.FINISHED:
            break

    return total_elapsed, action_mgr.call_log


# ============================================================
# 4: 主测试
# ============================================================
if __name__ == "__main__":
    results = {}
    print("State07 Warning Sign Response Test")
    print("=" * 55)

    for sign in ["ELECTRIC", "OXIDANT", "RADIATION", "NONE"]:
        print(f"\n--- Testing: {sign} ---")
        elapsed, calls = simulate_state07(sign)
        action = S07Config.action_for_sign(sign)
        results[sign] = {"elapsed": elapsed, "calls": calls, "action": action}
        print(f"  Elapsed: {elapsed:.1f}s")
        print(f"  ActionManager calls: {calls}")
        print(f"  Expected: {[action] if action else []}")

    # Summary
    print(f"\n{'='*55}")
    print(f"{'Sign':<14} {'Action':<14} {'Time':>6} {'Calls':<20} Result")
    print(f"{'-'*55}")

    all_ok = True
    for sign, r in results.items():
        action = r["action"] or "(skip)"
        calls = r["calls"]
        expected = [r["action"]] if r["action"] else []

        if sign == "NONE":
            ok = (calls == [])
            status = "OK (skipped)"
        else:
            ok = (calls == expected)
            status = "OK" if ok else f"FAIL: expected {expected}"

        if not ok:
            all_ok = False
        print(f"  {sign:<14} {action:<14} {r['elapsed']:>5.1f}s  {str(calls):<20} {status}")

    print(f"  {'='*55}")
    print(f"  Conclusion: {'ALL PASSED' if all_ok else 'SOME FAILED'}")

    # Extra: 重复触发防护
    print(f"\n--- Extra: Duplicate Trigger Protection ---")
    mgr2 = MockActionManager()
    mgr2.triggerAction("stretch")
    mgr2.triggerAction("greet")
    mgr2.triggerAction("flash_lights")
    print(f"  Call log: {mgr2.call_log}")
    if mgr2.call_log == ["stretch"]:
        print(f"  Protection: OK (only 'stretch' recorded)")
    else:
        print(f"  Protection: FAIL (expected ['stretch'], got {mgr2.call_log})")
        all_ok = False

    print(f"\n{'='*55}")
    print(f"  Final: {'ALL PASSED' if all_ok else 'SOME FAILED'}")
