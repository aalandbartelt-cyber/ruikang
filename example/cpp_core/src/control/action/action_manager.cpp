#include "control/action/action_manager.hpp"
#include "control/hal/robot_driver.hpp"

namespace ruikang {
namespace control {

ActionManager::ActionManager(RobotDriver& driver)
    : driver_(driver) {}

void ActionManager::triggerAction(const std::string& action_name) {
    if (action_in_progress_) return; // 防止重复触发

    current_action_ = action_name;
    action_start_time_ = std::chrono::steady_clock::now();
    action_in_progress_ = true;

    // 映射动作名到 RobotDriver 非阻塞方法
    if (action_name == "stretch") {
        driver_.stretch();
    } else if (action_name == "greet") {
        driver_.greet();
    } else if (action_name == "flash_lights") {
        driver_.flashLights();
    } else if (action_name == "jump") {
        driver_.jumpObstacle();
    } else {
        // 回退到 playSpecialAction（兼容旧名称）
        driver_.playSpecialAction(action_name);
    }
}

bool ActionManager::isDone() {
    if (!action_in_progress_) return true;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - action_start_time_).count();

    // 动作生命周期上限 3 秒
    if (elapsed >= 3.0) {
        action_in_progress_ = false;
        current_action_ = "none";
        return true;
    }
    return false;
}

} // namespace control
} // namespace ruikang
