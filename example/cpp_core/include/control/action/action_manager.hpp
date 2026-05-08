#ifndef ACTION_MANAGER_HPP
#define ACTION_MANAGER_HPP

#include <string>
#include <chrono>

namespace ruikang {
namespace control {

class RobotDriver;

class ActionManager {
public:
    explicit ActionManager(RobotDriver& driver);

    // 触发一个离散动作（非阻塞），防止重复触发
    void triggerAction(const std::string& action_name);

    // 检查动作是否完成（默认 3 秒超时）
    bool isDone();

    // 当前正在执行的动作名
    const std::string& currentAction() const { return current_action_; }

    // 是否有动作正在执行
    bool inProgress() const { return action_in_progress_; }

private:
    RobotDriver& driver_;
    std::string current_action_ = "none";
    bool action_in_progress_ = false;
    std::chrono::steady_clock::time_point action_start_time_;
};

} // namespace control
} // namespace ruikang

#endif // ACTION_MANAGER_HPP
