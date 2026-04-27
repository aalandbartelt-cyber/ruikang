#include <iostream>
#include <unistd.h>
#include <memory>
#include "control/hal/robot_driver.hpp"
#include "control/hal/lidar_handler.hpp" // 🌟 引入队友的新架构头文件
#include "common/udp_receiver.hpp"
#include "fsm/state_machine.hpp"
#include "fsm/states/state_01_init.hpp"

using namespace ruikang::control;
using namespace ruikang::fsm;

int main() {
    std::cout << "[System] Starting Multimodal Inspection System (HAL Architecture)..." << std::endl;

    // 1. 启动 UDP 接收子线程
    UdpReceiver receiver(8080);
    receiver.start();

    // 2. 初始化底层网络与驱动
    RobotDriver driver;
    driver.initConnection("eth0");

    // 3. 🌟 按队友思路初始化 HAL 层雷达管理器
    LidarHandler lidar_handler;
    lidar_handler.init();

    // 4. 实例化状态机大脑
    StateMachine brain(&driver, new State01Init());

    std::cout << "--- FSM Main Loop Started ---" << std::endl;

    while (true) {
        VisionData data = receiver.getLatestData();

        // 🌟 数据强覆盖：调用新的 HAL 层接口获取真实距离
        data.obstacle_distance = lidar_handler.get_front_distance();

        brain.update(data);
        usleep(10000);
    }

    receiver.stop();
    return 0;
}