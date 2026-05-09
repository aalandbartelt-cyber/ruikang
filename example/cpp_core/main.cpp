#include <iostream>
#include <unistd.h>
#include "control/hal/robot_driver.hpp"

#include "common/udp_receiver.hpp"
#include "fsm/state_machine.hpp"
#include "fsm/states/state_01_init.hpp"
#include "fsm/states/state_03_avoidance.hpp"
#include "fsm/states/state_04_stairs.hpp"
#include "fsm/states/state_07_detection.hpp"
#include "fsm/states/state_09_end_obs.hpp"
#include "fsm/states/state_10_finish.hpp"
#include <unitree/robot/channel/channel_factory.hpp>

using namespace ruikang::control;
using namespace ruikang::fsm;

int main() {
    std::cout << "[System] Starting Multimodal Inspection System V2.0..." << std::endl;

    // 1. UDP 接收线程
    UdpReceiver receiver(8080);
    receiver.start();

    // 2. 初始化 DDS 通道（必须在 lidar 和 driver 之前）
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");

    // 3. 实例化驱动和雷达
    RobotDriver  driver;
    
    driver.initConnection("eth0");
    //lidar.init();

    // 4. 状态机 — 完整比赛流程
    StateMachine brain(&driver, new State01Init());

    std::cout << "--- FSM Main Loop Started ---" << std::endl;

    while (true) {
        VisionData data = receiver.getLatestData();
        brain.update(data);
        usleep(10000);  // 10ms = 100Hz
    }

    receiver.stop();
    return 0;
}