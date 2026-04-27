#include "control/hal/robot_driver.hpp"
// 【极其重要】：必须引入宇树底层通道工厂的头文件！
#include <unitree/robot/channel/channel_factory.hpp>

namespace ruikang {
namespace control {

RobotDriver::RobotDriver() {
    sport_client = nullptr; // 指针初始化为空
    is_connected = false;
    std::cout << "[RobotDriver] Initialized. Awaiting hardware connection..." << std::endl;
}

RobotDriver::~RobotDriver() {
    if (is_connected && sport_client != nullptr) {
        std::cout << "[RobotDriver] Initiating graceful shutdown sequence..." << std::endl;
        
        // 第一步：强行刹车，确保狗不再有速度
        sport_client->Move(0.0f, 0.0f, 0.0f);
        usleep(500000); // 稍微缓冲 0.5 秒

        // 第二步：调用 SDK 的平缓趴下指令
        std::cout << "[RobotDriver] Standing down smoothly..." << std::endl;
        sport_client->StandDown();
        
        // 第三步：【极其重要】给狗留出 3 秒钟的物理执行时间！
        // 因为代码执行是一瞬间的，如果不 sleep，代码会立刻跑到下一步切断动力，狗还是会摔。
        sleep(3); 

        // 第四步：狗已经稳稳趴在地上了，此时再切断电机力矩（进入阻尼）绝对安全
        emergencyDamp(); 
        
        // 释放指针内存
        delete sport_client;
    }
    std::cout << "[RobotDriver] Destroyed safely. Hardware protected." << std::endl;
}

bool RobotDriver::initConnection(const std::string& network_interface) {
    // 【修改点 1】：绝对的第一步！初始化底层通道
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    
    // 【修改点 2】：通道打通后，再动态实例化 SportClient
    sport_client = new unitree::robot::go2::SportClient();
    sport_client->SetTimeout(10.0f);
    sport_client->Init();
    
    std::cout << "[RobotDriver] 真实硬件连接成功！网卡: " << network_interface << std::endl;
    is_connected = true;
    return true;
}

void RobotDriver::move(float vx, float vy, float vyaw) {
    if (!is_connected || sport_client == nullptr) return;
    // 【修改点 3】：用指针箭头 -> 调用
    sport_client->Move(vx, vy, vyaw);
    std::cout << "[RobotDriver] 真实执行 MOVE -> vx: " << vx << std::endl;
}

void RobotDriver::jump() {
    if (!is_connected || sport_client == nullptr) return;
    sport_client->FrontJump(); 
    std::cout << "[RobotDriver] 真实执行 ACTION: JUMP!" << std::endl;
}

void RobotDriver::emergencyDamp() {
    if (!is_connected || sport_client == nullptr) return;
    sport_client->Damp();
    std::cout << "[RobotDriver] ! WARNING ! Damping Mode Activated." << std::endl;
}

} 
}