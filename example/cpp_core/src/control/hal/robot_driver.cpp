#include "control/hal/robot_driver.hpp"
#include <unitree/robot/channel/channel_factory.hpp> // 必须引入底层的通道工厂
#include <unistd.h> // 用于 usleep 阻塞控制

namespace ruikang {
namespace control {

RobotDriver::RobotDriver() {
    sport_client = nullptr; 
    is_connected = false;
    std::cout << "[RobotDriver] Initialized. Awaiting hardware connection..." << std::endl;
}

RobotDriver::~RobotDriver() {
    if (is_connected && sport_client != nullptr) {
        std::cout << "[RobotDriver] Initiating graceful shutdown sequence..." << std::endl;
        
        // 1. 强行刹车，清空残余动量
        sport_client->Move(0.0f, 0.0f, 0.0f);
        usleep(500000); // 缓冲 0.5 秒

        // 2. 平缓趴下
        std::cout << "[RobotDriver] Standing down smoothly..." << std::endl;
        sport_client->StandDown();
        
        // 3. 阻塞等待狗安全触地 (3 秒)
        sleep(3); 

        // 4. 切断力矩进入阻尼态，彻底保护电机
        emergencyDamp(); 
        
        delete sport_client;
    }
    std::cout << "[RobotDriver] Destroyed safely. Hardware protected." << std::endl;
}

bool RobotDriver::initConnection(const std::string& network_interface) {
    // 关键步骤：在实例化客户端之前，必须先打通底层 DDS 通道
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    
    sport_client = new unitree::robot::go2::SportClient();
    sport_client->SetTimeout(10.0f);
    sport_client->Init();
    
    std::cout << "[RobotDriver] 真实硬件连接成功！网卡: " << network_interface << std::endl;
    is_connected = true;
    return true;
}

void RobotDriver::move(float vx, float vy, float vyaw) {
    if (!is_connected || sport_client == nullptr) return;
    sport_client->Move(vx, vy, vyaw);
}

// ==========================================
// 高级动作具体实现：包含物理时间补偿与防摔设计
// ==========================================

void RobotDriver::jump() {
    if (!is_connected || sport_client == nullptr) return;
    
    // 起跳前强制刹车，防止由于惯性导致的起跳方向偏移
    this->move(0.0f, 0.0f, 0.0f);
    usleep(100000); // 给底盘 0.1 秒的姿态复位时间

    // 下发前跳指令
    sport_client->FrontJump(); 
    std::cout << "[RobotDriver] ACTION: JUMP! 正在腾空与落地缓冲..." << std::endl;

    // 极其重要：阻塞当前进程 2.5 秒，保证状态机大脑此时不会下发其他行走指令，防止落地瞬间失控翻车
    usleep(2500000); 
    std::cout << "[RobotDriver] JUMP 落地平稳，状态机恢复控制权。" << std::endl;
}

void RobotDriver::turn90Degree(bool is_left) {
    if (!is_connected || sport_client == nullptr) return;

    // 🌟 修复底层安全保护断联：采用心跳式循环下发指令 🌟
    // 角速度标定：0.5 rad/s 下转 90 度 (1.5708 rad) 需要约 3.14 秒
    float vyaw = is_left ? 0.5f : -0.5f;
    float total_duration = 3.14159f; // 总旋转时间（秒）
    float hz = 50.0f;               // 刷新频率：50Hz (即每 20ms 下发一次指令)
    
    int total_steps = static_cast<int>(total_duration * hz);
    int step_us = static_cast<int>(1000000.0f / hz); // 每一步阻塞的微秒数 (20000us)

    std::cout << "[RobotDriver] 正在通过循环刷新模式执行原地 90 度自转，方向: " << (is_left ? "左" : "右") << std::endl;
    
    // 使用循环不断下发 Move 指令以维持底层“心跳”
    for (int i = 0; i < total_steps; ++i) {
        this->move(0.0f, 0.0f, vyaw);
        usleep(step_us); 
    }

    // 时间一到，强制截断角速度，精确定位防过冲
    this->move(0.0f, 0.0f, 0.0f);
    std::cout << "[RobotDriver] 90 度自转循环结束，底盘已锁死。" << std::endl;
}

void RobotDriver::playSpecialAction(const std::string& action_type) {
    if (!is_connected || sport_client == nullptr) return;

    // 不管之前在干什么，接管后第一步永远是刹车防摔
    this->move(0.0f, 0.0f, 0.0f);
    usleep(150000); 

    if (action_type == "ELECTRIC") {
        std::cout << "[RobotDriver] !!! 触发最高优中断 ELECTRIC: 执行伸懒腰 !!!" << std::endl;
        
        // 宇树底层伸懒腰动作
        sport_client->Stretch();
        
        // 伸懒腰动作较长，阻塞等待 3.5 秒让动作做完
        usleep(3500000);
        
        std::cout << "[RobotDriver] 伸懒腰动作完毕，系统交还控制权。" << std::endl;
    } else {
        std::cout << "[RobotDriver] 警告：未知或未配置的特殊动作: " << action_type << std::endl;
    }
}

void RobotDriver::emergencyDamp() {
    if (!is_connected || sport_client == nullptr) return;
    sport_client->Damp();
    std::cout << "[RobotDriver] ! WARNING ! Damping Mode Activated." << std::endl;
}

} // namespace control
} // namespace ruikang