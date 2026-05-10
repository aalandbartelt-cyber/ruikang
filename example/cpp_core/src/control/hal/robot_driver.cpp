#include "control/hal/robot_driver.hpp"
#include <unitree/robot/channel/channel_factory.hpp> // 必须引入底层的通道工厂
#include <unistd.h> // 用于 usleep 阻塞控制

namespace ruikang {
namespace control {

RobotDriver::RobotDriver() {
    sport_client = nullptr;
    vui_client   = nullptr;
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
        delete vui_client;
    }
    std::cout << "[RobotDriver] Destroyed safely. Hardware protected." << std::endl;
}

bool RobotDriver::initConnection(const std::string& network_interface) {
    // 关键步骤：在实例化客户端之前，必须先打通底层 DDS 通道
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    
    sport_client = new unitree::robot::go2::SportClient();
    sport_client->SetTimeout(10.0f);
    sport_client->Init();

    vui_client = new unitree::robot::go2::VuiClient();
    vui_client->SetTimeout(1.0f);
    vui_client->Init();

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

    // 刹车防摔
    this->move(0.0f, 0.0f, 0.0f);
    usleep(150000);

    if (action_type == "stretch" || action_type == "ELECTRIC") {
        std::cout << "[RobotDriver] 非阻塞伸懒腰 (stretch)" << std::endl;
        sport_client->Stretch();
    } else if (action_type == "greet" || action_type == "OXIDANT") {
        std::cout << "[RobotDriver] 非阻塞打招呼 (greet)" << std::endl;
        sport_client->Hello();
    } else if (action_type == "flash_lights" || action_type == "RADIATION") {
        std::cout << "[RobotDriver] 非阻塞闪灯 (flash_lights)" << std::endl;
        // 宇树 SDK 无直接闪灯 API，通过灯光控制模拟
        // sport_client->LightColor(0, 255, 0);
    } else {
        std::cout << "[RobotDriver] 未知动作: " << action_type << std::endl;
    }
}

void RobotDriver::emergencyDamp() {
    if (!is_connected || sport_client == nullptr) return;
    sport_client->Damp();
    std::cout << "[RobotDriver] ! WARNING ! Damping Mode Activated." << std::endl;
}

// ==========================================
// 5.8 新增：任务级纯非阻塞动作实现
// ==========================================

void RobotDriver::setGait(GaitType gait) {
    if (!is_connected || sport_client == nullptr) return;
    if (gait == GaitType::GAIT_CLASSIC) {
        std::cout << "[RobotDriver] 切换至 CLASSIC（经典）步态" << std::endl;
        sport_client->ClassicWalk(true);
    } else if (gait == GaitType::GAIT_AGILE) {
        std::cout << "[RobotDriver] 切换至 AGILE（灵动/Free）步态" << std::endl;
        sport_client->ClassicWalk(false);  // 退出经典 = 回到 Free/Agile 模式
    }
}

void RobotDriver::stretch() {
    if (!is_connected || sport_client == nullptr) return;
    std::cout << "[RobotDriver] 执行非阻塞伸懒腰..." << std::endl;
    sport_client->Stretch();
}

void RobotDriver::greet() {
    if (!is_connected || sport_client == nullptr) return;
    std::cout << "[RobotDriver] 执行非阻塞打招呼..." << std::endl;
    sport_client->Hello();
}

void RobotDriver::flashLights() {
    if (!is_connected || vui_client == nullptr) {
        std::cout << "[RobotDriver] ⚠️ VuiClient 未就绪，无法闪灯" << std::endl;
        return;
    }
    std::cout << "[RobotDriver] 💡 前灯闪烁 3 次..." << std::endl;

    // 3 次闪烁：亮 250ms → 灭 250ms，总计 1.5s
    for (int i = 0; i < 3; i++) {
        vui_client->SetBrightness(10);   // 最亮
        usleep(250000);
        vui_client->SetBrightness(0);    // 熄灭
        usleep(250000);
    }
    std::cout << "[RobotDriver] ✅ 闪灯完成" << std::endl;
}

void RobotDriver::jumpObstacle() {
    if (!is_connected || sport_client == nullptr) return;
    std::cout << "[RobotDriver] 执行非阻塞前跳..." << std::endl;
    sport_client->FrontJump();
}

void RobotDriver::lockMotors() {
    emergencyDamp();
}

void RobotDriver::stopMove() {
    move(0.0f, 0.0f, 0.0f);
}

} // namespace control
} // namespace ruikang