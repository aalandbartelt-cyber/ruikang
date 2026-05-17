#ifndef ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_HPP

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/go2/vui/vui_client.hpp>
#include <iostream>
#include <string>

namespace ruikang {
namespace control {

// 步态枚举
enum class GaitType {
    GAIT_CLASSIC = 0, // 经典步态（平地巡线）
    GAIT_AGILE   = 1  // 灵动步态（爬台阶）
};

class RobotDriver {
private:
    // 改成指针类型，避免在构造函数中被提前实例化，以确保网络通道优先建立
    unitree::robot::go2::SportClient* sport_client;
    unitree::robot::go2::VuiClient*   vui_client;   // 前灯控制
    bool is_connected;

public:
    RobotDriver();
    ~RobotDriver();
    
    // 基础连接与通信
    bool initConnection(const std::string& network_interface);
    
    // 基础运动控制
    void move(float vx, float vy, float vyaw);
    
    // ==========================================
    // 4.28 封装的高级物理动作 API (带安全阻塞)
    // ==========================================
    
    // 触发跨栏起跳（内含落地缓冲）
    void jump(); 
    
    // 原地精准 90 度转向（无视线盲走专用）
    void turn90Degree(bool is_left); 
    
    // 响应突发最高优中断（警示牌触发的伸懒腰/打招呼/闪灯）
    // ★ 非阻塞：下发指令后立即返回，由 ActionManager 轮询超时
    void playSpecialAction(const std::string& action_type);

    // 紧急阻尼保护
    void emergencyDamp();

    // ==========================================
    // 5.8 新增：任务级纯非阻塞动作 API
    // ==========================================

    // 步态切换（非阻塞）
    void setGait(GaitType gait);

    // 非阻塞伸懒腰 (ELECTRIC)
    void stretch();

    // 非阻塞打招呼 (OXIDANT)
    void greet();

    // 非阻塞闪灯 (RADIATION)
    void flashLights();

    // 非阻塞前跳
    void jumpObstacle();

    // 终点锁死电机（进入阻尼态）
    void lockMotors();

    // SDK 站立锁定（BalanceStand，电机保持力矩，狗站立不动）
    void balanceStand();

    // 紧急刹车（速度归零）
    void stopMove();
};

} // namespace control
} // namespace ruikang

#endif // ROBOT_DRIVER_HPP