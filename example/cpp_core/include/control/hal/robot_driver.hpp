#ifndef ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_HPP

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <iostream>
#include <string>

namespace ruikang {
namespace control {

class RobotDriver {
private:
    // 改成指针类型，避免在构造函数中被提前实例化，以确保网络通道优先建立
    unitree::robot::go2::SportClient* sport_client; 
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
    
    // 响应突发最高优中断（如：警示牌触发的伸懒腰）
    void playSpecialAction(const std::string& action_type); 

    // 紧急阻尼保护
    void emergencyDamp(); 
};

} // namespace control
} // namespace ruikang

#endif // ROBOT_DRIVER_HPP