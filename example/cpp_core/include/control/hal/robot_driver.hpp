#ifndef ROBOT_DRIVER_HPP
#define ROBOT_DRIVER_HPP

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <iostream>
#include <string>

namespace ruikang {
namespace control {

class RobotDriver {
private:
    // 【修改点】：改成指针类型！避免它在构造函数中被提前实例化
    unitree::robot::go2::SportClient* sport_client; 
    bool is_connected;

public:
    RobotDriver();
    ~RobotDriver();
    bool initConnection(const std::string& network_interface);
    void move(float vx, float vy, float vyaw);
    void jump();
    void emergencyDamp(); 
};

} 
} 
#endif