#ifndef VELOCITY_CTRL_HPP
#define VELOCITY_CTRL_HPP
#include <cmath>

namespace ruikang {
namespace control {

// 统一输出给底层机器狗的速度结构体
struct VelocityCmd {
    float vx;
    float vy;
    float vyaw;
};

class VelocityCtrl {
public:
    float Kp_yaw;
    float Ki_yaw;
    float Kd_yaw;

    float max_vyaw;
    float last_offset;
    float error_sum;
    float max_error_sum;
    float smoothed_vyaw;
    
    // 🌟 迟滞长效记忆（彻底解决提前丢线问题）
    float turn_memory; 
    bool is_first_run;

    VelocityCtrl() {
        Kp_yaw = 0.0060f;
        Ki_yaw = 0.0003f;
        Kd_yaw = 0.0050f;

        max_vyaw = 1.0f;   
        max_error_sum = 1000.0f; 
        reset();
    }

    void reset() {
        last_offset = 0.0f;
        error_sum = 0.0f;
        smoothed_vyaw = 0.0f;
        turn_memory = 0.0f; // 状态清零
        is_first_run = true;
    }

    float computeYaw(float current_offset) {
        if (is_first_run) {
            last_offset = current_offset;
            is_first_run = false;
        }

        // 🌟🌟🌟 1. 更新长效记忆 (迟滞区间逻辑) 🌟🌟🌟
        if (current_offset > 30.0f) {
            turn_memory = 300.0f;  // 确认为右大弯，死死记住
        } else if (current_offset < -30.0f) {
            turn_memory = -300.0f; // 确认为左大弯，死死记住
        } else if (std::abs(current_offset) < 10.0f) {
            turn_memory = 0.0f;    // 只有完美回到直道中心，才允许遗忘！
        }

        // 🌟🌟🌟 2. 丢线救场与积分急停 🌟🌟🌟
        if (std::abs(current_offset) < 1.0f) {
            error_sum = 0.0f; // 【绝杀】：瞬间清空积分，彻底消灭“往右偏的幽灵”！
            current_offset = turn_memory; // 提取长效记忆！有弯就转，没弯直走！
        } 
        // 正常条件积分
        else if (std::abs(current_offset) < 60.0f) {
            error_sum += current_offset;
            if (error_sum > max_error_sum) error_sum = max_error_sum;
            if (error_sum < -max_error_sum) error_sum = -max_error_sum;
        } else {
            error_sum = 0.0f; 
        }

        // 3. 防止寻回黑线时的“微分回踢”
        float derivative = 0.0f;
        if (std::abs(last_offset) >= 200.0f && std::abs(current_offset) < 100.0f) {
            derivative = 0.0f; // 刚从盲区找回黑线，忽略这一帧的突变防止抽搐
        } else {
            derivative = current_offset - last_offset;
        }
        
        float raw_vyaw = - (Kp_yaw * current_offset + Ki_yaw * error_sum + Kd_yaw * derivative);
        last_offset = current_offset;

        // 限幅
        if (raw_vyaw > max_vyaw) raw_vyaw = max_vyaw;
        if (raw_vyaw < -max_vyaw) raw_vyaw = -max_vyaw;

        // 动态线性滤波
        float abs_off = std::abs(current_offset);
        float filter_alpha = 0.25f + (0.30f * (abs_off / 150.0f)); 
        if (filter_alpha > 0.50f) filter_alpha = 0.50f;

        smoothed_vyaw = smoothed_vyaw + filter_alpha * (raw_vyaw - smoothed_vyaw);

        return smoothed_vyaw;
    }

    // ==========================================
    // 5.8 新增：供状态机直接调用的封装接口
    // ==========================================

    // 台阶专用：恒定低速前进，不转向，防跌落
    VelocityCmd getStairClimbVelocity();

    // 正常巡线：PID 偏航 + 动态线速度（弯道自动减速）
    VelocityCmd getNormalTrackingVelocity(float current_offset, float base_vx = 0.12f);

    // 紧急刹车
    VelocityCmd getStopVelocity();
};

} 
} 
#endif