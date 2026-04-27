// 状态基类：定义纯虚函数 enter(), execute(), exit()
#pragma once

namespace ruikang {
namespace fsm {

// 前向声明，告诉编译器有这么个类，避免头文件互相包含报错
class StateMachine; 

class StateBase {
public:
    virtual ~StateBase() = default;
    
    // 进入该状态时，只执行一次（用于初始化动作，如：命令狗起跳）
    virtual void enter(StateMachine* sm) = 0; 
    
    // 在该状态期间，主循环每帧都在调用它（用于持续检测，如：是否看到了标识？）
    virtual void execute(StateMachine* sm) = 0; 
    
    // 离开该状态时，只执行一次（用于清理工作，如：恢复正常步态）
    virtual void exit(StateMachine* sm) = 0;    
};

} // namespace fsm
} // namespace ruikang