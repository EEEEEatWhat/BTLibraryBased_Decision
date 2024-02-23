#ifndef FSM_HPP_
#define FSM_HPP_

// FSM状态项
class FSMItem
{
    friend class FSM;

public:
    // 枚举所有状态
    enum GimbalState
    {
        GIMBAL_READY = 0,       // 准备状态
        GIMBAL_SEARCH,          // 搜索状态
        GIMBAL_FIGHT_BACK,      // 反击状态
        GIMBAL_SHOOT,           // 攻击状态
    };

    enum ChassisState
    {
        CHASSIS_READY = 0,          // 准备状态
        CHASSIS_RELOCATION,         // 重定位状态
        CHASSIS_LOW_SPEED_TUOLUO,   // 低速小陀螺状态
        CHASSIS_NAV,                // 导航状态
        CHASSIS_AVOID,              // 躲避状态
    };

    enum RobotState
    {
        ROBOT_READY = 0,                 // 准备状态
        ROBOT_PATROL,                    // 巡逻状态
        ROBOT_ATTACK,                    // 进攻状态
        ROBOT_FIGHT_BACK,                // 反击状态
        ROBOT_SEARCH_FIXED_POINT,        // 定点搜索状态
    };

    // 枚举所有事件
    enum Events
    {
        EVENT_READY = 0,        // 准备事件
        EVENT_OPENING_STAGE,    // 开场事件
        EVENT_OUTPOST_DEAD,     // 前哨站被击毁事件

    };
private:
    RobotState _current_state;
    RobotState _next_state;
    Events _event;

    // 动作函数    


public:
    //初始化构造函数(初始化列表方式赋值)
	// FSMItem(State curState, Events event, void(*action)(), State nextState)
	// 	:_curState(curState), _event(event), _action(action), _nextState(nextState) { }
    // ~FSMItem();
};


#endif