#ifndef ROBOT_DECISION__INCLUDE__CONDITION_NODE_HPP_
#define ROBOT_DECISION__INCLUDE__CONDITION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"


namespace robot_decision{
    /* 提供检查状态的各项接口 */
    class Condition
    {
    private:
        BT::Blackboard::Ptr blackboard_;
        float lowest_blood;
        float current_blood; 
    public:
        Condition(BT::Blackboard::Ptr blackboard_): blackboard_(blackboard_)
        {}
        
        BT::NodeStatus Check_blood()
        {
            lowest_blood = blackboard_->get<float>("lowest_blood");
            // current_blood = // 裁判系统 参数服务器
            current_blood = 50;
            if (current_blood < lowest_blood)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus Check_enemy()
        {
            
        }

        BT::NodeStatus Check_game_status()
        {
            
        }

        ~Condition()
        {}
    };
    

    
}
#endif