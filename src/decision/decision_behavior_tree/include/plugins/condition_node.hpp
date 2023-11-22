#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__CONDITION_NODE_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__CONDITION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"


namespace decision_behavior_tree{
    /* 提供检查状态的各项接口 */
    class Condition
    {
    private:
        BT::Blackboard::Ptr blackboard_;
        float lowest_blood;
        float current_blood; 
    public:
        Condition(BT::Blackboard::Ptr blackboard_): blackboard_(blackboard_)
        {

        }
        BT::NodeStatus CheckBlood()
        {
            // lowest_blood = blackboard_->get<float>("lowest_blood");
            // current_blood = // 裁判系统 参数服务器
            lowest_blood = 100;
            current_blood = 50;
            if (current_blood < lowest_blood)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                return BT::NodeStatus::FAILURE;
        }
        ~Condition()
        {}
    };
    

    
}
#endif