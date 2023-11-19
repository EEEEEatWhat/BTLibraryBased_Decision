#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__IF_BLOOD_SUFFICIENT_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__IF_BLOOD_SUFFICIENT_HPP_


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace decision_behavior_tree
{
    class IfBloodSufficientCondition : public BT::SimpleConditionNode
    {
    private:
        BT::Blackboard::Ptr blackboard_;
        float lowest_blood;
        float current_blood; 

        BT::Blackboard::Ptr GetConfigBlackboard() const
        {
            return config().blackboard;
        }

    public:
        IfBloodSufficientCondition(const std::string& name, TickFunctor tick_functor, const BT::NodeConfig& conf)
                                    : BT::SimpleConditionNode(name, tick_functor, conf)
        {
            blackboard_ = config().blackboard;
            lowest_blood = blackboard_->get<float>("lowest_blood");
            // current_blood = // 裁判系统 参数服务器
            tick_functor = [this](TreeNode &node) -> BT::NodeStatus
            {
                blackboard_ = GetConfigBlackboard();
                if (current_blood < lowest_blood)
                {
                    return BT::NodeStatus::FAILURE;
                }
                return BT::NodeStatus::SUCCESS;
            };
        }
        
    };
    
    
}

#endif