#pragma once

#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "global_interfaces/action/go_goal.hpp"

namespace decision_behavior_tree
{
    class GainBloodOrBulletAction : public BT::RosActionNode<global_interfaces::action::Go>
    {
    public:
        
        GainBloodOrBulletAction(const std::string& name, const BT::NodeConfig& config , const BT::RosNodeParams& params)
                               : BT::RosActionNode<global_interfaces::action::Go>(name , config, params)
        {};


        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<double>("goal")}; 
        };

        bool setGoal(RosActionNode::Goal &goal_) override ;

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

    };
}  // namespace decision_behavior_tree

#endif //DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_