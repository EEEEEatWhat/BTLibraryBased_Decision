#pragma once

#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "global_interfaces/action/go.hpp"
#include "behaviortree_ros2/plugins.hpp"


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

        bool setGoal(RosActionNode::Goal &goal_) override 
        {
            auto pos_ = getInput<double>("goal");
            // goal_.goal = pos_.value();
            goal_.goal = 1.0;                   // 设置目标 
                        
            if(goal_.goal!=0)
            {
                std::cout << "成功设置目标点坐标----------" << std::endl;
                return true;
            }
            else
            {
                std::cout << "设置目标点坐标失败----------" << std::endl;
                return false;
            }
            
        }

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override
        {
        
        if(wr.result->done == true)
        std::cout << name().c_str() << ": onResultReceived. Done = true" << std::endl;
        else
        std::cout << name().c_str() << ": onResultReceived. Done = false" << std::endl;

        return wr.result->done ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
            double now_pos = feedback->current_pos;
            std::cout << "now_pos:" << now_pos << std::endl ;
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override 
        {
            std::cout << "Error:" << error << std::endl;
            return BT::NodeStatus::FAILURE;
        }

    // CreateRosNodePlugin(GainBloodOrBulletAction, "go_blood_or_bullet");// 还得配合插件 
    };
}  // namespace decision_behavior_tree



#endif //DECISION_BEHAVIORS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_
