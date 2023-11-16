#pragma once

#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP
#define DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/blackboard.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <thread>

namespace decision_behavior_tree
{
    class GainBloodOrBulletAction : public BT::StatefulActionNode
    {
    public:
        
        GainBloodOrBulletAction(const std::string& name, const BT::NodeConfig& config)
                                : BT::StatefulActionNode(name , config)
        {};

        ~GainBloodOrBulletAction()
        {};


        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<BT::NodeStatus>("result"),
                    BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_pose"),
                    BT::OutputPort<bool>("if_supply")}; 
        };
    private:
        /// Method called once, when transitioning from the state IDLE.
        /// If it returns RUNNING, this becomes an asynchronous node.
        BT::NodeStatus onStart()
        {
            BT::Blackboard::Ptr blackboard = config().blackboard;
            setOutput<geometry_msgs::msg::PoseStamped>("supply_pose", blackboard->get<geometry_msgs::msg::PoseStamped>("supply_pose"));
            setOutput<bool>("if_supply", true); // 动作结束后改为false
            return BT::NodeStatus::RUNNING;
        };

        /// method invoked when the action is already in the RUNNING state.
        BT::NodeStatus onRunning()
        {
            if (getInput<BT::NodeStatus>("result") == BT::NodeStatus::SUCCESS)
            {
                setOutput<bool>("if_supply", false);
                return BT::NodeStatus::SUCCESS;
            }
            else 
                return BT::NodeStatus::RUNNING;

        };
        

        /// when the method halt() is called and the action is RUNNING, this method is invoked.
        /// This is a convenient place todo a cleanup, if needed.
        void onHalted()
        {
            setOutput<bool>("if_supply", false);
        };

    };
}  // namespace decision_behavior_tree

#endif //DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP