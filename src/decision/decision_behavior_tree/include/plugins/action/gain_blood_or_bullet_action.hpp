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
    class GainBloodOrBulletAction : public BT::SyncActionNode
    {
    public:
        
        GainBloodOrBulletAction(const std::string& name, const BT::NodeConfig& config)
                                : BT::SyncActionNode(name , config)
        {};
        ~GainBloodOrBulletAction()
        {};


        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<BT::NodeStatus>("result"),
                    BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_pose"),
                    BT::OutputPort<bool>("if_supply")}; 
        };

        BT::NodeStatus tick() override
        {
            BT::Blackboard::Ptr blackboard = config().blackboard;
            setOutput<geometry_msgs::msg::PoseStamped>("supply_pose", blackboard->get<geometry_msgs::msg::PoseStamped>("supply_pose"));
            setOutput<bool>("if_supply", true); // 动作结束后改为false


            std::thread thread([&](){
            while (true)
            {
                if (getInput<BT::NodeStatus>("result") == BT::NodeStatus::SUCCESS)
                {
                    setOutput<bool>("if_supply", false);
                    break;
                }
                else std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            
            return BT::NodeStatus::SUCCESS;
            });
            thread.detach();
            return BT::NodeStatus::SUCCESS;


        }
    };
}  // namespace decision_behavior_tree

#endif //DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP