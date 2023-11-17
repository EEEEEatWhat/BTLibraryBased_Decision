#pragma once

#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP
#define DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_msgs/action/behavior_tree_pose.hpp"

namespace decision_behavior_tree
{
using namespace BT;

    class GainBloodOrBulletAction : public RosActionNode<behaviortree_msgs::action::BehaviorTreePose>
    {
    public:
        GainBloodOrBulletAction(const std::string &name,
                                const NodeConfig &conf,
                                const RosNodeParams &params)
            : RosActionNode<behaviortree_msgs::action::BehaviorTreePose>(name, conf, params)
        {
        }


    public:
        static PortsList providedPorts()
        {
            return {OutputPort<geometry_msgs::msg::PoseStamped>("supply_pose")};
        }


        bool setGoal(RosActionNode::Goal &goal) override
        {   
            BT::Blackboard::Ptr blackboard = config().blackboard;
            goal.set__pose(blackboard->get<geometry_msgs::msg::PoseStamped>("supply_pose"));
            RCLCPP_INFO(node_->get_logger(),"Goal设置成功. . . ");
            return true;
        };


        NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            (void)wr;
            std::stringstream ss;
            ss << "Result received: ";

            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::SUCCESS;
        }


        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);

            return NodeStatus::FAILURE;
        }


        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            (void)feedback;
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            // for (auto number : feedback->partial_sequence) {
            //   ss << number << " ";
            // }
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return NodeStatus::RUNNING;
        }
    private:
        geometry_msgs::msg::PoseStamped goal_pose;
    };
}  // namespace decision_behavior_tree

#endif //DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP