
#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP
#define DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"

namespace decision_behavior_tree
{
    class GainBloodOrBulletAction : public BT::RosActionNode<global_interfaces::action::BehaviorTreePose>
    {
    public:
        GainBloodOrBulletAction(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<global_interfaces::action::BehaviorTreePose>(name, conf, params)
        {
        };

        


        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_pose")});

            // return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("supply_pose")};
        };


        bool setGoal(RosActionNode::Goal &goal) override
        {   
            BT::Blackboard::Ptr blackboard = config().blackboard;
            goal.set__pose(blackboard->get<geometry_msgs::msg::PoseStamped>("supply_pose"));
            RCLCPP_INFO(node_->get_logger(),"Goal设置成功. . . ");
            return true;
        };


        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            (void)wr;
            std::stringstream ss;
            ss << "Result received: ";

            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return BT::NodeStatus::SUCCESS;
        };


        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));

            return BT::NodeStatus::FAILURE;
        };


        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            (void)feedback;
            std::stringstream ss;
            ss << "Next number in sequence received: ";
            // for (auto number : feedback->partial_sequence) {
            //   ss << number << " ";
            // }
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
            return BT::NodeStatus::RUNNING;
        };
    private:
        std::shared_ptr<rclcpp::Node> node_;
        geometry_msgs::msg::PoseStamped goal_pose;
    };
}  // namespace decision_behavior_tree

#endif //DECISION_BEHAVIOR_TREE__PLUGINS__ACTION__GAIN_BLOOD_OR_BULLET_ACTION_HPP