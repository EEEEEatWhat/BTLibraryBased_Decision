#ifndef SEND_GOAL_HPP
#define SEND_GOAL_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_decision_interfaces/action/behavior_tree_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree{
    class SendGoal : public BT::RosActionNode<rm_decision_interfaces::action::BehaviorTreePose>
    {
    public:
        SendGoal(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<rm_decision_interfaces::action::BehaviorTreePose>(name, conf, params)
        {
            blackboard_ = config().blackboard;
        };

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("action_name"),
            };
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {   
            goal.set__pose(blackboard_->get<geometry_msgs::msg::PoseStamped>("goal_pose"));
            RCLCPP_INFO(node_->get_logger(),"set goal successfully. . . ");
            return true;
        };


        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    std::cout << "Success!!!" << '\n';
                    return BT::NodeStatus::SUCCESS;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    std::cout << "Goal was aborted" << '\n';
                    return BT::NodeStatus::FAILURE;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    std::cout << "Goal was canceled" << '\n';
                    return BT::NodeStatus::FAILURE;
                    break;
                default:
                    std::cout << "Unknown result code" << '\n';
                    return BT::NodeStatus::FAILURE;
                    break;
            }
        };


        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
            return BT::NodeStatus::FAILURE;
        };


        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
            
            return BT::NodeStatus::RUNNING;
        };
        
    private:
        BT::Blackboard::Ptr blackboard_;
    };
}

#endif //SEND_GOAL_HPP