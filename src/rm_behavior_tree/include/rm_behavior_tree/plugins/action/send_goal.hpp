#ifndef SEND_GOAL_HPP
#define SEND_GOAL_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree{
    class SendGoal : public BT::RosActionNode<global_interfaces::action::BehaviorTreePose>
    {
    public:
        SendGoal(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<global_interfaces::action::BehaviorTreePose>(name, conf, params)
        {
            blackboard_ = config().blackboard;
        };

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("action_name"),
                BT::InputPort<std::string>("goal_name"),
            };
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {
            std::string goal_name;
            getInput("goal_name", goal_name);
            goal.set__pose(blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name));
            RCLCPP_INFO(node_->get_logger(),"set goal %s successfully...", goal_name.c_str());
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
            (void)feedback;
            return BT::NodeStatus::RUNNING;
        };
        
    private:
        BT::Blackboard::Ptr blackboard_;
    };
}

#endif //SEND_GOAL_HPP