#ifndef WANDERING_HPP
#define WANDERING_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_send_poses.hpp"

namespace rmuc_decision
{
    class Wandering : public BT::RosActionNode<global_interfaces::action::BehaviorTreeSendPoses>
    {
    public:
        Wandering(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<global_interfaces::action::BehaviorTreeSendPoses>(name, conf, params)
        {
            blackboard_ = config().blackboard;
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {   
            goal.poses.push_back(blackboard_->get<geometry_msgs::msg::PoseStamped>("our_bunker_pose"));
            goal.poses.push_back(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_1"));
            goal.poses.push_back(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_2"));
            goal.poses.push_back(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_3"));
            RCLCPP_INFO(node_->get_logger(),"多点Goal设置成功. . . ");
            return true;
        };


        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_INFO(node_->get_logger()," 任务被中止...");
                    return BT::NodeStatus::FAILURE;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(node_->get_logger()," 任务被取消...");
                    return BT::NodeStatus::FAILURE;
                default:
                    RCLCPP_INFO(node_->get_logger()," 未知异常...");
                    return BT::NodeStatus::FAILURE;
            }
            std::cout << "任务执行完毕..." << '\n';
            return BT::NodeStatus::SUCCESS;
        };


        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
            return BT::NodeStatus::FAILURE;
        };


        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
            
            RCLCPP_INFO(node_->get_logger(), "Feedback: remaining distance = %f, pose = (%lf, %lf) ", feedback->distance_remaining,feedback->current_pose.pose.position.x,feedback->current_pose.pose.position.y);
            RCLCPP_INFO(node_->get_logger(), "Feedback: number of poses remaining = %d", feedback->number_of_poses_remaining);
            return BT::NodeStatus::RUNNING;
        };
        
    private:
        BT::Blackboard::Ptr blackboard_;
    };
}
#endif //WANDERING_HPP