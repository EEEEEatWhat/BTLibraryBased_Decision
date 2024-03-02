#ifndef PATROL_2_HPP
#define PATROL_2_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"

namespace robot_decision
{
    class Patrol_2 : public BT::RosActionNode<global_interfaces::action::BehaviorTreePose>
    {
    public:
        Patrol_2(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<global_interfaces::action::BehaviorTreePose>(name, conf, params)
        {
            blackboard_ = config().blackboard;
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {   
            sleep(2);
            std::cout<<blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_startup_pose").pose.position.x;
            goal.set__pose(blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_startup_pose"));
            RCLCPP_INFO(node_->get_logger(),"Goal2设置成功. . . ");
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
            std::cout << "任务2执行完毕..." << '\n';
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

            // RCLCPP_INFO(node_->get_logger(), "Feedback: remaining distance = %f ", feedback->distance_remaining);
            return BT::NodeStatus::RUNNING;
        };
        
    private:
        BT::Blackboard::Ptr blackboard_;
    };
}  // namespace robot_decision

#endif //PATROL_2_HPP