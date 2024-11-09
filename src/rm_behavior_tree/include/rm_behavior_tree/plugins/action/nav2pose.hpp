#ifndef NAV2POSE_HPP
#define NAV2POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <queue>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateToPoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace rm_behavior_tree{
    class Nav2Pose : public BT::StatefulActionNode
    {
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
        NavigateToPoseGoalHandle::SharedPtr goal_handle_;
        int send_goal_timeout_;
        nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
        bool is_patrol_start;
        std::queue<geometry_msgs::msg::PoseStamped> patrol_points;

    public:
        Nav2Pose(const std::string &name, const BT::NodeConfig &conf) : BT::StatefulActionNode(name, conf){
            blackboard_ = config().blackboard;
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
            action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/BehaviorTreePose");
            send_goal_timeout_ = blackboard_->get<int>("send_goal_timeout_ms");
            is_patrol_start = false;
        };


        // this function is invoked once at the beginning.
        BT::NodeStatus onStart() override{
            auto goal_name = getInput<std::string>("goal_name");
            if (!goal_name){
                RCLCPP_ERROR(node_->get_logger(), "goal is not set");
                return BT::NodeStatus::FAILURE;
            }
            blackboard_->set<bool>("nav_mode", true);
            // blackboard_->set<bool>("search_mode", false);
            RCLCPP_INFO(node_->get_logger(),"goal name:%s", goal_name.value().c_str());
            if(goal_name == "patrol_points" || goal_name == "hero_patrol_points" || goal_name == "our_outpost_patrol_points"){
                if(!is_patrol_start){
                    is_patrol_start = true;
                    auto patrol_num = getInput<int>("patrol_num");
                    std::queue<geometry_msgs::msg::PoseStamped> temp_points = blackboard_->get<std::queue<geometry_msgs::msg::PoseStamped>>(goal_name.value());
                    for(int i=0; i<patrol_num.value(); i++){
                        patrol_points.push(temp_points.front());
                        temp_points.pop();
                    }
                }
                if(patrol_points.empty()){
                    RCLCPP_ERROR(node_->get_logger(),"patrol_points is empty!");
                    return BT::NodeStatus::FAILURE;
                }
                auto temp_pose = patrol_points.front();
                patrol_points.pop();
                patrol_points.emplace(temp_pose);
                // RCLCPP_INFO(node_->get_logger(),"goal:[%lf,%lf]",temp_pose.pose.position.x,temp_pose.pose.position.y);
                navigation_goal_.pose = temp_pose;
            } else {
                navigation_goal_.pose = blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name.value());
            }
            auto future_goal_handle = action_client_->async_send_goal(navigation_goal_);
            if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) != rclcpp::FutureReturnCode::SUCCESS){
                RCLCPP_ERROR(node_->get_logger(), "send goal failed");
                return BT::NodeStatus::FAILURE;
            }

            goal_handle_ = future_goal_handle.get();
            if (!goal_handle_){
                RCLCPP_ERROR(node_->get_logger(), "goal handle is null");
                return BT::NodeStatus::FAILURE;
            }

            return BT::NodeStatus::RUNNING;
                        
        };

        // If onStart() returned RUNNING, we will keep calling
        // this method until it return something different from RUNNING
        BT::NodeStatus onRunning() override{
            switch (goal_handle_->get_status()){
            case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_UNKNOWN");
                break;
            case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_ACCEPTED");
                break;
            case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_EXECUTING");
                break;
            case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELING");
                break;
            case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_SUCCEEDED");
                break;
            case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELED");
                break;
            case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_ABORTED");
                break;
            default:
                RCLCPP_INFO(node_->get_logger(), "goal status: ERROR CODE");
                break;
            }
            if (goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED){
                blackboard_->set<bool>("nav_mode", false);
                // blackboard_->set<bool>("search_mode", true);
                return BT::NodeStatus::SUCCESS;
            } else if (goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
                    goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_CANCELED){
                blackboard_->set<bool>("nav_mode", false);
                // blackboard_->set<bool>("search_mode", true);
                return BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::RUNNING;
            }
        };

        // callback to execute if the action was aborted by another node
        void onHalted() override{
            RCLCPP_INFO(node_->get_logger(), "goal halted");
            if (goal_handle_){
                auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
                if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_ERROR(node_->get_logger(), "cancel goal failed");
                }
                RCLCPP_INFO(node_->get_logger(), "goal canceled");
            }
        };

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("goal_name"),
                BT::InputPort<int>("patrol_num"),
            };
        };

        ~Nav2Pose(){};
    };
    

}

#endif