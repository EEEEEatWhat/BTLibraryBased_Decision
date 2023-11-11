#pragma once

#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"


namespace decision_behavior_tree
{
    class GainBloodOrBulletAction : public BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>
    {   
    public:
        
        GainBloodOrBulletAction(const std::string& name, const BT::NodeConfig& config , const BT::RosNodeParams& params)
                                : BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name , config, params)
        {
            std::cout <<"启动补血补弹---------" << std::endl;
        };

        static BT::PortsList providedPorts()
        {
            // return { BT::InputPort<double>("goal")}; //可以嘎了
            return {};
        };
        
        bool setGoal(RosActionNode::Goal &goal_) override 
        {
            // 补给区坐标写在config文件里，初始化时加载参数 在这里调取参数然后设置目标值 poses和behavior_tree
            
            
            auto goal_msg = std::make_shared<nav2_msgs::action::NavigateThroughPoses::Goal>();
             // 创建geometry_msgs::msg::PoseStamped消息的向量
            std::vector<geometry_msgs::msg::PoseStamped> poses;
            // 创建一个PoseStamped消息
            geometry_msgs::msg::PoseStamped pose1;
            // 设置PoseStamped消息的header字段 坐标系通常是map
            pose1.header.frame_id = "map";
            pose1.header.stamp = rclcpp::Clock().now();
            this->nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
            // 设置PoseStamped消息的pose字段
            pose1.pose.position.x = 2.0;
            pose1.pose.position.y = -0.1;
            // pose1.pose.position.z = 0.00000226632;
            // // 设置姿态信息（四元数表示）
            // pose1.pose.orientation.x = 0.0;
            // pose1.pose.orientation.y = 0.0;
            // pose1.pose.orientation.z = 0.0;
            pose1.pose.orientation.w = 1.0;
            poses.push_back(pose1); 
            goal_msg->poses = poses;


            goal_.set__poses(poses);
            return true;
        }

        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            // wr.result->result 怎么是空消息类型阿
            if (goal_handle_future.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
            {
                if (goal_handle_future.get()->get_status() == rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
                {
                    std::cout << "导航完成" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    std::cout << "导航失败" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }
            else
            {
                std::cout << "无法等待导航完成" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
            auto now_pos = feedback->current_pose;
            std::cout << "now_pos_x:" << now_pos.pose.position.x << std::endl ;
            std::cout << "now_pos_y:" << now_pos.pose.position.y << std::endl ;
            std::cout << "now_ori:w" << now_pos.pose.orientation.w << std::endl ;
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override 
        {
            std::cout << "Error:" << error << std::endl;
            return BT::NodeStatus::FAILURE;
        }

    // CreateRosNodePlugin(GainBloodOrBulletAction, "go_blood_or_bullet");// 还得配合插件 
    private:
        rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_through_poses_action_client_;
        nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
        rclcpp::Subscription<nav2_msgs::action::NavigateThroughPoses::Impl::GoalStatusMessage>::SharedPtr
            nav_through_poses_goal_status_sub_;
        std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr> goal_handle_future;

    };


}  // namespace decision_behavior_tree



#endif //DECISION_BEHAVIORS__GAIN_BLOOD_OR_BULLET_ACTION_HPP_
