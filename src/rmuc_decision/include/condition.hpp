#ifndef CONDITION_HPP
#define CONDITION_HPP

#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"

namespace rmuc_decision {
    class Condition {
    public:
        Condition(BT::Blackboard::Ptr blackboard_): blackboard_(blackboard_){
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("node");
        };

        BT::NodeStatus Check_game_started()
        {

            if (blackboard_->get<uint8_t>("game_progress") == 4)
            {
                RCLCPP_INFO(node_->get_logger(),"game start!");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(node_->get_logger(),"game hasn't started yet!");
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_openstage_strategy()
        {
            if(blackboard_->get<int>("openstage_strategy") == -1)
            {
                RCLCPP_INFO(node_->get_logger(),"open stage strategy isn't set!");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node_->get_logger(),"open stage strategy has set to %d.",blackboard_->get<int>("openstage_strategy"));
            return BT::NodeStatus::SUCCESS;
        };

        // TODO：如果某一方前哨站掉了需要考虑在某个地方将openstage_strategy设为5,使switch跳过
        // 考虑一下检查前哨站状态的节点放在哪

        BT::NodeStatus Default_skip()
        {
            // when open stage is set to -1 or game is in middle stage (open stage strategy = 5)
            // then skip the switch node.
            return BT::NodeStatus::SUCCESS;
        };

        BT::NodeStatus Check_outpost_status()
        {
            
        };

        BT::NodeStatus Check_Nav_status()
        {

        };

        BT::NodeStatus Check_is_go_enemy_door()
        {
            geometry_msgs::msg::PoseStamped enemy_door_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_door");
            // 从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-enemy_door_pose.pose.position.x, 2) + pow(y-enemy_door_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        };

        BT::NodeStatus Check_is_go_enemy_outpost()
        {
            geometry_msgs::msg::PoseStamped enemy_outpost_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_outpost");
            // 从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-enemy_outpost_pose.pose.position.x, 2) + pow(y-enemy_outpost_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_is_go_our_outpost()
        {
            geometry_msgs::msg::PoseStamped our_outpost_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("our_outpost");
            // 从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-our_outpost_pose.pose.position.x, 2) + pow(y-our_outpost_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_game_running()
        {
            // 从裁判系统中获取前哨站状态消息（如果可以 ）
            if(0)
            {
                blackboard_->set<bool>("game_running", true);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus Check_is_go_keystone_heights()
        {
            geometry_msgs::msg::PoseStamped keystone_heights_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("keystone_heights");
            // 从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-keystone_heights_pose.pose.position.x, 2) + pow(y-keystone_heights_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_is_set_game_status_as_last()
        {
            // 从裁判系统中获取当前阶段剩余时间，在最后一两分钟按云台手标点走
            
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
    };
}
#endif