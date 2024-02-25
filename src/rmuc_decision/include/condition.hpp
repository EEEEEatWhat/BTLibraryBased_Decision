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
                RCLCPP_INFO(node_->get_logger(),"open stage strategy hasn't set!");
                return BT::NodeStatus::FAILURE;
            }
            // 先检查双方前哨站状态，再判断是否需要设置openstage_strategy（不需要的话设为5）
            if(Check_outpost_status() == BT::NodeStatus::SUCCESS)
            {
                int openstage_strategy;
                this->declare_parameter<int>("openstage_strategy", -1);
                this->get_parameter<int>("openstage_strategy", openstage_strategy);
                blackboard_->set<int>("openstage_strategy", openstage_strategy);
                RCLCPP_INFO(node_->get_logger(),"open stage strategy has set to %d.",blackboard_->get<int>("openstage_strategy"));
                return BT::NodeStatus::SUCCESS;
            }
            blackboard_->set<int>("openstage_strategy", 5);
            RCLCPP_INFO(node_->get_logger(),"open stage strategy is set to 5, skip the switch_node!");
            return BT::NodeStatus::SUCCESS;
        };

        

        BT::NodeStatus Default_skip()
        {
            // when open stage is set to -1 or game is in middle stage
            // then skip the switch node.
            return BT::NodeStatus::SUCCESS;
        };

        // 在Check_openstage_strategy中调用，检查前哨站状态来调整openstage_strategy，确保比赛中期跳过switch_node
        BT::NodeStatus Check_outpost_status()
        {
            // TODO：双方前哨站都没掉时返回SUCCESS，有一方掉了都返回FAILURE
            // 在blackboard中设置双方前哨站状态
        };

        BT::NodeStatus Check_Nav_status()
        {

        };

        BT::NodeStatus Check_is_go_enemy_door()
        {
            geometry_msgs::msg::PoseStamped enemy_door_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_door");
            // TODO：从裁判系统中获取本机器人位置x y
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
            // TODO：从裁判系统中获取本机器人位置x y
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
            // TODO：从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-our_outpost_pose.pose.position.x, 2) + pow(y-our_outpost_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_game_running()
        {
            // TODO：从裁判系统中获取前哨站状态消息（如果可以 ）
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
            // TODO：从裁判系统中获取本机器人位置x y
            double x,y;
            if(pow(x-keystone_heights_pose.pose.position.x, 2) + pow(y-keystone_heights_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_is_set_game_status_as_last()
        {
            // TODO：从裁判系统中获取当前阶段剩余时间，在最后一两分钟按云台手标点走
            
        }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
    };
}
#endif