#ifndef CONDITION_HPP
#define CONDITION_HPP

#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "call_for_refereesystem.hpp"

namespace rmuc_decision {
    class Condition {
    public:
        Condition(BT::Blackboard::Ptr blackboard_): blackboard_(blackboard_){
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("node");
            call_for_refereesystem_node = std::make_shared<rmuc_decision::CallForRefereeSystem>(blackboard_);
        };

        BT::NodeStatus Check_game_started()
        {
            call_for_refereesystem_node->sendRequest(0x0001);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            uint8_t game_started = 4;
            if (blackboard_->get<uint8_t>("GameStatusStruct.game_progress") == game_started)
            {
                RCLCPP_INFO(node_->get_logger(),"game start!");
                blackboard_->set<std::string>("game_stage", "started");
                call_for_refereesystem_node->sendRequest(0x0201);
                while(!call_for_refereesystem_node->checkResponseReceived()) {
                    sleep(0.5);
                };
                uint8_t red_sentry_id = 7;
                if(blackboard_->get<uint8_t>("RobotStatusStruct.robot_id") == red_sentry_id) {
                    blackboard_->set<std::string>("my_color", "red");
                }
                blackboard_->set<std::string>("my_color", "blue");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(node_->get_logger(),"game hasn't started yet!");
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_openstage_strategy()
        {
            if(blackboard_->get<std::string>("openstage_strategy") == "-1")
            {
                RCLCPP_INFO(node_->get_logger(),"open stage strategy hasn't set!");
                return BT::NodeStatus::FAILURE;
            }
            // 先检查双方前哨站状态，再判断是否需要设置openstage_strategy（不需要的话设为5）
            this->Check_outpost_status();
            if(blackboard_->get<std::string>("outpost_status") == "ALL_ALIVE") {
                std::string openstage_strategy;
                node_->declare_parameter<std::string>("openstage_strategy", "-1");
                node_->get_parameter<std::string>("openstage_strategy", openstage_strategy);
                blackboard_->set<std::string>("openstage_strategy", openstage_strategy);
                RCLCPP_INFO(node_->get_logger(),"open stage strategy has set to %s.",blackboard_->get<std::string>("openstage_strategy").c_str());
                return BT::NodeStatus::SUCCESS;
            }
            blackboard_->set<std::string>("openstage_strategy", "5");
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
        void Check_outpost_status()
        {
            uint16_t our_outpost_HP;
            uint16_t enemy_outpost_HP;
            uint16_t dead_red_outpost_HP = 0;
            uint16_t dead_blue_outpost_HP = 0;
            call_for_refereesystem_node->sendRequest(0x0003);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            std::string my_color = blackboard_->get<std::string>("my_color");
            if(my_color == "red") {
                our_outpost_HP = blackboard_->get<uint16_t>("GameStatusStruct.red_outpost_HP");
                enemy_outpost_HP = blackboard_->get<uint16_t>("GameStatusStruct.blue_outpost_HP");
            } else {
                our_outpost_HP = blackboard_->get<uint16_t>("GameStatusStruct.blue_outpost_HP");
                enemy_outpost_HP = blackboard_->get<uint16_t>("GameStatusStruct.red_outpost_HP");
            }

            if(our_outpost_HP != dead_red_outpost_HP && enemy_outpost_HP != dead_blue_outpost_HP) {
                blackboard_->set<std::string>("outpost_status", "ALL_ALIVE");
            } else if(our_outpost_HP == dead_red_outpost_HP && enemy_outpost_HP == dead_blue_outpost_HP) {
                blackboard_->set<std::string>("outpost_status", "ALL_DEAD");
            } else if(our_outpost_HP != dead_red_outpost_HP && enemy_outpost_HP == dead_blue_outpost_HP) {
                blackboard_->set<std::string>("outpost_status", "OUR_ALIVE_AND_ENEMY_DEAD");
            } else if(our_outpost_HP == dead_red_outpost_HP && enemy_outpost_HP != dead_blue_outpost_HP) {
                blackboard_->set<std::string>("outpost_status", "OUR_DEAD_BUT_ENEMY_ALIVE");
            }
        };

        BT::NodeStatus Check_our_outpost()
        {
            if(blackboard_->get<std::string>("outpost_status") == "ALL_DEAD" || blackboard_->get<std::string>("outpost_status") == "OUR_DEAD_BUT_ENEMY_ALIVE")
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_Nav_status()
        {

        };

        BT::NodeStatus Check_if_go_enemy_door()
        {
            geometry_msgs::msg::PoseStamped enemy_door_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_door");
            // 从裁判系统中获取本机器人位置x y
            call_for_refereesystem_node->sendRequest(0x0203);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            float x = blackboard_->get<float>("RobotPosStruct.x");
            float y = blackboard_->get<float>("RobotPosStruct.y");
            if(pow(x - enemy_door_pose.pose.position.x, 2) + pow(y - enemy_door_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        };

        BT::NodeStatus Check_if_go_enemy_outpost()
        {
            geometry_msgs::msg::PoseStamped enemy_outpost_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_outpost");
            // 从裁判系统中获取本机器人位置x y
            call_for_refereesystem_node->sendRequest(0x0203);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            float x = blackboard_->get<float>("RobotPosStruct.x");
            float y = blackboard_->get<float>("RobotPosStruct.y");
            if(pow(x - enemy_outpost_pose.pose.position.x, 2) + pow(y - enemy_outpost_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_if_go_our_outpost()
        {
            geometry_msgs::msg::PoseStamped our_outpost_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("our_outpost");
            // 从裁判系统中获取本机器人位置x y
            call_for_refereesystem_node->sendRequest(0x0203);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            float x = blackboard_->get<float>("RobotPosStruct.x");
            float y = blackboard_->get<float>("RobotPosStruct.y");
            if(pow(x - our_outpost_pose.pose.position.x, 2) + pow(y - our_outpost_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_game_running()
        {
            // 根据前哨站状态设置比赛阶段
            if(blackboard_->get<std::string>("outpost_status") != "ALL_ALIVE")
            {
                blackboard_->set<std::string>("game_stage", "running");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus Check_if_go_keystone_heights()
        {
            // TODO: 可能需要考虑的：如果本来己方前哨站没掉，然后去了敌方梯高，后面双方前哨站都掉了，又会跑回己方梯高
            geometry_msgs::msg::PoseStamped keystone_heights_pose;
            if(blackboard_->get<std::string>("outpost_status") == "OUR_ALIVE_AND_ENEMY_DEAD") {
                keystone_heights_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("enemy_keystone_heights");
            }
            keystone_heights_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("our_keystone_heights");
            // 从裁判系统中获取本机器人位置x y
            call_for_refereesystem_node->sendRequest(0x0203);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            float x = blackboard_->get<float>("RobotPosStruct.x");
            float y = blackboard_->get<float>("RobotPosStruct.y");
            if(pow(x - keystone_heights_pose.pose.position.x, 2) + pow(y - keystone_heights_pose.pose.position.y, 2) > 2.25)
            {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus Check_is_set_game_status_as_last()
        {
            uint16_t last_stage_remaining_time = 60;
            // 从裁判系统中获取当前阶段剩余时间，在最后一两分钟按云台手标点走
            call_for_refereesystem_node->sendRequest(0x0001);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            uint16_t stage_remaining_time = blackboard_->get<uint16_t>("GameStatusStruct.stage_remaining_time");
            if(stage_remaining_time <= last_stage_remaining_time)
            {
                blackboard_->set<std::string>("game_stage", "last");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_blood()
        {
            uint16_t lowest_HP = blackboard_->get<uint16_t>("lowest_HP");

            // 从裁判系统得到当前血量
            call_for_refereesystem_node->sendRequest(0x0201);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            uint16_t current_HP = blackboard_->get<uint16_t>("RobotStatusStruct.current_HP");

            // for test
            // current_HP = 150;

            if (current_HP < lowest_HP)
            {
                RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.\n",current_HP,lowest_HP);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_enemy()
        {
            // TODO：从视觉获取是否发现敌人
            // if(blackboard_->get<uint8_t>("if_find_enemy"))
            // 如果未发现敌人返回FAILURE
            // 如果发现敌人，再获取敌人位置，如果敌人位置在射击范围内，返回Failure
            // 在涉及范围外返回SUCCESS（并设置敌人位置到黑板中）
            if(1)
            {
                RCLCPP_INFO(node_->get_logger(),"发现敌人，开始打弹.\n");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_is_dead()
        {
            // TODO: 拿什么判断
        };


        BT::NodeStatus Check_supply_zone()
        {
            // 从裁判系统获取己方补给站前补血点的占领状态和己方补给站内部补血点的占领状态（1为已占领）
            call_for_refereesystem_node->sendRequest(0x0101);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.5);
            };
            // TODO：只有一个event_data数据，等解析后再写
        };


    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<CallForRefereeSystem> call_for_refereesystem_node;
        BT::Blackboard::Ptr blackboard_;
    };
}
#endif