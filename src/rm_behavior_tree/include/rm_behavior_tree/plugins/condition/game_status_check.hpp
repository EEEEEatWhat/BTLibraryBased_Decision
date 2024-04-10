#ifndef GAME_STATUS_CHECK_HPP
#define GAME_STATUS_CHECK_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class GameStatusCheck : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    GameStatusCheck(const std::string & name, const BT::NodeConfig & conf)
        : BT::SimpleConditionNode(name, std::bind(&GameStatusCheck::check_game_status, this), conf){
        // RCLCPP_INFO(rclcpp::get_logger("TEST"),"GameStatusCheck created!");
        blackboard_ = config().blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus check_game_status(){
        // if(blackboard_->get<std::string>("game_stage") == "started") {
        //     RCLCPP_INFO(node_->get_logger(),"game has started!");
        //     return BT::NodeStatus::SUCCESS;
        // }]
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0001); // game_status_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        RCLCPP_INFO(node_->get_logger(),"received game_progress: %u, stage_remain_time: %d",
                                        blackboard_->get<uint8_t>("GameStatusStruct.game_progress"), blackboard_->get<uint16_t>("GameStatusStruct.stage_remain_time"));
        call_for_refereesystem_node->processResponse(0x0003); // game_robot_HP_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint8_t game_started = 4;
        if (blackboard_->get<uint8_t>("GameStatusStruct.game_progress") == game_started ||
            (blackboard_->get<uint16_t>("GameStatusStruct.stage_remain_time") < 420 && blackboard_->get<uint16_t>("GameStatusStruct.stage_remain_time") > 400) ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_1_robot_HP") != 150 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_2_robot_HP") != 250 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_3_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_4_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_5_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_7_robot_HP") != 400 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_outpost_HP") != 1500 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.red_base_HP") != 5000 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_1_robot_HP") != 150 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_2_robot_HP") != 250 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_3_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_4_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_5_robot_HP") != 200 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_7_robot_HP") != 400 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_outpost_HP") != 1500 ||
            blackboard_->get<uint16_t>("GameRobotHPStruct.blue_base_HP") != 5000 
            )
        {
            RCLCPP_INFO(node_->get_logger(),"game start!");
            blackboard_->set<std::string>("game_stage", "started");
            
            call_for_refereesystem_node->processResponse(0x0201); // robot_status_t
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.1);
            };
            uint8_t red_sentry_id = 7;
            if(blackboard_->get<uint8_t>("RobotStateStruct.robot_id") == red_sentry_id) {
                blackboard_->set<uint8_t>("sentry_id", 7);
            }
            else {
                blackboard_->set<uint8_t>("sentry_id", 107);
            }
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(),"game hasn't started yet!");
        return BT::NodeStatus::FAILURE;
    }
    };
}

#endif

