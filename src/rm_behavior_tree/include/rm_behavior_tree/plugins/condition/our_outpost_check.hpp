#ifndef OUR_OUTPOST_CHECK
#define OUR_OUTPOST_CHECK

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class OurOutpostCheck : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    OurOutpostCheck(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&OurOutpostCheck::check_our_outpost, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus check_our_outpost(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0003); // game_status_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint8_t sentry_id = blackboard_->get<uint8_t>("sentry_id");
        uint16_t outpost_hp;
        if(sentry_id == 7){
            outpost_hp = blackboard_->get<uint16_t>("GameRobotHPStruct.red_outpost_HP");
        } else {
            outpost_hp = blackboard_->get<uint16_t>("GameRobotHPStruct.blue_outpost_HP");
        }
        if(outpost_hp < 50){
            RCLCPP_INFO(node_->get_logger(),"Our outpost will be destroyed right now!");
            blackboard_->set<bool>("en_chassis_spin",true);
            blackboard_->set<std::string>("game_stage","running");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif

