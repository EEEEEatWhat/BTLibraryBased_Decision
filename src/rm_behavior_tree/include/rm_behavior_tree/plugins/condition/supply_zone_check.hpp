#ifndef SUPPLY_ZONE_CHECK_HPP
#define SUPPLY_ZONE_CHECK_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class SupplyZoneCheck : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    SupplyZoneCheck(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&SupplyZoneCheck::check_supply_zone, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus check_supply_zone(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0101); // event_data_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint32_t bit0 = blackboard_->get<uint32_t>("PlaygroundEventStruct.event_data") & 1; // 己方补给站前补血点的占领状态，1 为已占领
        if(bit0 == 1){
            RCLCPP_INFO(node_->get_logger(),"Supply zone is occupied!");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),"Supply zone is free!");
        return BT::NodeStatus::SUCCESS;
    }

    };
}  // namespace rm_behavior_tree

#endif