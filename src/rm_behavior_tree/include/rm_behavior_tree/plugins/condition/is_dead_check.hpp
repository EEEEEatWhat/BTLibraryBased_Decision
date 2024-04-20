#ifndef IS_DEAD_CHECK_HPP
#define IS_DEAD_CHECK_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class IsDeadCheck : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    IsDeadCheck(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&IsDeadCheck::check_is_dead, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    }
    
    BT::NodeStatus check_is_dead(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0201);
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint16_t current_hp = blackboard_->get<uint16_t>("RobotStatusStruct.current_HP");
        if(current_hp == 0){
            RCLCPP_INFO(node_->get_logger(),"has been dead, try to reborn...");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    } 


    };
}  // namespace rm_behavior_tree

#endif

