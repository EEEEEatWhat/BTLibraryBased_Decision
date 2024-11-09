#ifndef CHECK_SHOOTER_STATUS_HPP
#define CHECK_SHOOTER_STATUS_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class CheckShooterStatus : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    CheckShooterStatus(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckShooterStatus::check_shooter_status, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {
            BT::OutputPort<std::string>("shooter_status"),
        };
    }
    
    BT::NodeStatus check_shooter_status(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0201); // event_data_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint8_t bit0 = blackboard_->get<uint8_t>("RobotStateStruct.power_management_shooter_output") & 1; // shooter 口输出， 0 无输出，1 有24V输出
        if(bit0){
            setOutput("shooter_status","unlock");
            return BT::NodeStatus::SUCCESS;
        }
        setOutput("shooter_status","lock");
        RCLCPP_INFO(node_->get_logger(),"shooter lock!");
        return BT::NodeStatus::FAILURE;
    } 


    };
}  // namespace rm_behavior_tree

#endif

