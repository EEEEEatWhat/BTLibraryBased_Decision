#ifndef HP_CHECK_HPP
#define HP_CHECK_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class HPCheck : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    HPCheck(const std::string & name, const BT::NodeConfig & conf)
        : BT::SimpleConditionNode(name, std::bind(&HPCheck::check_hp, this), conf){
        blackboard_ = config().blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }


    BT::NodeStatus check_hp(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0201);
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        uint16_t current_hp = blackboard_->get<uint16_t>("RobotStateStruct.current_HP");
        uint16_t hp_threshold;
        getInput("hp_threshold", hp_threshold);
        if(blackboard_->get<int>("res_count") == 0){
            if(current_hp < 100){
                setOutput("need_supply", true);
                RCLCPP_INFO(node_->get_logger(),"当前血量：%u,未复活，低于100.",current_hp);
                return BT::NodeStatus::FAILURE;
            }
        }
        if(current_hp < hp_threshold){
            setOutput("need_supply", true);
            RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.",current_hp,hp_threshold);
            return BT::NodeStatus::FAILURE;
        }
        setOutput("need_supply", false);
        RCLCPP_INFO(node_->get_logger(),"current HP:%u",current_hp);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint16_t>("hp_threshold"),
            BT::OutputPort<bool>("need_supply"),
        };
    };

    };
}  // namespace rm_behavior_tree

#endif

