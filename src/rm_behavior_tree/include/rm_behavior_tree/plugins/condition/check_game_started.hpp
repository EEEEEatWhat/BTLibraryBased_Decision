#ifndef CHECK_GAME_STARTED_HPP
#define CHECK_GAME_STARTED_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
    class CheckGameStarted : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    CheckGameStarted(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckGameStarted::check_game_started, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus check_game_started(){
        if(blackboard_->get<std::string>("game_stage") == "not_started"){
            RCLCPP_INFO(node_->get_logger(),"Game not start!");
            return BT::NodeStatus::FAILURE;
        }
            return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif

