#ifndef CHECK_GAME_RUNNING_HPP
#define CHECK_GAME_RUNNING_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
    class CheckGameRunning : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    CheckGameRunning(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckGameRunning::check_game_running, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus check_game_running(){
        if(blackboard_->get<std::string>("game_stage") == "running"){
            RCLCPP_INFO(node_->get_logger(),"Game is running!");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }


    };
}  // namespace rm_behavior_tree

#endif

