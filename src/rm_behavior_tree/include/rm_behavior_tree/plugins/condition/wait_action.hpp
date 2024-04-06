#ifndef WAIT_ACTION_HPP
#define WAIT_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
    class WaitAction : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    WaitAction(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&WaitAction::wait_action, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<int>("wait_time"),
        };
    };

    BT::NodeStatus wait_action(){
        int wait_time;
        getInput("wait_time", wait_time);
        sleep(wait_time);
        RCLCPP_INFO(node_->get_logger(),"wait...");
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree
#endif