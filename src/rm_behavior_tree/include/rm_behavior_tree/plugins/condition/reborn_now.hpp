#ifndef REBORN_NOW_HPP
#define REBORN_NOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
    class RebornNow : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    RebornNow(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&RebornNow::set_reborn, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    }

    BT::NodeStatus set_reborn(){
        auto last_res_count = blackboard_->get<int>("res_count");
        if(last_res_count >= 4){
            RCLCPP_INFO(node_->get_logger(),"res_count >= 4, can't reborn...");
            // return BT::NodeStatus::FAILURE;
        }
        if(blackboard_->get<std::string>("own_status") == "activated_reborn"){
            blackboard_->set<int>("res_count", last_res_count+1);
        }
        RCLCPP_INFO(node_->get_logger(), "res_count: %d", blackboard_->get<int>("res_count"));
        if(blackboard_->get<bool>("en_instaRes")){
            RCLCPP_INFO(node_->get_logger(),"set instant reborn...");
            blackboard_->set<uint8_t>("confirmInstaRes",1);
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(),"set normal reborn...");
        blackboard_->set<uint8_t>("confirmRes",1);
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif
