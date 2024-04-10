#ifndef RESET_RES_DATA_HPP
#define RESET_RES_DATA_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree{
    class ResetResData : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    ResetResData(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&ResetResData::reset_res_data, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {};
    };

    BT::NodeStatus reset_res_data(){
        blackboard_->set<uint8_t>("confirmInstaRes",0);
        blackboard_->set<uint8_t>("confirmRes",0);
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif
