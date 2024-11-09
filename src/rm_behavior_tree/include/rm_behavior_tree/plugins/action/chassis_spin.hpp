#ifndef CHASSIS_SPIN_HPP
#define CHASSIS_SPIN_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class ChassisSpin : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    ChassisSpin(const std::string & name, const BT::NodeConfig & conf)
        : BT::SimpleConditionNode(name, std::bind(&ChassisSpin::chassis_spin_action, this), conf){
        blackboard_ = config().blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }


    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<int>("chassis_angular_vel"),
        };
    }

    BT::NodeStatus chassis_spin_action(){
        setParam_rotation(blackboard_, getInput<int>("chassis_angular_vel").value());
        // RCLCPP_INFO(node_->get_logger(),"set chassis spin vel:%d...", getInput<int>("chassis_angular_vel").value());
        return BT::NodeStatus::SUCCESS;
    }
    };
}  // namespace rm_behavior_tree

#endif

