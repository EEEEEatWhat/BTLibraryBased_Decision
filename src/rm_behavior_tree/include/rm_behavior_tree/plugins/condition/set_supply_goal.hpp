#ifndef SET_SUPPLY_GOAL_HPP
#define SET_SUPPLY_GOAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace rm_behavior_tree{
    class SetSupplyGoal : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    SetSupplyGoal(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&SetSupplyGoal::set_supply_goal, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }


    static BT::PortsList providedPorts(){
        return {};
    }
    
    BT::NodeStatus set_supply_goal(){
        auto supply_zone_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("supply_zone_pose");
        blackboard_->set<geometry_msgs::msg::PoseStamped>("goal_pose", supply_zone_pose);
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SetSupplyGoal>("SetSupplyGoal");
}