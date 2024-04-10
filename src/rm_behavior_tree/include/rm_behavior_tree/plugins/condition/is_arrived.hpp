#ifndef IS_ARRIVED_HPP
#define IS_ARRIVED_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class IsArrived : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    IsArrived(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&IsArrived::check_is_arrived, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
    }

    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("goal_name"),
        };
    }
    
    BT::NodeStatus check_is_arrived(){
        std::string goal_name;
        getInput("goal_name", goal_name);
        auto goal_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name);
        get_my_pose(blackboard_);
        auto my_x = blackboard_->get<double>("my_x");
        auto my_y = blackboard_->get<double>("my_y");
        if(pow(my_x - goal_pose.pose.position.x, 2) + pow(my_y - goal_pose.pose.position.y, 2) > 2.25){
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif