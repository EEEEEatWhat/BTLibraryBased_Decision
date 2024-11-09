#ifndef CHECK_AERIAL_CMD_HPP
#define CHECK_AERIAL_CMD_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "my_msg_interface/msg/aerial_cmd.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class CheckAerialCmd : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<my_msg_interface::msg::AerialCmd>::SharedPtr aerial_cmd_sub;
    public:
    CheckAerialCmd(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckAerialCmd::chekc_aerial_cmd, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        aerial_cmd_sub = node_->create_subscription<my_msg_interface::msg::AerialCmd>
                ("/AerialCmd", 10,std::bind(&CheckAerialCmd::aerial_callback, this, std::placeholders::_1));
        blackboard_->set<bool>("updated", false);
        RCLCPP_INFO(rclcpp::get_logger("TEST"),"start!");
    }

    void aerial_callback(const my_msg_interface::msg::AerialCmd::SharedPtr msg){
        RCLCPP_INFO(rclcpp::get_logger("TEST"),"1111");
        blackboard_->set<bool>("updated", msg->updated);
        blackboard_->set<geometry_msgs::msg::PoseStamped>("aerial_target_pose", geometry_msgs::msg::PoseStamped());
        RCLCPP_INFO(rclcpp::get_logger("TEST"),"updated: %d",msg->updated);
    }

    static BT::PortsList providedPorts(){
        return {};
    }

    BT::NodeStatus chekc_aerial_cmd(){
        if(!blackboard_->get<bool>("updated")){
            return BT::NodeStatus::FAILURE;
        } else {
            geometry_msgs::msg::PoseStamped aerial_target_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>("aerial_target_pose");
            blackboard_->set<geometry_msgs::msg::PoseStamped>("aerial_target_pose", aerial_target_pose);
            return BT::NodeStatus::SUCCESS;
        }
    }
    };
}  // namespace rm_behavior_tree

#endif

