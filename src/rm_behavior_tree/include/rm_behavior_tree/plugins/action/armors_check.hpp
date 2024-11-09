#ifndef ARMORS_CHECK_HPP
#define ARMORS_CHECK_HPP

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
// #include "auto_aim_interfaces/msg/target.hpp"
#include "my_msg_interface/msg/power_heat.hpp"

namespace rm_behavior_tree{
class ArmorsCheck: public BT::RosTopicSubNode<my_msg_interface::msg::PowerHeat>
{
public:
  ArmorsCheck(const std::string& name,
                const BT::NodeConfig& conf,
                const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<my_msg_interface::msg::PowerHeat>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("topic_name"),
    };
  }

  BT::NodeStatus onTick(const std::shared_ptr<my_msg_interface::msg::PowerHeat>& last_msg) override
  {
    if(last_msg) // empty if no new message received, since the last tick
    {
        RCLCPP_INFO(logger(), "New message: %f",  last_msg->chassis_power);   
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}


#endif