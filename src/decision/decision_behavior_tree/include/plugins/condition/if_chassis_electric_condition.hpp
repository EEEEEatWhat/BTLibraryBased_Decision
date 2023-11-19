#ifndef DECISION_BEHAVIOR_TREE__PLUGINS__IF_CHASSIS_ELECTRIC_CONDITION_HPP_
#define DECISION_BEHAVIOR_TREE__PLUGINS__IF_CHASSIS_ELECTRIC_CONDITION_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace decision_behavior_tree
{
    class IfChassisElectricCondition : public BT::SimpleConditionNode
    {
    public:
        IfChassisElectricCondition(const std::string& name, const BT::NodeConfig& config);

        IfChassisElectricCondition() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                // BT::InputPort<std::string>(“chassis_condition”,  "default_value", "Port description")
            };
        }

    private:
        void ChassisCallback();
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<bool>::SharedPtr chassis_sub_; 
        std::string chassis_topic_;
        bool is_chassis_electric_;
    };
    
} // namespace decision_behavior_tree

#endif // DECISION_BEHAVIOR_TREE__PLUGINS__IF_CHASSIS_ELECTRIC_CONDITION_HPP_