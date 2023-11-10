#include "condition/if_chassis_electric_condition.hpp"

namespace decision_behavior_tree
{
    IfChassisElectricCondition::IfChassisElectricCondition(const std::string& name, const BT::NodeConfig& config)
                               : BT::ConditionNode(name , config),
                                chassis_topic_("/chassis_status"),
                                is_chassis_electric_(false)
    {
        getInput("chassis_condition",chassis_topic_);
        auto node = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
        callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        chassis_sub_ = node->create_subscription<bool>(chassis_topic_,
                                                rclcpp::SystemDefaultsQoS(),
                                                std::bind(&IfChassisElectricCondition::ChassisCallback, this, std::placeholders::_1),
                                                sub_option);
    }

    BT::NodeStatus IfChassisElectricCondition::tick()
    {
        callback_group_executor_.spin_some();
        if(is_chassis_electric_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    void IfChassisElectricCondition::ChassisCallback()
    {   
        // is_chassis_electric_ = // 从某处读取数据 
        
    }
} // namespece decision_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<decision_behavior_tree::IfChassisElectricCondition>("if_chassis_electric");
}