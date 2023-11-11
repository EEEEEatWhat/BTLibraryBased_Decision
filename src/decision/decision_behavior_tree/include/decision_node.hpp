#pragma once

#ifndef DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP
#define DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP

// #include "global_interfaces/msg/"

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "yaml-cpp/yaml.h"
namespace decision_behavior_tree
{
    class DecisionNode : public rclcpp::Node
    {
    private:
        BT::Blackboard::Ptr blackboard_;

    public:
        DecisionNode(/* args */);
        void init();
        ~DecisionNode();
    };
    
    DecisionNode::DecisionNode(/* args */)
    {
    }
    
    DecisionNode::~DecisionNode()
    {
    }
    
}

#endif