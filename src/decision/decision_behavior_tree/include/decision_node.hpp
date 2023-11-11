#pragma once

#ifndef DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP
#define DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP

// #include "global_interfaces/msg/"

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

namespace decision_behavior_tree
{
    class DecisionNode : public rclcpp::Node
    {
    private:
        /* data */
    public:
        DecisionNode(/* args */);
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