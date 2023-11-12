#pragma once

#ifndef DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP
#define DECISION_BEHAVIOR_TREE__DECISION_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "yaml-cpp/yaml.h"
#include "yamlsetup.h"
namespace decision_behavior_tree
{
    class DecisionNode : public rclcpp::Node
    {
    private:
        std::shared_ptr<BT::Blackboard> blackboard_ ;
        BT::Blackboard blackboard;
        BT::BehaviorTreeFactory factory_;
        BT::Tree tree_;

    private:
        /**
         * @brief 载入配置信息
         * @return 是否成功
         **/
        bool decodeConfig();

    public:
        DecisionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~DecisionNode();

        /**
         * @brief 初始化
         **/
        void init();

        template <class A, class B>
        B convert(const A &a, B &b, bool c);
    };
}

#endif