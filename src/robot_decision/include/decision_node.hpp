#ifndef ROBOT_DECISION__INCLUDE__DECISION_NODE_HPP
#define ROBOT_DECISION__INCLUDE__DECISION_NODE_HPP

#include <memory>

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


#include "go_pub.hpp"
#include "action/patrol_to_supply_action.hpp"
#include "condition_node.hpp"
namespace robot_decision
{
    class DecisionNode : public rclcpp::Node
    {
    public:
        DecisionNode(const rclcpp::NodeOptions &options);
        ~DecisionNode();

        /**
         *  @brief RPY转四元数后返回四元数自身类的重载函数
         *  @return 四元数的类
         **/
        template <class A, class B>
        B convert(const A &a, B &b, bool c);

        /**
         *  @brief 载入配置信息
         *  @param yaml_file
         *         yaml文件路径
         *  @param blackboard_
         *         黑板指针
         *  @return 是否成功
         **/
        bool decodeConfig(std::string yaml_file, BT::Blackboard::Ptr blackboard_);

        /**
         *  @brief 初始化
         **/
        void init();

    public:
        BT::Tree tree_;
        BT::Blackboard::Ptr blackboard_;

    private:
        BT::RosNodeParams patrolParams; 
        rclcpp::NodeOptions options_;
        BT::BehaviorTreeFactory factory_;
        const std::string xml_file_path = "src/decision/robot_decision/tree/mainTree.xml";
        const std::string yaml_file_path = "src/decision/params/init_pose.yaml";
    };
}

#endif