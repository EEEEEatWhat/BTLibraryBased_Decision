#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

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
#include "action/happy_patrol_action.hpp"
#include "action/gain_blood_action.hpp"
#include "condition_node.hpp"
#include "set_tuoluo_status.hpp"
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

    protected:
        /**
         *  @brief 载入yaml文件（各巡逻点位姿、状态消息），向黑板中设置所用到的参数的默认值
         *  @param init_pose_path
         *         目标点yaml文件路径
         *  @param blackboard_
         *         黑板指针
         *  @return 是否成功
         **/
        bool Decode_config_pose(std::string init_pose_path, BT::Blackboard::Ptr blackboard_);

        /**
         *  @brief 初始化行为树
         **/
        void Init();

        /**
         *  @brief 注册解包函数
         *  @param 
         *         
         *  @return 
         **/
        void RigisteredMapSolver();

    public:
        BT::Tree tree_;
        BT::Blackboard::Ptr blackboard_;

    private:
        BT::RosNodeParams actionParams; 
        BT::RosNodeParams topicParams; 
        rclcpp::NodeOptions options_;
        BT::BehaviorTreeFactory factory_;
        const std::string condition_path = "robot_decision/config/condition.yaml";
        // const std::string xml_file_path = "robot_decision/tree/mainTree.xml";
        const std::string xml_file_path = "/home/hannah/BTLibraryBased_Decision/src/robot_decision/tree/mainTree.xml";
        // const std::string init_pose_path = "robot_decision/config/init_pose.yaml";
        const std::string init_pose_path = "/home/hannah/BTLibraryBased_Decision/src/robot_decision/config/init_pose.yaml";
    };
}

#endif