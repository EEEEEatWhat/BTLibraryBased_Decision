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

#include "condition_node.hpp"
#include "action/patrol_to_supply_action.hpp"
#include "action/gain_blood_action.hpp"
#include "set_tuoluo_status.hpp"

#include "action/check_mode.hpp"

#include "action/wandering.hpp"
#include "action/go_enemy_bunker.hpp"
#include "action/go_enemy_startup.hpp"
#include "action/go_our_bunker.hpp"

#include "action/patrol_1.hpp"
#include "action/patrol_2.hpp"
#include "action/patrol_3.hpp"
#include "action/patrol_4.hpp"
#include "action/patrol_5.hpp"

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
         *  @param blackboard_
         *         黑板指针
         *  @return 是否成功
         **/
        bool Decode_config_pose(BT::Blackboard::Ptr blackboard_);

        /**
         *  @brief 初始化行为树
         **/
        void Init();


    public:
        BT::Tree tree_;
        BT::Blackboard::Ptr blackboard_;

    private:
        BT::RosNodeParams actionParams; 
        BT::RosNodeParams actionParamsForPoses; 
        BT::RosNodeParams topicParams; 
        BT::RosNodeParams topicParams2; 
        rclcpp::NodeOptions options_;
        BT::BehaviorTreeFactory factory_;
        const std::string xml_file_path = "../tree/rmulTree.xml";
    };
}

#endif