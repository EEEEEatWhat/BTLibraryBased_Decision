#ifndef SET_TUOLUO_STATUS_HPP_
#define SET_TUOLUO_STATUS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot_decision{
// class SetTuoluoStatus : public BT::RosTopicPubNode<std_msgs::msg::Float32>
class SetTuoluoStatus : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    SetTuoluoStatus(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
        tuoluo_status = 0.f;
    };


    bool setMessage(geometry_msgs::msg::Twist &msg) override
    {
        // TODO: 通过当前自身是否无敌、比赛时间、是否有敌人等信息来设置小陀螺状态
        sleep(1);
        msg.angular.z = 0.3;

        RCLCPP_INFO(node_->get_logger(),"设置小陀螺状态话题消息...\n");
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
    // 小陀螺状态：0为不转，1为转
    float tuoluo_status;
};

};// namespace robot_decision


#endif