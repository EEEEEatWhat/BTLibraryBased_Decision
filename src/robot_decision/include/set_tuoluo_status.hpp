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
        tuoluo_status = false;
    };


    bool setMessage(geometry_msgs::msg::Twist &msg) override
    {
        // TODO: 通过当前自身是否无敌、比赛时间、是否有敌人等信息来设置小陀螺状态
        sleep(1);
        msg.angular.z = blackboard_->get<double>("tuoluo_angular_vel");
        tuoluo_status = true;
        RCLCPP_INFO(node_->get_logger(),"设置小陀螺状态，角速度为%lf...", msg.angular.z);
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
    bool tuoluo_status;
};

};// namespace robot_decision


#endif