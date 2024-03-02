#ifndef SEARCH_ENEMY_HPP
#define SEARCH_ENEMY_HPP

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rmuc_decision{
class SearchEnemy : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    SearchEnemy(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
        tuoluo_status = false;
    };


    bool setMessage(geometry_msgs::msg::Twist &msg) override
    {
        sleep(1);
        msg.angular.z = blackboard_->get<double>("gimbal_angular_vel");
        tuoluo_status = true;
        RCLCPP_INFO(node_->get_logger(),"设置云台状态，角速度为%lf...", msg.angular.z);
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
    bool tuoluo_status;
};

};// namespace robot_decision


#endif