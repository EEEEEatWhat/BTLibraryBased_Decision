#ifndef SET_OWN_STATUS_HPP
#define SET_OWN_STATUS_HPP

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
class SetOwnStatus : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    SetOwnStatus(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
    };


    bool setMessage(geometry_msgs::msg::Twist &msg) override
    {
        if(blackboard_->get<bool>("en_gimbal_spin")){
            msg.angular.z = blackboard_->get<double>("gimbal_angular_vel");
            RCLCPP_INFO(node_->get_logger(),"设置云台角速度为%lf...", msg.angular.z);
        }
        else{
            msg.angular.z = 0;
        }
        if(blackboard_->get<bool>("en_chassis_spin")){
            setParam_rotation(blackboard_, blackboard_->get<int>("chassis_angular_vel"));
        }
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
};

};// namespace rm_behavior_tree

#endif