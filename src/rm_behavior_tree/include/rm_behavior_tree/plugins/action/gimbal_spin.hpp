#ifndef GIMBAL_SPIN_HPP
#define GIMBAL_SPIN_HPP

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
class GimbalSpin : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
{
public:
    GimbalSpin(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
    };

    static BT::PortsList providedPorts(){
        return {
            // BT::InputPort<std::string>("topic_name"),
            // BT::InputPort<bool>("en_gimbal_spin"),
        };
    };


    bool setMessage(geometry_msgs::msg::Twist &msg) override
    {
        // auto en_gimbal_spin = getInput<bool>("en_gimbal_spin");
            msg.angular.z = 0;

        // if(!blackboard_->get<bool>("nav_mode") ){
        //     msg.angular.z = blackboard_->get<double>("gimbal_angular_vel");
        // } else {
        //     msg.angular.z = 0;
        // }
        // if(en_gimbal_spin.value()){
        //     msg.angular.z = blackboard_->get<double>("gimbal_angular_vel");
        // } else {
        //     msg.angular.z = 0;
        // }
            RCLCPP_INFO(node_->get_logger(),"gimbal w:%lf...", msg.angular.z);
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
};

};// namespace rm_behavior_tree

#endif