#ifndef ChECK_MODE_HPP
#define ChECK_MODE_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_msgs/msg/float64.hpp"

namespace robot_decision{
class CheckMode : public BT::RosTopicSubNode<std_msgs::msg::Float64>
{
private:
    BT::Blackboard::Ptr blackboard_;
    
public:
    CheckMode(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicSubNode<std_msgs::msg::Float64>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
    };

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float64>& last_msg) override
    {
        
        int mode_num = static_cast<int>(last_msg->data);
        std::string mode = std::to_string(mode_num);
        blackboard_->set<std::string>("mode", mode);
        RCLCPP_INFO(node_->get_logger(),"设置模式为%s", mode.c_str());
        return BT::NodeStatus::SUCCESS;
    };

    ~CheckMode(){};
};

}
#endif