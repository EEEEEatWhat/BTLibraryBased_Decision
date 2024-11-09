#ifndef PUB_GOAL_HPP
#define PUB_GOAL_HPP

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
class PubGoal : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
    PubGoal(const std::string & instance_name,
                        const BT::NodeConfig& conf,
                        const BT::RosNodeParams& params)
                    : BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>(instance_name, conf, params)
    {
        blackboard_ = config().blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        is_patrol_start = false;
    };

    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("goal_name"),
            BT::InputPort<int>("patrol_num"),
        };
    };


    bool setMessage(geometry_msgs::msg::PoseStamped &msg) override
    {
        blackboard_->set<bool>("nav_mode", true);

        auto goal_name = getInput<std::string>("goal_name");
        auto patrol_num = getInput<int>("patrol_num");
        if(patrol_num.value()){
            if(!is_patrol_start){
                is_patrol_start = true;
                int patrol_num = 0;
                std::queue<geometry_msgs::msg::PoseStamped> temp_points = blackboard_->get<std::queue<geometry_msgs::msg::PoseStamped>>(goal_name.value());
                for(int i=0; i<patrol_num; i++){
                    patrol_points.push(temp_points.front());
                    temp_points.pop();
                }
            }
            if(patrol_points.empty()){
                RCLCPP_ERROR(node_->get_logger(),"patrol_points is empty!");
                return false;
            }
            auto temp_pose = patrol_points.front();
            patrol_points.pop();
            patrol_points.emplace(temp_pose);
            msg = temp_pose;
            RCLCPP_INFO(node_->get_logger(),"set goal for %s successfully...",goal_name.value().c_str());
            return true;
        }
        msg = blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name.value());
        return true;
    };

private:
    BT::Blackboard::Ptr blackboard_;
    bool is_patrol_start;
    std::queue<geometry_msgs::msg::PoseStamped> patrol_points;
};

};// namespace rm_behavior_tree

#endif