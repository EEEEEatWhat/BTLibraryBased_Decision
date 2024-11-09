#ifndef IS_ARRIVED_HPP
#define IS_ARRIVED_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class IsArrived : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    public:
    IsArrived(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&IsArrived::check_is_arrived, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
                
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("goal_name"),
        };
    }
    
    BT::NodeStatus check_is_arrived(){
        std::string goal_name;
        getInput("goal_name", goal_name);
        auto goal_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name);
        this->get_my_pose(blackboard_,node_);
        auto my_x = blackboard_->get<double>("my_x");
        auto my_y = blackboard_->get<double>("my_y");
        if(pow(my_x - goal_pose.pose.position.x, 2) + pow(my_y - goal_pose.pose.position.y, 2) > 1){
            RCLCPP_INFO(node_->get_logger(),"not arrived %s",goal_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }

    void get_my_pose(BT::Blackboard::Ptr blackboard_, rclcpp::Node::SharedPtr node_){
        geometry_msgs::msg::TransformStamped transform_stamped;
        std::string fromFrameRel = "map";
        std::string toFrameRel = "base_link";
        if(tf_buffer_->canTransform(fromFrameRel,toFrameRel,tf2::TimePointZero, tf2::durationFromSec(0.1))) {
            try {
                transform_stamped = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero, tf2::durationFromSec(0.5));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(rclcpp::get_logger("WARN"), "Transform exception: %s", ex.what());
                // return;
            }
        }
        
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        );

        float yaw = std::atan2( 
            2*(q.w() * q.z() 
            + q.x() * q.y()),
            1-2*(q.y() * q.y() 
            + q.z() * q.z())
        );
        blackboard_->set<float>("my_x", transform_stamped.transform.translation.x);
        blackboard_->set<float>("my_y", transform_stamped.transform.translation.y);
        blackboard_->set<float>("my_yaw", yaw);
    };


    };
}  // namespace rm_behavior_tree

#endif