#ifndef PUBLIC_HPP
#define PUBLIC_HPP

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_decision_interfaces/msg/target.hpp"

namespace rm_behavior_tree{
    /**
     *  @brief RPY转四元数后返回四元数自身类的重载函数
     *  @return 四元数的类
     **/
    template <class A, class B>
    B convert(const A &a, B &b, bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    }

    /**
     *  @brief 通过查tf获得自身车在map下的坐标
     *  @return void
     **/
    void get_my_pose(BT::Blackboard::Ptr blackboard_){
        geometry_msgs::msg::TransformStamped transform_stamped;
        std::string fromFrameRel = "map";
        std::string toFrameRel = "base_link";
        try {
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            transform_stamped = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("WARN"), "Transform exception: %s", ex.what());
        return;
        }
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        blackboard_->set<float>("my_x", transform_stamped.transform.translation.x);
        blackboard_->set<float>("my_y", transform_stamped.transform.translation.y);
        blackboard_->set<float>("my_yaw", yaw);
    };

    /**
     *  @brief 通过查tf获得敌方车在map下的坐标
     *  @return void
     **/
    void get_enemy_pose(BT::Blackboard::Ptr blackboard_, const std::shared_ptr<rm_decision_interfaces::msg::Target>& last_msg){
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        auto node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        geometry_msgs::msg::PointStamped point_in_b;
        point_in_b.header.frame_id = "base_link";
        point_in_b.point.x = last_msg->aim_x;
        point_in_b.point.y = last_msg->aim_y;
        point_in_b.point.z = 0.0;
        geometry_msgs::msg::PointStamped point_in_a;
        try {
            point_in_a = tf_buffer_->transform(point_in_b, point_in_a, "map");
            blackboard_->set<double>("enemy_x", point_in_a.point.x);
            blackboard_->set<double>("enemy_y", point_in_a.point.y);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(rclcpp::get_logger("WARN"), "%s", ex.what());
        }
    };

    void setParam_rotation(BT::Blackboard::Ptr blackboard_, int speed){
        using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
        ResultFuturePtr set_param_future_rotation;
        rclcpp::SyncParametersClient::SharedPtr param_client_;
        std::vector<rclcpp::Parameter> params;
        auto node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        if(blackboard_->get<bool>("initial_set_param_rotation")){
            return;
        }
        if (!param_client_->service_is_ready()){
            RCLCPP_WARN(node_->get_logger(), "Service not ready, skipping parameter set");
            return;
        }
        params.push_back(rclcpp::Parameter("self_rotation",speed));
        auto result = param_client_->set_parameters_atomically(params);
        if(result.successful){
            RCLCPP_INFO(node_->get_logger(),"rotation speed set to %d successfully...",speed);
        } else {
            RCLCPP_INFO(node_->get_logger(),"Failed to set rotation parameter");
        }
        blackboard_->set<bool>("initial_set_param_rotation", true);
    }

}



#endif