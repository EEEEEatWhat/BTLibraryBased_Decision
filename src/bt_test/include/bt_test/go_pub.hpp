#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "global_interfaces/msg/goal_pose.hpp"
#include "rclcpp/qos.hpp"
using namespace std::chrono_literals;

class GoPublisher : public rclcpp::Node
{
public:
    GoPublisher(std::string name) : Node("go_publisher")
    {
      // 3-1.创建发布方；
        publisher_ = this->create_publisher<global_interfaces::msg::GoalPose>("goal_pose",rclcpp::SystemDefaultsQoS());
      // 3-2.创建定时器；
        timer_ = this->create_wall_timer(500ms, std::bind(&GoPublisher::timer_callback, this));
    }

private:
    template<class A, class B>
    B convert(const A & a, B & b,bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    }

    void timer_callback()
    {
      // 3-3.组织消息并发布。
        geometry_msgs::msg::PoseStamped goal_pose_;
        geometry_msgs::msg::Point goal_position;
        geometry_msgs::msg::Quaternion geo_qnt;
        tf2::Quaternion tf_qnt;  

        goal_pose_.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
        goal_position.set__x(1.0).set__y(1.0).set__z(0);
        goal_pose_.pose.set__position(goal_position);
        goal_pose_.pose.set__orientation(convert<tf2::Quaternion,geometry_msgs::msg::Quaternion>(tf_qnt,geo_qnt,true));
        
        RCLCPP_INFO(this->get_logger(), "导航信息:x=%lf,y=%lf,w=%lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y,goal_pose_.pose.orientation.w);
        goal_pose.set__header(goal_pose_.header).set__pose(goal_pose_.pose);
        publisher_->publish(goal_pose);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<global_interfaces::msg::GoalPose>::SharedPtr publisher_;
    global_interfaces::msg::GoalPose goal_pose;
    
};
