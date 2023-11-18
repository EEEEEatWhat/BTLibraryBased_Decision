#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "global_interfaces/action/behavior_tree_pose.hpp"
using namespace BT;
using global_interfaces::action::BehaviorTreePose;
class BehavioreTreePose: public RosActionNode<BehaviorTreePose>
{
public:
  BehavioreTreePose(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<BehaviorTreePose>(name, conf, params)
  {}
private:
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::Point goal_position;
    geometry_msgs::msg::Quaternion geo_qnt;
    tf2::Quaternion tf_qnt;    
    template<class A, class B>
    B convert(const A & a, B & b,bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    }

public:
  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    return providedBasicPorts({
      InputPort<float>("x"),
      InputPort<float>("y"),      
      InputPort<float>("theta")      
      });
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    /* 原代码过于臃肿，不如重载的模板函数 -by suzukisuncy
    goal_pose.pose.set__orientation(geo_qnt);
    tf2::convert<tf2::Quaternion,geometry_msgs::msg::Quaternion>(tf_qnt,geo_qnt);
    */
    // ~TODO:(suzukisuncy&ynghawU,2023/11/14)补充端口和端口结构体  
    /* (ynghawU 2023/11/14 22:18) 应该完成力!
    在 src/decision/decision_behavior_tree/include/go_pub.hpp和
    src/decision/decision_behavior_tree/include/plugins/action/gain_blood_or_bullet_action.hpp
    测试 : ros2 run decision decision_node
    */
    goal_position.set__x(1.0).set__y(1.0).set__z(0);
    tf_qnt.setRPY(1.1,1.2,1.3);
    goal_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
    goal_pose.pose.set__position(goal_position);
    goal_pose.pose.set__orientation(convert<tf2::Quaternion,geometry_msgs::msg::Quaternion>(tf_qnt,geo_qnt,true));
    goal.set__pose(goal_pose);
    RCLCPP_INFO(node_->get_logger(),"Goal设置成功. . . ");
    return true;
  };
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    (void)wr;
    std::stringstream ss;
    ss << "Result received: ";
    // for (auto number : wr.result->sequence) {
    //   ss << number << " ";
    // }

    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);

    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    (void)feedback;
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};