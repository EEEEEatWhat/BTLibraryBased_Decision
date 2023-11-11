/*  
  需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作客户端；
      3-2.发送请求；
      3-3.处理目标发送后的反馈；
      3-4.处理连续反馈；
      3-5.处理最终响应。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

using namespace nav2_msgs::action;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::placeholders;

// 3.定义节点类；
class MinimalActionClient : public rclcpp::Node
{
public:

  explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("minimal_action_client", node_options)
  {
    // 3-1.创建动作客户端；
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
  }

  // 3-2.发送请求；
  void send_goal(int64_t num)
  {
    (void)num;
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "动作客户端未被初始化。");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "服务连接失败！");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_position.set__x(1.0).set__y(1.0).set__z(0);
    tf_qnt.setRPY(1.1,1.2,1.3);
    goal_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
    goal_pose.pose.set__position(goal_position);
    goal_pose.pose.set__orientation(convert<tf2::Quaternion,geometry_msgs::msg::Quaternion>(tf_qnt,geo_qnt,true));
    goal_msg.set__pose(goal_pose);

    RCLCPP_INFO(this->get_logger(), "发送请求数据！");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&MinimalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&MinimalActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
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
  // 3-3.处理目标发送后的反馈；
  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
    }
  }

  // 3-4.处理连续反馈；
  void feedback_callback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // int32_t NavigateToPose = (int32_t)(feedback->NavigateToPose * 100);
    // RCLCPP_INFO(this->get_logger(), "当前进度: %d%%", NavigateToPose);
  }

  // 3-5.处理最终响应。
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "任务被中止");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "任务被取消");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "未知异常");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "任务执行完毕，最终结果: %ld", result.result->sum);
  }
}; 

int main(int argc, char ** argv)
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);

  // 4.调用spin函数，并传入节点对象指针；
  auto action_client = std::make_shared<MinimalActionClient>();
  action_client->send_goal(10);
  rclcpp::spin(action_client);
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}