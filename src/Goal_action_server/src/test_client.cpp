/*
  测试能否向behaviortree_server发送请求
*/
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"
using global_interfaces::action::BehaviorTreePose;
using GoalHandlePose = rclcpp_action::ClientGoalHandle<BehaviorTreePose>;
using namespace std::placeholders;

// 3.定义节点类；
class MinimalActionClient : public rclcpp::Node
{
public:

  explicit MinimalActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("BehaviorTreePoseClient", node_options)
  {
    // 3-1.创建动作客户端；
    this->client_ptr_ = rclcpp_action::create_client<BehaviorTreePose>(this,"BehaviorTreePose");
  }

  // 3-2.发送请求；
  void send_goal()
  {

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "动作客户端未被初始化。");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "服务连接失败！");
      return;
    }

    auto goal_msg = BehaviorTreePose::Goal();
    goal_msg.pose.pose.position.x = 1.0;
    RCLCPP_INFO(this->get_logger(), "发送请求数据！");

    auto send_goal_options = rclcpp_action::Client<BehaviorTreePose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&MinimalActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&MinimalActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<BehaviorTreePose>::SharedPtr client_ptr_;

  // 3-3.处理目标发送后的反馈；
  void goal_response_callback(GoalHandlePose::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
    }
  }

  // 3-4.处理连续反馈；
  void feedback_callback(GoalHandlePose::SharedPtr,const std::shared_ptr<const BehaviorTreePose::Feedback> feedback)
  {
    std::cout<<"连续反馈"<<"\n";
  }

  // 3-5.处理最终响应。
  void result_callback(const GoalHandlePose::WrappedResult & result)
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

    RCLCPP_INFO(this->get_logger(), "任务执行完毕");
  }
}; 

int main(int argc, char ** argv)
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);

  // 4.调用spin函数，并传入节点对象指针；
  auto action_client = std::make_shared<MinimalActionClient>();
  action_client->send_goal();
  rclcpp::spin(action_client);
  // 5.释放资源。
  rclcpp::shutdown();
  return 0;
}