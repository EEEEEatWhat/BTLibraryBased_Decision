#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "global_interfaces/action/go.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

class GainBloodOrBulletActionServer : public rclcpp::Node
{
public:
  using Go = global_interfaces::action::Go;
  using GoalHandleGo = rclcpp_action::ServerGoalHandle<Go>;

  explicit GainBloodOrBulletActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("go_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Go>(
      this,
      "go_service",
      std::bind(&GainBloodOrBulletActionServer::handle_goal, this, _1, _2),
      std::bind(&GainBloodOrBulletActionServer::handle_cancel, this, _1),
      std::bind(&GainBloodOrBulletActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Go>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Go::Goal> goal)
  {
    //判断
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGo> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGo> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread

    std::thread{std::bind(&GainBloodOrBulletActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGo> goal_handle)
  {
    /*
     给nav2传目标点?
     */


    std::cout << "Executing goal" << std::endl;
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Go::Feedback>();
    auto result = std::make_shared<Go::Result>();

    rclcpp::Time deadline = get_clock()->now() + rclcpp::Duration::from_seconds(60);// 设置限制时间
    int cycle = 0;
    /*
     获取当前位置作为反馈
     */
    while( get_clock()->now() < deadline  )
    {
      if (goal_handle->is_canceling())
      {
        result->done = false;
        goal_handle->canceled(result);
        std::cout << "Goal canceled" << std::endl;
        return;
      }

      feedback->current_pos = cycle++; // 递增
      goal_handle->publish_feedback(feedback);

      std::cout << "Publish feedback" << std::endl;

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->done = true;
      goal_handle->succeed(result);
      std::cout << "Goal succeeded" << std::endl;
    }
  }
};  // class GainBloodOrBulletActionServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GainBloodOrBulletActionServer>();

  rclcpp::spin(node);

  return 0;
}