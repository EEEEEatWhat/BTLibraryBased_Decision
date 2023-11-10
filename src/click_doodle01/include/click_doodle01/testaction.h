#include "behaviortree_ros2/bt_action_node.hpp"
#include "base_interfaces_demo/action/nav.hpp"
using namespace BT;

class NavAction: public RosActionNode<base_interfaces_demo::action::Nav>
{
public:
  NavAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<base_interfaces_demo::action::Nav>(name, conf, params)
  {}

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
    // get "order" from the Input port
    // getInput("order", goal.order);   
    std::cout << getInput<float>("x").value() << std::endl
      << getInput<float>("y").value() << std::endl
      << getInput<float>("theta").value() << std::endl; 
    goal.set__goal_theta(getInput<float>("theta").value()).set__goal_x(getInput<float>("x").value()).set__goal_y(getInput<float>("y").value());
    RCLCPP_INFO(node_->get_logger(),"Goal设置成功. . . ");
    // return true, if we were able to set the goal correctly.
    return true;
  };
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
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
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    std::cout << feedback->distance;
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }
};