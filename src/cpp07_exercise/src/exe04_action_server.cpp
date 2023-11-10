#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using base_interfaces_demo::action::Nav;
using namespace std::placeholders;

// 3.定义节点类；
class ExeNavActionServer: public rclcpp::Node {
public:
    ExeNavActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):        
    Node("exe_nav_action_server",options){
        pose_sub = rclcpp_action::create_server<Nav>(
            this,
            "nav",
            std::bind(&ExeNavActionServer::handle_goal,this,_1,_2),
            std::bind(&ExeNavActionServer::handle_cancel,this,_1),
            std::bind(&ExeNavActionServer::handle_accepted,this,_1)
        );
    };
private:

    //using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const  Nav::Goal> goal_handle){
        (void)uuid;
        (void)goal_handle;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };
    //using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT; 
    };
    //using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        (void)goal_handle;
    };
    rclcpp_action::Server<Nav>::SharedPtr pose_sub;


};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);
    // 4.调用spin函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<ExeNavActionServer>());
    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}