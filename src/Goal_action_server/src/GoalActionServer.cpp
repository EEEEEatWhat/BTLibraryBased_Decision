/*
    作为behaviortree_server
*/
#include <string>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::placeholders;
using geometry_msgs::msg::PoseStamped;
using global_interfaces::action::BehaviorTreePose;
using nav2_msgs::action::NavigateToPose;
using GoalHandlePose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalActionServer: public rclcpp::Node {
public:
    GoalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):        
    Node("GoalActionServer",options){

        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
        RCLCPP_INFO(this->get_logger(),"\n创建Navigate_to_pose客户端成功 !\n");

        pose_sub = rclcpp_action::create_server<BehaviorTreePose>(
            this,
            "BehaviorTreePose",
            std::bind(&GoalActionServer::handle_goal,this,_1,_2),
            std::bind(&GoalActionServer::handle_cancel,this,_1),
            std::bind(&GoalActionServer::handle_accepted,this,_1)
        );
        RCLCPP_INFO(this->get_logger(),"\n创建BehaviorTreeAction服务端成功 !\n");
    };
private:
    //using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const  BehaviorTreePose::Goal> goal_handle){

        std::cout<<"接受到goal开始处理. . . "<<"\n";
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    //using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreePose>> goal_handle){

        std::cout<<"goal被取消!"<<"\n";
        return rclcpp_action::CancelResponse::ACCEPT; 
    };

    //using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreePose>> goal_handle){

        std::thread{std::bind(&GoalActionServer::execute, this, _1), goal_handle}.detach();
    };

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreePose>> goal_handle){

        std::cout<<"goal反馈数据. . . "<<"\n";
        auto result_msg = std::make_shared<global_interfaces::action::BehaviorTreePose::Result>();
        result_msg->result_pose.header.stamp = rclcpp::Clock().now();
        result_msg->result_pose.header.frame_id = "map";
        result_msg->result_pose.pose.orientation.set__w(1.0)
                                                .set__x(0.0)
                                                .set__y(0.0)
                                                .set__z(0.0); 
        result_msg->result_pose.pose.position.set__x(1.0)
                                            .set__y(0.0)
                                            .set__z(0.0);
        rclcpp::sleep_for(std::chrono::seconds(5));
        try
        {
            goal_handle->succeed(result_msg);        
            std::cout<<"任务完成. . .  "<<"\n";
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void send_goal() {
        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "动作客户端未被初始化。");
        }
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "服务连接失败！");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = rclcpp::Clock().now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position
                                    .set__x(1.0)
                                    .set__y(1.0)
                                    .set__z(0.0);
        goal_msg.pose.pose.orientation
                                    .set__w(1.0)
                                    .set__x(0.0)
                                    .set__y(0.0)
                                    .set__z(0.0);
    
        RCLCPP_INFO(this->get_logger(), "发送请求数据！");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =std::bind(&GoalActionServer::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =std::bind(&GoalActionServer::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =std::bind(&GoalActionServer::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(GoalHandlePose::SharedPtr goal_handle){ 
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
        }
    }

    void feedback_callback(GoalHandlePose::SharedPtr goal_handle,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
        std::cout<<"连续反馈"<<"\n";
    }

    void result_callback(const GoalHandlePose::WrappedResult & result){
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

private:

    rclcpp_action::Server<BehaviorTreePose>::SharedPtr pose_sub;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

};

// 已弃用，原因：融合在server中
class Nac2GoalClient: public rclcpp::Node {
public:
    explicit Nac2GoalClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("Nac2GoalClient", node_options)
    {
        // 3-1.创建动作客户端；
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
        std::cout<<"创建动作客户端"<<"\n";
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

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = rclcpp::Clock().now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position
                                .set__x(1.0)
                                .set__y(1.0)
                                .set__z(0.0);
    goal_msg.pose.pose.orientation
                                .set__w(1.0)
                                .set__x(0.0)
                                .set__y(0.0)
                                .set__z(0.0);
    
    RCLCPP_INFO(this->get_logger(), "发送请求数据！");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&Nac2GoalClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =std::bind(&Nac2GoalClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&Nac2GoalClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

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
    void feedback_callback(GoalHandlePose::SharedPtr goal_handle,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
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
int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto action_server = std::make_shared<GoalActionServer>();

    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}