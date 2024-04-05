/*
    作为behaviortree_server
*/
#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rm_decision_interfaces/action/behavior_tree_send_poses.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

using namespace std::placeholders;
using geometry_msgs::msg::PoseStamped;

using rm_decision_interfaces::action::BehaviorTreeSendPoses;
using nav2_msgs::action::NavigateThroughPoses;
class GoalActionServerWithPoses: public rclcpp::Node {
public:
    GoalActionServerWithPoses(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()):        
    Node("GoalActionServerWithPoses",options){

        this->pose_sub = rclcpp_action::create_server<BehaviorTreeSendPoses>(
            this,
            "BehaviorTreeSendPoses",
            std::bind(&GoalActionServerWithPoses::handle_goal,this,_1,_2),
            std::bind(&GoalActionServerWithPoses::handle_cancel,this,_1),
            std::bind(&GoalActionServerWithPoses::handle_accepted,this,_1)
        );
        RCLCPP_INFO(this->get_logger(),"创建BehaviorTreeAction多点服务端成功 !");
        
        this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");
        send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =std::bind(&GoalActionServerWithPoses::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =std::bind(&GoalActionServerWithPoses::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =std::bind(&GoalActionServerWithPoses::result_callback, this, _1);
        RCLCPP_INFO(this->get_logger(),"创建navigate_through_poses客户端成功 !");
    };
private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const  BehaviorTreeSendPoses::Goal> goal_handle){
        (void)goal_handle;
        std::cout<<"BehaviorTreeSendPoses 接受到goal开始处理. . . "<<"\n";
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreeSendPoses>> goal_handle){
        (void)goal_handle;
        std::cout<<"BehaviorTreeSendPoses goal被取消!"<<"\n";
        return rclcpp_action::CancelResponse::ACCEPT; 
    };

    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreeSendPoses>> goal_handle){
        (void)goal_handle;
        std::thread{std::bind(&GoalActionServerWithPoses::execute, this, _1), goal_handle}.detach();
        
    };

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreeSendPoses>> goal_handle){
        my_goal_handle = goal_handle;
        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 动作客户端未被初始化。");
            return;
        }
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 服务连接失败！");
            return;
        }
        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.set__poses(goal_handle->get_goal()->poses);
        // goal_msg.set__behavior_tree();
        RCLCPP_INFO(this->get_logger(), "navigate_through_poses 组织Nav2数据！");
        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =std::bind(&GoalActionServerWithPoses::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =std::bind(&GoalActionServerWithPoses::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =std::bind(&GoalActionServerWithPoses::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "navigate_through_poses 发送Nav2数据！");
        auto goal_future_handle_get = goal_handle_future.get();
        if (!goal_future_handle_get) {
            RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 请求失败");
            return;
        }
    }

    void goal_response_callback(rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr goal_handle){ 
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 目标请求被服务器拒绝！");
        } 
        else {
            RCLCPP_INFO(this->get_logger(), "navigate_through_poses 目标被接收，等待结果中");

        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr goal_handle,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback){
        std::cout<<"navigate_through_poses 连续反馈"<<"\n";
        auto Pfeedback_rec = std::make_shared<rm_decision_interfaces::action::BehaviorTreeSendPoses::Feedback>();
        Pfeedback_rec->current_pose = feedback->current_pose;
        Pfeedback_rec->distance_remaining = feedback->distance_remaining;
        Pfeedback_rec->estimated_time_remaining = feedback->estimated_time_remaining;
        Pfeedback_rec->navigation_time = feedback->navigation_time;
        Pfeedback_rec->number_of_recoveries = feedback->number_of_recoveries;
        Pfeedback_rec->number_of_poses_remaining = feedback->number_of_poses_remaining;
        my_goal_handle->publish_feedback(Pfeedback_rec);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult & result){
        auto Presult_rec = std::make_shared<rm_decision_interfaces::action::BehaviorTreeSendPoses::Result>();
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "navigate_through_poses 任务执行完毕");
                Presult_rec->result_pose = feedback_rec.current_pose ;
                try{
                    my_goal_handle->succeed(Presult_rec);                }
                catch(const std::exception& e) {
                    std::cerr << e.what() << '\n';}
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 任务被中止");
                Presult_rec->result_pose = feedback_rec.current_pose ;
                try{
                    my_goal_handle->abort(Presult_rec);                }
                catch(const std::exception& e) {
                    std::cerr << e.what() << '\n';}
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 任务被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "navigate_through_poses 未知异常");
                break;
        }
    }

private:
    rclcpp_action::Server<BehaviorTreeSendPoses>::SharedPtr pose_sub;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions send_goal_options;
    rm_decision_interfaces::action::BehaviorTreeSendPoses::Feedback feedback_rec;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<BehaviorTreeSendPoses>> my_goal_handle;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    auto action_server = std::make_shared<GoalActionServerWithPoses>();

    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}
/*
// 已弃用，原因：融合在server中
class Nac2GoalClient: public rclcpp::Node {
public:
    explicit Nac2GoalClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("Nac2GoalClient", node_options)
    {
        // 3-1.创建动作客户端；
        this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(this,"navigate_through_poses");
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

    auto goal_msg = NavigateThroughPoses::Goal();
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

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&Nac2GoalClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =std::bind(&Nac2GoalClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =std::bind(&Nac2GoalClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;

  // 3-3.处理目标发送后的反馈；
    void goal_response_callback(rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr goal_handle)
    { 
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "目标请求被服务器拒绝！");
    } else {
        RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中");
    }
}

  // 3-4.处理连续反馈；
    void feedback_callback(rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::SharedPtr goal_handle,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        std::cout<<"连续反馈"<<"\n";
    }

  // 3-5.处理最终响应。
    void result_callback(const rclcpp_action::ClientGoalHandle<NavigateThroughPoses>::WrappedResult & result)
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
*/