/*
   需求：处理请求发送的目标点，控制乌龟向该目标点运动，并连续反馈乌龟与目标点之间的剩余距离。
   步骤：
       1.包含头文件；
       2.初始化 ROS2 客户端；
       3.定义节点类；
            3-1.创建原生乌龟位姿订阅方，回调函数中获取乌龟位姿；
            3-2.创建原生乌龟速度发布方；
            3-3.创建动作服务端；
            3-4.解析动作客户端发送的请求；
            3-5.处理动作客户端发送的取消请求；
            3-6.创建新线程处理请求；
            3-7.新线程产生连续反馈并响应最终结果。
       4.调用spin函数，并传入节点对象指针；
       5.释放资源。
*/
// 1.包含头文件； 
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
        // 3-1.创建原生乌龟位姿订阅方，回调函数中获取乌龟位姿；
        pose_sub = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&ExeNavActionServer::poseCallBack, this, std::placeholders::_1));
        // 3-2.创建原生乌龟速度发布方；
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
        // 3-3.创建动作服务端；
        nav_action_server = rclcpp_action::create_server<Nav>(
            this,
            "nav",
            std::bind(&ExeNavActionServer::handle_goal,this,_1,_2),
            std::bind(&ExeNavActionServer::handle_cancel,this,_1),
            std::bind(&ExeNavActionServer::handle_accepted,this,_1)
            );

    }
private:
    turtlesim::msg::Pose::SharedPtr turtle1_pose = nullptr;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp_action::Server<Nav>::SharedPtr nav_action_server;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    void poseCallBack(const turtlesim::msg::Pose::SharedPtr pose){
        turtle1_pose = pose;
    }
    // 3-4.解析动作客户端发送的请求；
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & goal_uuid, std::shared_ptr<const Nav::Goal> goal){
        (void)goal_uuid;
        RCLCPP_INFO(this->get_logger(),"请求坐标:(%.2f,%.2f),航向:%.2f", goal->goal_x,goal->goal_y,goal->goal_theta);
        if (goal->goal_x < 0 || goal->goal_x > 11.1 || goal->goal_y < 0 || goal->goal_y > 11.1)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 3-5.处理动作客户端发送的取消请求；
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"任务取消!");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // 3-7.新线程产生连续反馈并响应最终结果。
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        RCLCPP_INFO(this->get_logger(),"开始执行任务......");
        // 解析目标值
        float goal_x = goal_handle->get_goal()->goal_x;
        float goal_y = goal_handle->get_goal()->goal_y;
        // 创建连续反馈对象指针；
        auto feedback = std::make_shared<Nav::Feedback>();
        // 创建最终结果对象指针；
        auto result = std::make_shared<Nav::Result>();
        rclcpp::Rate rate(1.0);
        while (true)
        {
            // 任务执行中，关于客户端发送取消请求的处理；
            if(goal_handle->is_canceling()){
                goal_handle->canceled(result);
                return;
            }
            // 解析原生乌龟位姿数据；
            float turtle1_x = turtle1_pose->x;
            float turtle1_y = turtle1_pose->y;
            float turtle1_theta = turtle1_pose->theta;
            // 计算原生乌龟与目标乌龟的x向以及y向距离；
            float x_distance = goal_x - turtle1_x;
            float y_distance = goal_y - turtle1_y;


            // 计算速度
            geometry_msgs::msg::Twist twist;
            double scale = 0.5;
            twist.linear.x = scale * x_distance;
            twist.linear.y = scale * y_distance;

            cmd_vel_pub->publish(twist);
            // 计算剩余距离
            float distance = sqrt(pow(x_distance,2) + pow(y_distance,2));

            // 当两龟距离小于0.15米时，将当前乌龟位姿设置进result并退出循环
            if (distance < 0.15)
            {   
                //将当前乌龟坐标赋值给 result
                result->turtle_x = turtle1_x;
                result->turtle_y = turtle1_y;
                result->turtle_theta = turtle1_theta;
                break;
            }
            // 为feedback设置数据并发布
            feedback->distance = distance;

            goal_handle->publish_feedback(feedback);

            rate.sleep();
        }
        // 设置最终响应结果

        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(),"任务结束!");
        }


    }
    // 3-6.创建新线程处理请求；
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> goal_handle){
        std::thread{std::bind(&ExeNavActionServer::execute,this,_1),goal_handle}.detach();
    }


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