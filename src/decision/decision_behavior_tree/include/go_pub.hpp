#pragma once

#ifndef DECISION_BEHAVIOR_TREE__GO_PUB_HPP_
#define DECISION_BEHAVIOR_TREE__GO_PUB_HPP_

#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>



namespace decision_behavior_tree
{
using namespace std::chrono_literals;

    auto pow2 = [](float x) {
        return std::pow(x, 2);
    };    
    class GoPublisher : public BT::StatefulActionNode
    {
    public:
        GoPublisher(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
        {
            go_pub_node = rclcpp::Node::make_shared("go_publisher_node");
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(go_pub_node->get_clock()); 
            tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            publisher_ = go_pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::SystemDefaultsQoS());
        };




        static BT::PortsList providedPorts()
        {
            return { 
                BT::OutputPort<BT::NodeStatus>("result"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>("supply_pose"),
                BT::InputPort<bool>("if_supply"),
                BT::InputPort<geometry_msgs::msg::PoseStamped>("born_pose"),
                // BT::InputPort<bool>("if_patrol")
            }; 
        }

    private:
        /// Method called once, when transitioning from the state IDLE.
        /// If it returns RUNNING, this becomes an asynchronous node.
        BT::NodeStatus onStart()
        {
            // 判断发布哪个目标点
            std::vector<std::string> judge_ports = {"if_supply"};
            bool flag_port_value = false;
            for(const auto& port : judge_ports)
            {
                auto value = getInput<bool>(port);
                if(value.value())
                {
                    std::istringstream iss(port);
                    std::string firstPart, secondPart;
                    std::getline(iss, firstPart, '_');
                    std::getline(iss, secondPart);
                    std::string mergedString = secondPart + "_pose";
                    goal_pose_ = getInput<geometry_msgs::msg::PoseStamped>(mergedString).value();
                    flag_port_value = true;
                    break;
                }
            }
            
            if(flag_port_value)
            {

                RCLCPP_INFO(go_pub_node->get_logger(), "导航信息:x=%lf,y=%lf,w=%lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.orientation.w);
                publisher_->publish(goal_pose_);
                return BT::NodeStatus::RUNNING;
            }
            else 
            {
                RCLCPP_INFO(go_pub_node->get_logger(),"未接收到目标点数据----------");
                return BT::NodeStatus::FAILURE;
            }
        }

        /// method invoked when the action is already in the RUNNING state.
        BT::NodeStatus onRunning()
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            try{
            transformStamped = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(go_pub_node->get_logger(), "%s", ex.what());
            }
            // 等待动作完成，将输出端口result改为SUCCESS再返回SUCCESS

            auto IsInDistance = std::bind(&GoPublisher::isindistance, this, transformStamped.transform.translation, goal_pose_.pose);
            std::cout << "1111111" << std::endl;
            if (IsInDistance())
            {
                std::cout<<"InDistance!"<<"\n";
                setOutput<BT::NodeStatus>("result", BT::NodeStatus::SUCCESS);
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(go_pub_node->get_logger(), "导航信息:x=%lf,y=%lf,w=%lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.orientation.w);
            publisher_->publish(goal_pose_);
            rclcpp::sleep_for(std::chrono::seconds(1s));
            return BT::NodeStatus::RUNNING;
        }

        /// when the method halt() is called and the action is RUNNING, this method is invoked.
        /// This is a convenient place todo a cleanup, if needed.
        void onHalted()
        {
            RCLCPP_INFO(go_pub_node->get_logger(),"已中断----------");
            setOutput<BT::NodeStatus>("result",BT::NodeStatus::FAILURE);

        };
        

    
        bool isindistance(geometry_msgs::msg::Vector3 now_pose,geometry_msgs::msg::Pose goal_pose){
            auto distance2 = pow2(now_pose.x-goal_pose.position.x)+pow2(now_pose.y-goal_pose.position.y);
            if(pow2(tolerance_distance) >= distance2)
                return true;
            return false;
        };
        
    private:
        float tolerance_distance = 0.5; //尽量写指针
        geometry_msgs::msg::PoseStamped goal_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(go_pub_node->get_clock());
        // std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        std::shared_ptr<rclcpp::Node> go_pub_node ;

    };
} // namespace decision_behavior_tree

#endif // DECISION_BEHAVIOR_TREE__GO_PUB_HPP_