#pragma once

#ifndef DECISION_BEHAVIOR_TREE__GO_PUB_HPP_
#define DECISION_BEHAVIOR_TREE__GO_PUB_HPP_

#include <vector>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include <geometry_msgs/msg/polygon.h>


namespace decision_behavior_tree
{
using namespace std::chrono_literals;
    class GoPublisher : public BT::StatefulActionNode
    {
    public:
        GoPublisher(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config)
        {
            publisher_ = go_pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::SystemDefaultsQoS());
            subscription_ = go_pub_node->create_subscription<geometry_msgs::msg::PolygonStamped>("/global_costmap/published_footprint",
                            rclcpp::SystemDefaultsQoS(),std::bind(&GoPublisher::poseCallback,this,std::placeholders::_1));
        }
        //(const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback, const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> &options = rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(), MessageMemoryStrategyT::SharedPtr msg_mem_strat = MessageMemoryStrategyT::create_default())



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
            // 冻结一段时间等待动作完成，将输出端口result改为SUCCESS再返回SUCCESS
            if(!now_pose.points.empty()){
                auto distance_2 = (now_pose.points[0].x - goal_pose_.pose.position.x)*(now_pose.points[0].x - goal_pose_.pose.position.x) + 
                                    (now_pose.points[0].y - goal_pose_.pose.position.y)*(now_pose.points[0].y - goal_pose_.pose.position.y);
                std::cout << "now_pose: x: " << now_pose.points[0].x << ", now_pose: y: " << now_pose.points[0].y << std::endl;
                if(distance_2 <= 0.5*0.5)
                {
                    setOutput<BT::NodeStatus>("result", BT::NodeStatus::SUCCESS);
                    return BT::NodeStatus::SUCCESS;
                }
            }
            return BT::NodeStatus::RUNNING;

        }

        /// when the method halt() is called and the action is RUNNING, this method is invoked.
        /// This is a convenient place todo a cleanup, if needed.
        void onHalted()
        {
            RCLCPP_INFO(go_pub_node->get_logger(),"已中断----------");
            setOutput<BT::NodeStatus>("result",BT::NodeStatus::FAILURE);

        };
        
        void poseCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr footprint) 
        {
            now_pose = footprint->polygon;
        };
    
    private:
        geometry_msgs::msg::Polygon now_pose;
        std::shared_ptr<rclcpp::Node>  go_pub_node = rclcpp::Node::make_shared("go_publisher_node");
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr subscription_;
        geometry_msgs::msg::PoseStamped goal_pose_;

    };
} // namespace decision_behavior_tree

#endif // DECISION_BEHAVIOR_TREE__GO_PUB_HPP_