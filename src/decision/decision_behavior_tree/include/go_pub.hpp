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



namespace decision_behavior_tree
{
using namespace std::chrono_literals;
    class GoPublisher : public BT::SyncActionNode
    {
    public:
        GoPublisher(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::OutputPort<BT::NodeStatus>("result"),
                    BT::InputPort<geometry_msgs::msg::PoseStamped>("supply_pose"),
                    BT::InputPort<bool>("if_supply"),
                    BT::InputPort<geometry_msgs::msg::PoseStamped>("born_pose"),
                    // BT::InputPort<bool>("if_patrol")
            }; 
        }

    private:

        BT::NodeStatus tick() override
        {
            // 判断发布哪个目标点
            geometry_msgs::msg::PoseStamped goal_pose_;
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
                publisher_ = go_pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::SystemDefaultsQoS());

                RCLCPP_INFO(go_pub_node->get_logger(), "导航信息:x=%lf,y=%lf,w=%lf", goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.orientation.w);
                publisher_->publish(goal_pose_);

                // TODO: (suzukisuncy)来点nav2回传数据,判断动作是否完成
                // 冻结一段时间等待动作完成，将输出端口result改为SUCCESS再返回SUCCESS


                setOutput<BT::NodeStatus>("result",BT::NodeStatus::SUCCESS);
                return BT::NodeStatus::SUCCESS;
            }
            else 
            {
                RCLCPP_INFO(go_pub_node->get_logger(),"未接收到目标点数据----------");
                return BT::NodeStatus::FAILURE;
            }


        }



        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<rclcpp::Node>  go_pub_node = rclcpp::Node::make_shared("go_publisher_node");
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    };
} // namespace decision_behavior_tree

#endif // DECISION_BEHAVIOR_TREE__GO_PUB_HPP_