#ifndef FIRE_OR_SKIP_HPP
#define FIRE_OR_SKIP_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "rm_behavior_tree/call_for_refereesystem.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rm_behavior_tree
{
    class FireOrSkip : public BT::StatefulActionNode{
    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
        std::chrono::milliseconds wait_time;
        std::chrono::system_clock::time_point _completion_time;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> gimbal_spin_pub;

    public:
        FireOrSkip(const std::string &name, const BT::NodeConfig &conf) : BT::StatefulActionNode(name, conf), wait_time(0){
            blackboard_ = config().blackboard;
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
            gimbal_spin_pub = blackboard_->get<std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>>("gimbal_spin_pub");

        }


        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<int>("wait_time"),
            };
        }

        BT::NodeStatus onStart() override{
            // blackboard_->set<bool>("search_mode",true);
            geometry_msgs::msg::Twist send_data;
            send_data.angular.z = 2.2;
            RCLCPP_INFO(node_->get_logger(),"gimbal w:2.2 ...");
            gimbal_spin_pub->publish(send_data);

            blackboard_->set<bool>("nav_mode",false);
            int temp_time;
            getInput("wait_time", temp_time);
            wait_time = std::chrono::milliseconds(temp_time*1000);
            if (wait_time <= std::chrono::milliseconds(0)){
                RCLCPP_INFO(node_->get_logger(),"wait_time too short, action over.");
                return BT::NodeStatus::SUCCESS;
            } else if(temp_time == 4){
                auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
                call_for_refereesystem_node->processResponse(0x0001); // game_status_t
                while(!call_for_refereesystem_node->checkResponseReceived()) {
                    sleep(0.1);
                };
                auto stage_remain_time=blackboard_->get<uint16_t>("GameStatusStruct.stage_remain_time");
                auto current_get_bullet_count = blackboard_->get<int>("get_bullet_count");
                if((420-stage_remain_time)/60 > current_get_bullet_count){
                    blackboard_->set<int>("get_bullet_count", current_get_bullet_count+1);
                    RCLCPP_INFO(node_->get_logger(),"get_bullet_count:%d",current_get_bullet_count+1);
                }
            }
            _completion_time = std::chrono::system_clock::now() + std::chrono::duration_cast<std::chrono::milliseconds>(wait_time);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override{
            RCLCPP_INFO(node_->get_logger(),"Firing or Skipping continuing...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if(std::chrono::system_clock::now() >= _completion_time){
                RCLCPP_INFO(node_->get_logger(),"wait finished...");
                // blackboard_->set<bool>("search_mode",false);
                blackboard_->set<bool>("nav_mode",true);
                geometry_msgs::msg::Twist send_data;
                send_data.angular.z = 0;
                RCLCPP_INFO(node_->get_logger(),"gimbal w:0...");
                
                gimbal_spin_pub->publish(send_data);

                return BT::NodeStatus::SUCCESS;
            }
            geometry_msgs::msg::Twist send_data;
            send_data.angular.z = 2.2;
            RCLCPP_INFO(node_->get_logger(),"gimbal w:2.2 ...");
            gimbal_spin_pub->publish(send_data);
            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override{
            RCLCPP_INFO(node_->get_logger(),"wait interrupted...");
        }

        ~FireOrSkip(){
            // halt();
        };
    };
}
#endif
