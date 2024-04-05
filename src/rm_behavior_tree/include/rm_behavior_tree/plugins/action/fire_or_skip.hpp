#ifndef FIRE_OR_SKIP_HPP
#define FIRE_OR_SKIP_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"


namespace rm_behavior_tree
{
    class FireOrSkip : public BT::StatefulActionNode{
    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
        std::chrono::milliseconds wait_time;
        std::chrono::system_clock::time_point _completion_time;

    public:
        FireOrSkip(const std::string &name, const BT::NodeConfig &conf) : BT::StatefulActionNode(name, conf), wait_time(0){
            blackboard_ = config().blackboard;
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        }


        static BT::PortsList providedPorts(){
            return {};
        }

        BT::NodeStatus onStart() override{
            wait_time = std::chrono::milliseconds(1500);
            if (wait_time <= std::chrono::milliseconds(0)){
                RCLCPP_INFO(node_->get_logger(),"wait_time too short, action over.");
                return BT::NodeStatus::SUCCESS;
            }
            _completion_time = std::chrono::system_clock::now() + std::chrono::duration_cast<std::chrono::milliseconds>(wait_time);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override{
            RCLCPP_INFO(node_->get_logger(),"Firing or Skipping continuing...");
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
            if(std::chrono::system_clock::now() >= _completion_time){
                RCLCPP_INFO(node_->get_logger(),"wait finished...");
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }

        void onHalted() override{
            RCLCPP_INFO(node_->get_logger(),"wait interrupted...");
        }

        ~FireOrSkip(){
            halt();
        };
    };
}
#endif
