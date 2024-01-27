#ifndef GAIN_BLOOD_ACTION_HPP_
#define GAIN_BLOOD_ACTION_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/blackboard.h"


namespace robot_decision
{
    class GainBloodAction : public BT::StatefulActionNode
    {
    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;

        std::chrono::milliseconds wait_time;
        std::chrono::system_clock::time_point _completion_time;

    public:
        GainBloodAction(const std::string &name, const BT::NodeConfig &conf) : BT::StatefulActionNode(name, conf), wait_time(0)
        {
            blackboard_ = config().blackboard;
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("node");
        }

        static BT::PortsList providedPorts()
        {
            return{};
        }

        BT::NodeStatus onStart() override
        {
            // TODO:获取当前比赛时间和自身血量，修改wait_time赋值
            // 前三分钟回血增益为上限血量的10%，比赛开始的四分钟后回血增益为上限血量的25%
            
            //for test
            wait_time = std::chrono::milliseconds(2000);
            
            if (0)
            {
                throw BT::RuntimeError("Missing parameter [wait_time] in GainBloodActionNode");
            }

            if (wait_time <= std::chrono::milliseconds(0))
            {
                RCLCPP_INFO(node_->get_logger(),"wait_time too short, action over.");
                return BT::NodeStatus::SUCCESS;
            }

            _completion_time = std::chrono::system_clock::now() + std::chrono::duration_cast<std::chrono::milliseconds>(wait_time);
            
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            RCLCPP_INFO(node_->get_logger(),"Gain blood continuing...");
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
            if(std::chrono::system_clock::now() >= _completion_time)
            {
                RCLCPP_INFO(node_->get_logger(),"Gain blood finished...");
                
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;

        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(),"Gain blood interrupted...");
        }

        ~GainBloodAction(){};
    };
    

}
#endif