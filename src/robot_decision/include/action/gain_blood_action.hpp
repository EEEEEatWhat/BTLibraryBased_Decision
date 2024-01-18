#ifndef ROBOT_DECISION__INCLUDE__ACTION__GAIN_BLOOD_ACTION_HPP_
#define ROBOT_DECISION__INCLUDE__ACTION__GAIN_BLOOD_ACTION_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/utils/timer_queue.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/blackboard.h"
#include <logging.hpp>


namespace robot_decision
{
    class GainBloodAction : public BT::StatefulActionNode
    {
    private:
        std::shared_ptr<rclcpp::Node> node_;

        std::chrono::seconds wait_time;
        BT::Blackboard::Ptr blackboard_;

        BT::TimerQueue<> timer_;
        uint64_t timer_id_;

        std::atomic_bool timer_waiting_;
        std::mutex delay_mutex_;

    public:
        GainBloodAction(const std::string &name, const BT::NodeConfig &conf) : BT::StatefulActionNode(name, conf), wait_time(0)
        {
            blackboard_ = config().blackboard;
        }

        BT::NodeStatus onStart() override
        {
            // TODO:获取当前比赛时间和自身血量，修改代码逻辑
            // 前三分钟回血增益为上限血量的10%，比赛开始的四分钟后回血增益为上限血量的25%
            if (1)
            {
                throw BT::RuntimeError("Missing parameter [wait_time] in GainBloodActionNode");
            }

            if (wait_time <= std::chrono::seconds(0))
            {
                RCLCPP_INFO(node_->get_logger(),"所需等待时间过短，动作已结束。");
                return BT::NodeStatus::SUCCESS;
            }

            setStatus(BT::NodeStatus::RUNNING);

            timer_waiting_ = true;

            timer_id_ = timer_.add(std::chrono::duration_cast<std::chrono::milliseconds>(wait_time), [this](bool aborted){
            std::unique_lock<std::mutex> lk(delay_mutex_); 
            if (!aborted)
            {
            emitWakeUpSignal();
            }
            timer_waiting_ = false; });
            RCLCPP_INFO(node_->get_logger(),"开始回血增益...");

            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            RCLCPP_INFO(node_->get_logger(),"持续回血...");
            return timer_waiting_ ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
        }

        void onHalted() override
        {
            RCLCPP_INFO(node_->get_logger(),"回血打断...");
            timer_waiting_ = false;
            timer_.cancel(timer_id_);
        }

        ~GainBloodAction(){};
    };
    

}
#endif