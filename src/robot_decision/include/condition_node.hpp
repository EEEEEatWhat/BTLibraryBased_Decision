#ifndef CONDITION_NODE_HPP_
#define CONDITION_NODE_HPP_

#include "behaviortree_cpp/behavior_tree.h"


namespace robot_decision{
    /* 提供检查状态的各项接口 */
    class Condition
    {
    private:
        std::shared_ptr<rclcpp::Node> node_;
        BT::Blackboard::Ptr blackboard_;
        uint16_t lowest_HP;
        uint16_t current_HP; 
    public:
        Condition(BT::Blackboard::Ptr blackboard_): blackboard_(blackboard_)
        {
            node_ = blackboard_->get<rclcpp::Node::SharedPtr>("node");
        };
        
        BT::NodeStatus Check_game_started()
        {

            if (blackboard_->get<uint8_t>("game_progress") == 4)
            {
                RCLCPP_INFO(node_->get_logger(),"game start!");
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_INFO(node_->get_logger(),"game hasn't started yet!");
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_blood()
        {
            lowest_HP = blackboard_->get<uint16_t>("lowest_HP");

            // TODO: 从裁判系统得到当前血量
            // RobotState = 0x0201
            // current_blood = 

            // for test
            current_HP = 150;

            if (current_HP < lowest_HP)
            {
                RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.\n",current_HP,lowest_HP);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Check_enemy()
        {
            // 从视觉获取是否发现敌人
            // if(blackboard_->get<uint8_t>("if_find_enemy"))
            if(1)
            {
                RCLCPP_INFO(node_->get_logger(),"发现敌人，开始打弹.\n");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus Set_strategy()
        {
            if(blackboard_->get<int>("strategy") == -1)
            {
                RCLCPP_INFO(node_->get_logger(),"open stage strategy hasn't set!");
                return BT::NodeStatus::FAILURE;
            }                
            int strategy;
            this->declare_parameter<int>("strategy", -1);
            this->get_parameter<int>("strategy", strategy);
            blackboard_->set<int>("strategy", strategy);
            RCLCPP_INFO(node_->get_logger(),"open stage strategy has set to %d.",blackboard_->get<int>("strategy"));
            return BT::NodeStatus::SUCCESS;
            // TODO：考虑是否需要在比赛后期让哨兵回到自家启动区
        };

        BT::NodeStatus Stay()
        {
            // when strategy is set to 4
            // skip the switch node.
            return BT::NodeStatus::SUCCESS;
        };
        
        ~Condition()
        {}
    };
    
}
#endif