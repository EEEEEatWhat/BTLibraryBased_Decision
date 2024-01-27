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
        }
        
        BT::NodeStatus Check_blood()
        {
            lowest_HP = blackboard_->get<uint16_t>("lowest_HP");

            // TODO: 通过解包函数解析从服务端拿到的字节流并解析得到当前血量
            // RobotState = 0x0201
            // current_blood = // 裁判系统 参数服务器

            // for test
            current_HP = 150;

            if (current_HP < lowest_HP)
            {
                RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.\n",current_HP,lowest_HP);
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus Check_enemy()
        {
            if(blackboard_->get<uint8_t>("if_find_enemy"))
            {
                RCLCPP_INFO(node_->get_logger(),"发现敌人，开始打弹.\n");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus Check_game_status()
        {
            // TODO: 通过解包函数解析字节流，获取裁判系统中的比赛状态（包括比赛时间、）
            // GameStatus = 0x0001,

            return BT::NodeStatus::SUCCESS;
        }


        ~Condition()
        {}
    };
    
}
#endif