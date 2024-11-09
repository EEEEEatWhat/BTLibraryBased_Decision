#ifndef CHECK_BULLET_HPP
#define CHECK_BULLET_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rm_behavior_tree/public.hpp"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class CheckBullet : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    public:
    CheckBullet(const std::string & name, const BT::NodeConfig & conf)
        : BT::SimpleConditionNode(name, std::bind(&CheckBullet::chekc_bullet, this), conf){
        blackboard_ = config().blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        
    }



    static BT::PortsList providedPorts(){
        return {
            BT::OutputPort<bool>("en_bullet_supply"),
        };
    }

    BT::NodeStatus chekc_bullet(){
        auto call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
        call_for_refereesystem_node->processResponse(0x0208); // projectile_allowance_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        call_for_refereesystem_node->processResponse(0x0001); // game_status_t
        while(!call_for_refereesystem_node->checkResponseReceived()) {
            sleep(0.1);
        };
        auto projectile_allowance_17mm = blackboard_->get<uint16_t>("ProjectileAllowanceStruct.projectile_allowance_17mm");
        auto stage_remain_time = blackboard_->get<uint16_t>("GameStatusStruct.stage_remain_time");
        auto current_get_bullet_count = blackboard_->get<int>("get_bullet_count");
        RCLCPP_INFO(node_->get_logger(),"projectile_allowance_17mm :%d",projectile_allowance_17mm);
        if(projectile_allowance_17mm == 0 && (420-stage_remain_time)/60 > current_get_bullet_count){
            blackboard_->set<int>("get_bullet_count", current_get_bullet_count+1);
            RCLCPP_INFO(node_->get_logger(),"get_bullet_count:%d",current_get_bullet_count+1);
            setOutput("en_bullet_supply", true);
        } else {
            setOutput("en_bullet_supply", false);
        }
        return BT::NodeStatus::SUCCESS;
    }


    };
}  // namespace rm_behavior_tree

#endif

