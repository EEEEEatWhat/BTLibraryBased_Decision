#ifndef CHECK_ARMORS_HPP
#define CHECK_ARMORS_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rm_behavior_tree/public.hpp"

namespace rm_behavior_tree{
    class CheckArmors : public BT::SimpleConditionNode{
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub;
    public:
    CheckArmors(const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckArmors::chekc_armors, this), config){
        blackboard_ = config.blackboard;
        node_ = blackboard_->get<rclcpp::Node::SharedPtr>("decision_node");
        target_sub = node_->create_subscription<auto_aim_interfaces::msg::Target>
                ("tracker/target",rclcpp::SensorDataQoS(),std::bind(&CheckArmors::target_callback, this, std::placeholders::_1));
        blackboard_->set<std::string>("target_id", "");
        
    }

    void target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg){
        RCLCPP_INFO(rclcpp::get_logger("TEST"),"11111");
        if(msg->id != ""){
            blackboard_->set<std::string>("target_id", msg->id);
            get_enemy_pose(blackboard_, msg);
        }
    }

    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<std::string>("topic_name"),
        };
    }

    BT::NodeStatus chekc_armors(){
        auto id = blackboard_->get<std::string>("target_id");
        if(id != ""){
            RCLCPP_INFO(node_->get_logger(),"find enemy %s...", id.c_str());
            if(blackboard_->get<bool>("en_chase_enemy")){
                RCLCPP_INFO(node_->get_logger(),"chase enemy mode ...");
                get_my_pose(blackboard_, node_);
                auto enemy_x = blackboard_->get<double>("enemy_x");
                auto enemy_y = blackboard_->get<double>("enemy_y");
                auto my_x = blackboard_->get<double>("my_x");
                auto my_y = blackboard_->get<double>("my_y");
                double distance = sqrt(pow(enemy_x - my_x, 2) + pow(enemy_y - my_y, 2));
                if(distance > blackboard_->get<double>("tracking_scope")){
                    RCLCPP_INFO(node_->get_logger(),"enemy out of tracking scope...");
                    return BT::NodeStatus::FAILURE;
                } else {
                    RCLCPP_INFO(node_->get_logger(),"enemy in tracking scope...");
                }
            }
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(node_->get_logger(),"no enemy found...");
            return BT::NodeStatus::SUCCESS;
        }
    }


    };
}  // namespace rm_behavior_tree

#endif

