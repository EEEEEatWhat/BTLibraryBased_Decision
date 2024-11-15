#ifndef SEND_GOAL_HPP
#define SEND_GOAL_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"
#include "rm_behavior_tree/call_for_refereesystem.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rm_behavior_tree{
    class SendGoal : public BT::RosActionNode<global_interfaces::action::BehaviorTreePose>
    {
    public:
        SendGoal(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
            : BT::RosActionNode<global_interfaces::action::BehaviorTreePose>(name, conf, params)
        {
            blackboard_ = config().blackboard;
            call_for_refereesystem_node = blackboard_->get<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node");
            is_patrol_start = false;
            gimbal_spin_pub = blackboard_->get<std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>>("gimbal_spin_pub");
        };

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("action_name"),
                BT::InputPort<std::string>("goal_name"),
                // BT::InputPort<uint16_t>("hp_threshold"),
                BT::InputPort<int>("patrol_num"),
            };
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {
            geometry_msgs::msg::Twist send_data;
            send_data.angular.z = 0;
            RCLCPP_INFO(node_->get_logger(),"gimbal w:0 ...");

            gimbal_spin_pub->publish(send_data);

            blackboard_->set<bool>("nav_mode",true);
            // blackboard_->set<bool>("search_mode",false);
            std::string goal_name;
            getInput("goal_name", goal_name);
            if(goal_name == "patrol_points" || goal_name == "hero_patrol_points" || goal_name == "our_outpost_patrol_points"){
                if(!is_patrol_start){
                    is_patrol_start = true;
                    int patrol_num = 0;
                    getInput("patrol_num", patrol_num);
                    std::queue<geometry_msgs::msg::PoseStamped> temp_points = blackboard_->get<std::queue<geometry_msgs::msg::PoseStamped>>(goal_name);
                    for(int i=0; i<patrol_num; i++){
                        patrol_points.push(temp_points.front());
                        temp_points.pop();
                    }
                }
                if(patrol_points.empty()){
                    RCLCPP_ERROR(node_->get_logger(),"patrol_points is empty!");
                    return false;
                }
                auto temp_pose = patrol_points.front();
                patrol_points.pop();
                patrol_points.emplace(temp_pose);
                goal.set__pose(temp_pose);
                RCLCPP_INFO(node_->get_logger(),"set goal for %s successfully...",goal_name.c_str());
                return true;
            }
            goal.set__pose(blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name));
            RCLCPP_INFO(node_->get_logger(),"set goal %s successfully...", goal_name.c_str());
            return true;
        };


        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            if(getInput<std::string>("goal_name") != "supply_zone_pose"){
                blackboard_->set<bool>("nav_mode",false);
                // blackboard_->set<bool>("search_mode",true);
            }
            geometry_msgs::msg::Twist send_data;
            send_data.angular.z = 2.2;
            RCLCPP_INFO(node_->get_logger(),"gimbal w:2.2 ...");
            gimbal_spin_pub->publish(send_data);
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
                    return BT::NodeStatus::SUCCESS;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_INFO(node_->get_logger(), "Goal was aborted");
                    return BT::NodeStatus::FAILURE;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
                    return BT::NodeStatus::FAILURE;
                    break;
                default:
                    RCLCPP_INFO(node_->get_logger(), "Unknown result code");
                    return BT::NodeStatus::FAILURE;
                    break;
            }
        };


        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %s", toStr(error));
            return BT::NodeStatus::FAILURE;
        };


        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override
        {
            (void)feedback;
            RCLCPP_INFO(node_->get_logger(),"Feedback....");
            // call_for_refereesystem_node->processResponse(0x0201);
            // while(!call_for_refereesystem_node->checkResponseReceived()) {
            //     sleep(0.1);
            // };
            // uint16_t current_hp = blackboard_->get<uint16_t>("RobotStateStruct.current_HP");
            // uint16_t hp_threshold;
            // getInput("hp_threshold", hp_threshold);
            // if(current_hp < hp_threshold){
            //     RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.",current_hp,hp_threshold);
            //     // cancel goal 直接在feedback里面返回FAILURE或SUCCESS
            //     return BT::NodeStatus::FAILURE;
            // }
            // RCLCPP_INFO(node_->get_logger(),"current_hp:%u", current_hp);
            return BT::NodeStatus::RUNNING;
        };

        void onHalt() {
            RCLCPP_INFO(node_->get_logger(),"halt...");
        }
        
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rm_behavior_tree::CallForRefereeSystem> call_for_refereesystem_node;
        std::queue<geometry_msgs::msg::PoseStamped> patrol_points;
        bool is_patrol_start;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> gimbal_spin_pub;
    };
}

#endif //SEND_GOAL_HPP