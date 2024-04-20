#ifndef SEND_GOAL_HPP
#define SEND_GOAL_HPP


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_interfaces/action/behavior_tree_pose.hpp"
#include "rm_behavior_tree/call_for_refereesystem.hpp"

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
        };

        static BT::PortsList providedPorts(){
            return {
                BT::InputPort<std::string>("action_name"),
                BT::InputPort<std::string>("goal_name"),
                BT::InputPort<uint16_t>("hp_threshold"),
                BT::InputPort<int>("patrol_num"),
            };
        };

        bool setGoal(RosActionNode::Goal &goal) override
        {
            std::string goal_name;
            getInput("goal_name", goal_name);
            if(goal_name == "patrol_points"){
                if(!is_patrol_start){
                    is_patrol_start = true;
                    int patrol_num = 0;
                    getInput("patrol_num", patrol_num);
                    std::queue<geometry_msgs::msg::PoseStamped> temp_points = blackboard_->get<std::queue<geometry_msgs::msg::PoseStamped>>("patrol_points");
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
                RCLCPP_INFO(node_->get_logger(),"set goal for patrol successfully...");
                return true;
            }
            goal.set__pose(blackboard_->get<geometry_msgs::msg::PoseStamped>(goal_name));
            RCLCPP_INFO(node_->get_logger(),"set goal %s successfully...", goal_name.c_str());
            return true;
        };


        BT::NodeStatus onResultReceived(const WrappedResult &wr) override
        {
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    std::cout << "Success!!!" << '\n';
                    return BT::NodeStatus::SUCCESS;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    std::cout << "Goal was aborted" << '\n';
                    return BT::NodeStatus::FAILURE;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    std::cout << "Goal was canceled" << '\n';
                    return BT::NodeStatus::FAILURE;
                    break;
                default:
                    std::cout << "Unknown result code" << '\n';
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
            call_for_refereesystem_node->processResponse(0x0201);
            while(!call_for_refereesystem_node->checkResponseReceived()) {
                sleep(0.1);
            };
            uint16_t current_hp = blackboard_->get<uint16_t>("RobotStateStruct.current_HP");
            uint16_t hp_threshold;
            getInput("hp_threshold", hp_threshold);
            if(current_hp < hp_threshold){
                RCLCPP_INFO(node_->get_logger(),"当前血量：%u,血量低于预设的%u.",current_hp,hp_threshold);
                BT::RosActionNode<global_interfaces::action::BehaviorTreePose>::halt();
            }
            // BT::RosActionNode<global_interfaces::action::BehaviorTreePose>::halt();
            // RCLCPP_INFO(node_->get_logger(),"halt...");
            return BT::NodeStatus::RUNNING;
        };

        void onHalt() {
            RCLCPP_INFO(node_->get_logger(),"halt...");
        }
        
    private:
        BT::Blackboard::Ptr blackboard_;
        std::shared_ptr<rm_behavior_tree::CallForRefereeSystem> call_for_refereesystem_node;
        bool is_patrol_start;
        std::queue<geometry_msgs::msg::PoseStamped> patrol_points;
    };
}

#endif //SEND_GOAL_HPP