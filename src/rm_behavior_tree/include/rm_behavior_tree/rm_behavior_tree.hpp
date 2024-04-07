#ifndef RM_BEHAVIOR_TREE_HPP
#define RM_BEHAVIOR_TREE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

#include "public.hpp"
#include "call_for_refereesystem.hpp"

namespace rm_behavior_tree{
    class RMBehaviorTree : public rclcpp::Node
    {
    private:
        rclcpp::NodeOptions options_;
        BT::BehaviorTreeFactory factory;
        BT::Blackboard::Ptr blackboard_;
        BT::RosNodeParams params_decision;
        std::vector<std::string> msg_update_plugin_libs;
        std::vector<std::string> bt_plugin_libs;
        std::shared_ptr<rm_behavior_tree::CallForRefereeSystem> call_for_refereesystem_node;
        const std::string tree_path = "/home/hannah/BTLibraryBased_Decision/src/rm_behavior_tree/tree/rmucTree.xml";
        const std::string test_tree_path = "/home/hannah/BTLibraryBased_Decision/src/rm_behavior_tree/tree/test_tree.xml";

    public:
        RMBehaviorTree(const rclcpp::NodeOptions &options)
        :rclcpp::Node("rm_behavior_tree",options) , options_(options){
            RCLCPP_INFO(this->get_logger(),"Start RM_Behavior_Tree!");
            blackboard_ = BT::Blackboard::create();
            auto decision_node = std::make_shared<rclcpp::Node>("decision");  
            blackboard_->set<rclcpp::Node::SharedPtr>("decision_node",decision_node);
            params_decision.nh = decision_node;
            call_for_refereesystem_node = std::make_shared<rm_behavior_tree::CallForRefereeSystem>(blackboard_);
            blackboard_->set<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node",call_for_refereesystem_node);
            // ROS Node
            msg_update_plugin_libs = {
                "send_sentrycmd",
                "send_goal",
                "set_own_status",
            };
            bt_plugin_libs = {
                "fire_or_skip",
                "game_status_check",
                "hp_check",
                "supply_zone_check",
                "check_armors",
                "is_dead_check",
                "is_arrived",
                "reborn_now",
                "set_enemy_goal",
                // "reset_res_data",
                // "set_supply_goal",
            };
            this->decode_config();
            this->run();
        };

        void decode_config(){
            bool en_test;
            bool en_instaRes;
            bool en_gimbal_spin;
            bool en_chassis_spin;
            uint16_t hp_threshold;
            bool en_chase_enemy;
            double tracking_scope;
            double gimbal_angular_vel;
            int chassis_angular_vel;
            this->declare_parameter("en_test", false);
            this->declare_parameter("en_instaRes", false);
            this->declare_parameter("en_gimbal_spin", true);
            this->declare_parameter("en_chassis_spin", true);
            this->declare_parameter("hp_threshold", 100);
            this->declare_parameter("en_chase_enemy", false);
            this->declare_parameter("tracking_scope", 5.0);
            this->declare_parameter("gimbal_angular_vel", 1.2);
            this->declare_parameter("chassis_angular_vel", 6);
            this->get_parameter("en_test", en_test);
            this->get_parameter("en_instaRes", en_instaRes);
            this->get_parameter("en_gimbal_spin", en_gimbal_spin);
            this->get_parameter("en_chassis_spin", en_chassis_spin);
            this->get_parameter("hp_threshold", hp_threshold);
            this->get_parameter("en_chase_enemy", en_chase_enemy);
            this->get_parameter("tracking_scope", tracking_scope);
            this->get_parameter("gimbal_angular_vel", gimbal_angular_vel);
            this->get_parameter("chassis_angular_vel", chassis_angular_vel);
            blackboard_->set<bool>("en_test", en_test);
            blackboard_->set<bool>("en_instaRes", en_instaRes);
            blackboard_->set<bool>("en_gimbal_spin", en_gimbal_spin);
            blackboard_->set<bool>("en_chassis_spin", en_chassis_spin);
            blackboard_->set<uint16_t>("hp_threshold", hp_threshold);
            blackboard_->set<bool>("en_chase_enemy", en_chase_enemy);
            blackboard_->set<double>("tracking_scope", tracking_scope);
            blackboard_->set<double>("gimbal_angular_vel", gimbal_angular_vel);
            blackboard_->set<int>("chassis_angular_vel", chassis_angular_vel);

            blackboard_->set<std::string>("game_stage", "not_started");
            blackboard_->set<int>("res_count", 0); // 累积复活次数
            blackboard_->set<bool>("initial_set_param_rotation", false);

            // For Sentrycmd
            blackboard_->set<uint8_t>("confirmRes", 0);
            blackboard_->set<uint8_t>("confirmInstaRes", 0);
            blackboard_->set<uint16_t>("pendingMissileExch", 0);
            blackboard_->set<uint8_t>("remoteMissileReqCount", 0);
            blackboard_->set<uint8_t>("remoteHealthReqCount", 0);

            // For goal pose
            std::map<std::string, geometry_msgs::msg::PoseStamped> poses_map = {
                {"home_pose", {}},
                {"supply_zone_pose", {}}
            };
            this->decode_goal_pose(poses_map, blackboard_);
        }

        void decode_goal_pose(std::map<std::string, geometry_msgs::msg::PoseStamped> poses_map, BT::Blackboard::Ptr blackboard_){
            for (auto it = poses_map.begin(); it != poses_map.end(); ++it)
            {
                const std::string pose_key = it->first;
                geometry_msgs::msg::PoseStamped temp_pose;
                geometry_msgs::msg::Quaternion geo_qnt;
                tf2::Quaternion tf_qnt;
                double temp_yaw;

                temp_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());

                std::string x_key = pose_key + ".x";
                std::string y_key = pose_key + ".y";
                std::string yaw_key = pose_key + ".yaw";

                this->declare_parameter<double>(x_key, 0.0);
                this->declare_parameter<double>(y_key, 0.0);
                this->declare_parameter<double>(yaw_key, 0.0);
                
                this->get_parameter<double>(x_key, temp_pose.pose.position.x);
                this->get_parameter<double>(y_key, temp_pose.pose.position.y);
                this->get_parameter<double>(yaw_key, temp_yaw);


                tf_qnt.setRPY( 0 , 0 , temp_yaw);
                temp_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
                temp_pose.pose.position.set__z(0.0);
                temp_pose.pose.set__orientation(convert<tf2::Quaternion, geometry_msgs::msg::Quaternion>(tf_qnt, geo_qnt, true));
        
                blackboard_->set<geometry_msgs::msg::PoseStamped>(pose_key,temp_pose);
            }
        }

        void load_plugins(){
            for (const auto & p : msg_update_plugin_libs) {
                RegisterRosNode(factory, BT::SharedLibrary::getOSName(p), params_decision);
            }
            for (const auto & p : bt_plugin_libs) {
                try {
                    std::string os_name = BT::SharedLibrary::getOSName(p);
                    factory.registerFromPlugin(os_name);
                } catch (const std::exception& e) {
                    RCLCPP_INFO(this->get_logger(),"Error while registering plugin %s", p.c_str());
                }
            }
            RCLCPP_INFO(this->get_logger(),"Load plugins successfully...");
        };

        void run(){
            this->load_plugins();
            if(blackboard_->get<bool>("en_test")){
                factory.registerBehaviorTreeFromFile(test_tree_path);
            } else {
                factory.registerBehaviorTreeFromFile(tree_path);
            }
            auto tree_ = factory.createTree("mainTree",blackboard_);

            BT::NodeStatus status = tree_.tickOnce();

            while(status == BT::NodeStatus::RUNNING && rclcpp::ok()){
                tree_.sleep(std::chrono::milliseconds(100));
                tree_.tickOnce();
            }

        };

        ~RMBehaviorTree(){};
    };
    
    
}

#endif