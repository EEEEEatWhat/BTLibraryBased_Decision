#ifndef RM_BEHAVIOR_TREE_HPP
#define RM_BEHAVIOR_TREE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "geometry_msgs/msg/twist.hpp"

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
        const std::string tree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/rmucTree.xml";
        const std::string Atree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/rmucTreeA.xml";
        const std::string Btree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/rmucTreeB.xml";
        const std::string Ctree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/rmucTreeC.xml";
        const std::string Dtree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/rmucTreeD.xml";
        const std::string test_tree_path = "/home/eatwhat/ws01_decision/src/rm_behavior_tree/tree/test_tree.xml";
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gimbal_spin_pub ;

    public:
        rclcpp::Node::SharedPtr decision_node;
        RMBehaviorTree(const rclcpp::NodeOptions &options)
        :rclcpp::Node("rm_behavior_tree",options) , options_(options){
            RCLCPP_INFO(this->get_logger(),"Start RM_Behavior_Tree!");
            blackboard_ = BT::Blackboard::create();
            decision_node = std::make_shared<rclcpp::Node>("decision_node");  
            blackboard_->set<rclcpp::Node::SharedPtr>("decision_node",decision_node);
            params_decision.nh = decision_node;
            call_for_refereesystem_node = std::make_shared<rm_behavior_tree::CallForRefereeSystem>(blackboard_);
            blackboard_->set<std::shared_ptr<rm_behavior_tree::CallForRefereeSystem>>("call_for_refereesystem_node",call_for_refereesystem_node);
            gimbal_spin_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::SystemDefaultsQoS());
            blackboard_->set<std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>>>("gimbal_spin_pub",gimbal_spin_pub);
            // ROS Node
            msg_update_plugin_libs = {
                "send_sentrycmd",
                "send_goal",
                "gimbal_spin",
                // "pub_goal",
                "armors_check",
            };
            bt_plugin_libs = {
                "chassis_spin",
                "fire_or_skip",
                "nav2pose",
                "game_status_check",
                "hp_check",
                "supply_zone_check",
                "check_bullet",
                "check_aerial_cmd",
                "is_dead_check",
                "is_arrived",
                "reborn_now",
                "set_enemy_goal",
                "reset_res_data",
                "our_outpost_check",
                "check_game_started",
                "check_shooter_status",
            };
            this->decode_config();
            this->run();
        };

        void decode_config(){
            bool en_test;
            bool en_instaRes;
            bool en_gimbal_spin;
            bool en_chassis_spin;
            bool en_chase_enemy;
            double tracking_scope;
            double gimbal_angular_vel;
            int chassis_angular_vel;
            int chassis_angular_vel_slow;
            std::string plan;
            this->declare_parameter("en_test", false);
            this->declare_parameter("en_instaRes", false);
            this->declare_parameter("en_gimbal_spin", true);
            this->declare_parameter("en_chassis_spin", true);
            this->declare_parameter("en_chase_enemy", false);
            this->declare_parameter("tracking_scope", 5.0);
            this->declare_parameter("gimbal_angular_vel", 1.2);
            this->declare_parameter("chassis_angular_vel", 6);
            this->declare_parameter("chassis_angular_vel_slow", 3);
            this->declare_parameter("plan","A");
            this->get_parameter("en_test", en_test);
            this->get_parameter("en_instaRes", en_instaRes);
            this->get_parameter("en_gimbal_spin", en_gimbal_spin);
            this->get_parameter("en_chassis_spin", en_chassis_spin);
            this->get_parameter("en_chase_enemy", en_chase_enemy);
            this->get_parameter("tracking_scope", tracking_scope);
            this->get_parameter("gimbal_angular_vel", gimbal_angular_vel);
            this->get_parameter("chassis_angular_vel", chassis_angular_vel);
            this->get_parameter("chassis_angular_vel_slow", chassis_angular_vel_slow);
            this->get_parameter("plan", plan);
            blackboard_->set<bool>("en_test", en_test);
            blackboard_->set<bool>("en_instaRes", en_instaRes);
            blackboard_->set<bool>("en_gimbal_spin", en_gimbal_spin);
            blackboard_->set<bool>("en_chassis_spin", en_chassis_spin);
            blackboard_->set<bool>("en_chase_enemy", en_chase_enemy);
            blackboard_->set<double>("tracking_scope", tracking_scope);
            blackboard_->set<double>("gimbal_angular_vel", gimbal_angular_vel);
            blackboard_->set<int>("chassis_angular_vel", chassis_angular_vel);
            blackboard_->set<int>("chassis_angular_vel_slow", chassis_angular_vel_slow);
            blackboard_->set<std::string>("plan",plan);

            blackboard_->set<std::string>("game_stage", "not_started");
            blackboard_->set<int>("res_count", 0); // 累积复活次数
            blackboard_->set<std::string>("own_status", "alive");
            blackboard_->set<bool>("nav_mode", true);
            // blackboard_->set<bool>("search_mode", false);
            blackboard_->set<int>("send_goal_timeout_ms", 60000);
            blackboard_->set<int>("our_outpost_hp", 1500);
            blackboard_->set<int>("get_bullet_count", 0); // 每隔一分钟允许到补血点获取允许发弹量

            // For Sentrycmd
            blackboard_->set<uint8_t>("confirmRes", 0);
            blackboard_->set<uint8_t>("confirmInstaRes", 0);
            blackboard_->set<uint16_t>("pendingMissileExch", 0);
            blackboard_->set<uint8_t>("remoteMissileReqCount", 0);
            blackboard_->set<uint8_t>("remoteHealthReqCount", 0);

            // For goal pose
            std::map<std::string, geometry_msgs::msg::PoseStamped> poses_map = {
                {"home_pose", geometry_msgs::msg::PoseStamped()},
                {"supply_zone_pose", geometry_msgs::msg::PoseStamped()},
                {"goal_pose", geometry_msgs::msg::PoseStamped()},
                {"attack_enemy_outpost_pose", geometry_msgs::msg::PoseStamped()},
                {"circular_highland_pose", geometry_msgs::msg::PoseStamped()},
                {"patrol_1", geometry_msgs::msg::PoseStamped()},
                {"patrol_2", geometry_msgs::msg::PoseStamped()},
                {"patrol_3", geometry_msgs::msg::PoseStamped()},
                {"patrol_4", geometry_msgs::msg::PoseStamped()},
                {"patrol_5", geometry_msgs::msg::PoseStamped()},
                {"patrol_6", geometry_msgs::msg::PoseStamped()},
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
            std::queue<geometry_msgs::msg::PoseStamped> home_patrol_points;
            std::queue<geometry_msgs::msg::PoseStamped> hero_patrol_points;
            std::queue<geometry_msgs::msg::PoseStamped> our_outpost_patrol_points;
            home_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_1"));
            home_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_2"));
            // patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_21"));
            blackboard_->set<std::queue<geometry_msgs::msg::PoseStamped>>( "home_patrol_points", home_patrol_points);
            hero_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_3"));
            hero_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_4"));
            blackboard_->set<std::queue<geometry_msgs::msg::PoseStamped>>( "hero_patrol_points", hero_patrol_points);
            our_outpost_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_5"));
            our_outpost_patrol_points.emplace(blackboard_->get<geometry_msgs::msg::PoseStamped>("patrol_6"));
            blackboard_->set<std::queue<geometry_msgs::msg::PoseStamped>>( "our_outpost_patrol_points", our_outpost_patrol_points);
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
            } else if(blackboard_->get<std::string>("plan") == "A"){
                factory.registerBehaviorTreeFromFile(Atree_path);
            } else if(blackboard_->get<std::string>("plan") == "B"){
                factory.registerBehaviorTreeFromFile(Btree_path);
            } else if(blackboard_->get<std::string>("plan") == "C"){
                factory.registerBehaviorTreeFromFile(Ctree_path);
            } else if(blackboard_->get<std::string>("plan") == "D"){
                factory.registerBehaviorTreeFromFile(Dtree_path);
            } else {
                factory.registerBehaviorTreeFromFile(tree_path);
            }
            auto tree_ = factory.createTree("mainTree",blackboard_);

            BT::NodeStatus status = tree_.tickOnce();

            while(rclcpp::ok()){
                tree_.sleep(std::chrono::milliseconds(100));
                tree_.tickOnce();
            }

        };

        ~RMBehaviorTree(){};
    };
    
    
}

#endif