#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "yaml-cpp/yaml.h"
#include "yamlsetup.h"
#include <memory>

    template <class A, class B>
    B convert(const A & a, B & b,bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    };

        struct Pose
        {
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
        };
class Node : public rclcpp::Node
{
private:
    /* data */
public:
    Node(const rclcpp::NodeOptions &options)
        :rclcpp::Node("decision_node",options){};
    bool LoadParam(std::string yaml_file,BT::Blackboard::Ptr blackboard_);
    
    ~Node (){};
};



    bool Node::LoadParam(std::string yaml_file,BT::Blackboard::Ptr blackboard_)
    {
        YAML::Node yaml_node = YAML::LoadFile(yaml_file);
        std::map<std::string, Pose> poses_map = { {"supply_pose", {}} , {"born_pose", {}}};

        // 遍历位姿信息的键 自动按首字母顺序读取
        for (auto it = poses_map.begin(); it != poses_map.end(); ++it)
        {
            // it++;
            const std::string& pose_key = it->first;
            Pose& temp_pose = it->second;
            // 获取位姿信息节点
            YAML::Node pose_node = yaml_node[pose_key];
            std::cout << pose_key <<std::endl;

            // 获取位姿信息的各个字段值，并存储到临时对象中
            temp_pose.x = pose_node["pose"]["position"]["x"].as<double>();
            temp_pose.y = pose_node["pose"]["position"]["y"].as<double>();
            temp_pose.z = pose_node["pose"]["position"]["z"].as<double>();
            temp_pose.roll = pose_node["pose"]["orientation"]["roll"].as<double>();
            temp_pose.pitch = pose_node["pose"]["orientation"]["pitch"].as<double>();
            temp_pose.yaw = pose_node["pose"]["orientation"]["yaw"].as<double>();



            std::string key_prefix = pose_key;
            std::string x_key = key_prefix + "_x";
            std::string y_key = key_prefix + "_y";
            std::string z_key = key_prefix + "_z";
            std::string roll_key = key_prefix + "_roll";
            std::string pitch_key = key_prefix + "_pitch";
            std::string yaw_key = key_prefix + "_yaw";

            geometry_msgs::msg::PoseStamped goal_pose_;
            geometry_msgs::msg::Point goal_position;
            geometry_msgs::msg::Quaternion geo_qnt;
            tf2::Quaternion tf_qnt;
            
            tf_qnt.setRPY(poses_map[pose_key].roll , poses_map[pose_key].pitch , poses_map[pose_key].yaw);
            goal_pose_.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
            goal_position.set__x(poses_map[pose_key].x).set__y(poses_map[pose_key].y).set__z(poses_map[pose_key].z);
            goal_pose_.pose.set__position(goal_position);
            goal_pose_.pose.set__orientation(convert<tf2::Quaternion, geometry_msgs::msg::Quaternion>(tf_qnt, geo_qnt, true));

        std::cout << "4444444444" <<std::endl;
        blackboard_->set<geometry_msgs::msg::PoseStamped>(key_prefix,goal_pose_);
        std::cout << "5555555555" <<std::endl;

            geometry_msgs::msg::PoseStamped retrieved_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>(key_prefix);
            std::cout << "pose.x=" << retrieved_pose.pose.position.x << "," 
                        << "pose.y=" << retrieved_pose.pose.position.y << "," 
                        << "pose.z=" << retrieved_pose.pose.position.z << "," 
                        << "pose.ori.x=" << retrieved_pose.pose.orientation.x << "," 
                        << "pose.ori.y=" << retrieved_pose.pose.orientation.y << "," 
                        << "pose.ori.z=" << retrieved_pose.pose.orientation.z << "," 
                        << "pose.ori.w=" << retrieved_pose.pose.orientation.w << std::endl;
    }
    return true;
    }

int main(int argc, char **argv)
{
    BT::BehaviorTreeFactory factory;
    rclcpp::init(argc, argv); // 初始化ROS2
    BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();

    std::string yaml_file = "src/decision/params/init_pose.yaml";
    rclcpp::NodeOptions options;
    auto my_node = std::make_shared<Node>(options);
    my_node->LoadParam(yaml_file,blackboard_);

    // BT::Tree tree = factory.createTreeFromFile("/home/hannah/BTLibraryBased_RobotDecision/src/decision/decision_behavior_tree/behavior_tree.xml");

    // tree.tickWhileRunning();

    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}