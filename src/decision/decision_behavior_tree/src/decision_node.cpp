#if WIN32
    #define YAML_CPP_STATIC_DEFINE
#endif
#include "decision_node.hpp"

namespace decision_behavior_tree
{
    DecisionNode::DecisionNode(const rclcpp::NodeOptions &options)
        :rclcpp::Node("decision_node",options)
    {
        RCLCPP_INFO(this->get_logger() , "Decision node...");
        RCLCPP_INFO(this->get_logger() , "starting...");
        if (!this->decodeConfig())
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to get Config!");
            abort();
        }
        this->init();
        // assert(CARPOS_NUM == this->type_id.size() / 2);
    }

    DecisionNode::~DecisionNode()
    {
    }

    template <class A, class B>
    B DecisionNode::convert(const A & a, B & b,bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    };

    bool DecisionNode::decodeConfig()
    {
        struct Pose
        {
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
        };

        std::string yaml_file = "src/decision/params/init_pose.yaml";
        YAML::Node yaml_node = YAML::LoadFile(yaml_file);
        std::map<std::string, Pose> poses_map = { {"supply_pose", {}} , {"born_pose", {}}};

        // 遍历位姿信息的键
        for (const auto &pose_key : poses_map)
        {
        std::cout << "11111111" <<std::endl;

            // 获取位姿信息节点
            YAML::Node pose_node = yaml_node[pose_key.first];
        std::cout << "222222222" <<std::endl;

            // 获取位姿信息的各个字段值
            poses_map[pose_key.first].x = pose_node["pose"]["position"]["x"].as<double>();
            poses_map[pose_key.first].y = pose_node["pose"]["position"]["y"].as<double>();
            poses_map[pose_key.first].z = pose_node["pose"]["position"]["z"].as<double>();
            poses_map[pose_key.first].roll = pose_node["pose"]["orientation"]["roll"].as<double>();
            poses_map[pose_key.first].pitch = pose_node["pose"]["orientation"]["pitch"].as<double>();
            poses_map[pose_key.first].yaw = pose_node["pose"]["orientation"]["yaw"].as<double>();
        std::cout << "3333333333" <<std::endl;
        std::cout << poses_map[pose_key.first].x <<std::endl;

            std::string key_prefix = "pose_" + pose_key.first;
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
            
            tf_qnt.setRPY(poses_map[pose_key.first].roll , poses_map[pose_key.first].pitch , poses_map[pose_key.first].yaw);
            goal_pose_.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
            goal_position.set__x(poses_map[pose_key.first].x).set__y(poses_map[pose_key.first].y).set__z(poses_map[pose_key.first].z);
            goal_pose_.pose.set__position(goal_position);
            goal_pose_.pose.set__orientation(convert<tf2::Quaternion, geometry_msgs::msg::Quaternion>(tf_qnt, geo_qnt, true));
        std::cout << "4444444444" <<std::endl;
            // blackboard_->set<int>("one",1);
        // std::cout << "4444444444" <<std::endl;
        blackboard.set<geometry_msgs::msg::PoseStamped>(key_prefix,goal_pose_);
        std::cout << "5555555555" <<std::endl;

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
    }

    

    void DecisionNode::init()
    {
        blackboard_ = BT::Blackboard::create();

        tree_ = factory_.createTreeFromFile("/home/hannah/BTLibraryBased_RobotDecision/src/decision/decision_behavior_tree/behavior_tree.xml");

        this->decodeConfig();
    }


}

int main(int argc, char const **argv)
{
    rclcpp::init(argc,argv);
    auto my_node = std::make_shared<decision_behavior_tree::DecisionNode>();
    // rclcpp::spin(my_node);
    my_node->init();
    rclcpp::shutdown();
    return 0;
}
