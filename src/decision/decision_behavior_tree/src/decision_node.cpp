#include "decision_node.hpp"

namespace decision_behavior_tree
{
    DecisionNode::DecisionNode(const rclcpp::NodeOptions &options)
        :rclcpp::Node("decision_node",options) , options_(options)
    {
        RCLCPP_INFO(this->get_logger() , "Decision node...");
        blackboard_ = BT::Blackboard::create();

        
        RCLCPP_INFO(this->get_logger() , "starting...");
        if (!this->decodeConfig(yaml_file_path,blackboard_))
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to get Config!");
            abort();
        }

        this->init();
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

    bool DecisionNode::decodeConfig(std::string yaml_file,BT::Blackboard::Ptr blackboard_)
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

        YAML::Node yaml_node = YAML::LoadFile(yaml_file);
        std::map<std::string, Pose> poses_map = { {"supply_pose", {}} , {"born_pose", {}}};
        
        // 遍历位姿信息的键 自动按键的首字母顺序读取
        for (auto it = poses_map.begin(); it != poses_map.end(); ++it)
        {
            const std::string& pose_key = it->first;
            Pose& temp_pose = it->second;
            // 获取位姿信息节点
            YAML::Node pose_node = yaml_node[pose_key];
            

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

            blackboard_->set<geometry_msgs::msg::PoseStamped>(key_prefix,goal_pose_);

            /*
            测测写进去了没
            geometry_msgs::msg::PoseStamped retrieved_pose = blackboard_->get<geometry_msgs::msg::PoseStamped>(key_prefix);
            std::cout << "pose.x=" << retrieved_pose.pose.position.x << "," 
                        << "pose.y=" << retrieved_pose.pose.position.y << "," 
                        << "pose.z=" << retrieved_pose.pose.position.z << "," 
                        << "pose.ori.x=" << retrieved_pose.pose.orientation.x << "," 
                        << "pose.ori.y=" << retrieved_pose.pose.orientation.y << "," 
                        << "pose.ori.z=" << retrieved_pose.pose.orientation.z << "," 
                        << "pose.ori.w=" << retrieved_pose.pose.orientation.w << std::endl;
            */
        }
        return true;
    }

    // 测试用
    static const char* xml_text = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence>
            <GainBloodOrBulletAction supply_pose = "{supply_pose}"/>
        </Sequence>
    </BehaviorTree>
</root>
)";

    void DecisionNode::init()
    {
        // tree_ = factory_.createTreeFromFile(xml_file_path,blackboard_);
        factory.registerNodeType<>(node_name, params);
        params.default_port_value = "go_server_action";
        factory_.registerNodeType<decision_behavior_tree::GainBloodOrBulletAction>("GainBloodOrBulletAction", params);

        auto tree = factory_.createTreeFromText(xml_text,blackboard_);
        
        while (rclcpp::ok())
        {
            tree.tickOnce();
        };
    }


}

int main(int argc, char const **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    auto decision_node = std::make_shared<decision_behavior_tree::DecisionNode>(options);

    rclcpp::spin(decision_node);
    rclcpp::shutdown();
    return 0;
}
