#include "decision_node.hpp"

namespace robot_decision
{
    DecisionNode::DecisionNode(const rclcpp::NodeOptions &options)
        :rclcpp::Node("decision_node",options) , options_(options)
    {
        RCLCPP_INFO(this->get_logger() , "Decision node...");

        blackboard_ = BT::Blackboard::create();
        
        RCLCPP_INFO(this->get_logger() , "starting...");
        if (!this->Decode_config_pose(init_pose_path, blackboard_))
        {
            RCLCPP_ERROR(this->get_logger() , "Failed to get Config!");
            abort();
        }
        this->RigisteredMapSolver();

        this->Init();
        RCLCPP_INFO(this->get_logger() , "init finished...");
    }

    DecisionNode::~DecisionNode()
    {}

    template <class A, class B>
    B DecisionNode::convert(const A & a, B & b,bool c){
        (void)c;
        tf2::impl::Converter<rosidl_generator_traits::is_message<A>::value,
            rosidl_generator_traits::is_message<B>::value>::convert(a, b);
        return b;
    };

    bool DecisionNode::Decode_config_pose(const std::string init_pose_path, BT::Blackboard::Ptr blackboard_)
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

        YAML::Node yaml_node = YAML::LoadFile(init_pose_path);
        std::map<std::string, Pose> poses_map = { {"supply_pose", {}} , {"born_pose", {}} , {"patrol_pose", {}} };
        
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
        }

        // Variables defined in condition.yaml
        uint16_t lowest_HP;
        float tuoluo_status;
        this->declare_parameter<uint16_t>("lowest_HP", 100);
        this->declare_parameter<float>("tuoluo_status", 0.f);
        this->get_parameter<uint16_t>("lowest_HP", lowest_HP);
        this->get_parameter<float>("tuoluo_status", tuoluo_status);
        blackboard_->set<uint16_t>("lowest_HP", lowest_HP);
        blackboard_->set<float>("tuoluo_status", tuoluo_status);

        // Variables from referee system or Sensor set in blackboard
        
        blackboard_->set<uint8_t>("if_find_enemy", 0);

        return true;
    };

    void DecisionNode::RigisteredMapSolver()
    {

    };

    // void DecisionNode::listenInput()
    // {
    //     int input; // 0 patrol 1 tuoluo
    //     while (!stop) {
    //     std::getline(std::cin, input);
    //     std::cout << "You entered: " << input << std::endl;
    //     if(input)
    //     {
    //     }
    // };

//     // 测试用
    static const char* xml_text = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Patrol_1/>
            <Patrol_2/>
        </Sequence>
    </BehaviorTree>
</root>
)";

    static const char* xml_text_1 = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Repeat num_cycles="2">
                <Sequence>
                <Patrol_1/>
                <Patrol_2/>
                </Sequence>
            </Repeat>
            <Repeat num_cycles="1000">
                <SetTuoluoStatus/>
            </Repeat>
        </Sequence>
    </BehaviorTree>
</root>
)";

    void DecisionNode::Init()
    {
        auto node = std::make_shared<rclcpp::Node>("rclcpp_node");
        blackboard_->set<rclcpp::Node::SharedPtr>("node",node);
        Condition robocondition(blackboard_);
        actionParams.nh = node;
        actionParams.default_port_value = "BehaviorTreePose"; // name of the action server
        topicParams.nh = node;
        topicParams.default_port_value = "cmd_vel"; // name of the topic to publish

        factory_.registerSimpleCondition("IfFindEnemy", [&](BT::TreeNode&) { return robocondition.Check_enemy(); });
        factory_.registerSimpleCondition("IfNeedSupply", [&](BT::TreeNode&) { return robocondition.Check_blood(); });
        factory_.registerSimpleCondition("CheckGameStatus", [&](BT::TreeNode&) { return robocondition.Check_game_status(); });

        factory_.registerNodeType<robot_decision::PatrolToSupplyAction>("PatrolToSupply", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_1>("Patrol_1", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_2>("Patrol_2", actionParams);
        factory_.registerNodeType<robot_decision::SetTuoluoStatus>("SetTuoluoStatus", topicParams);
        factory_.registerNodeType<robot_decision::GainBloodAction>("GainBlood");
        factory_.registerNodeType<robot_decision::HappyPatrolAction>("HappyPatrol", actionParams);

        // factory_.registerBehaviorTreeFromFile(xml_file_path);
        // tree_ = factory_.createTree("mainTree",blackboard_);
        
        tree_ = factory_.createTreeFromText(xml_text_1,blackboard_);
        
        // while (rclcpp::ok())
        // {

            tree_.tickWhileRunning();
        // };
    }

    
}


int main(int argc, char const **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    auto decision_node = std::make_shared<robot_decision::DecisionNode>(options);
    rclcpp::spin(decision_node);
    rclcpp::shutdown();
    return 0;
}
