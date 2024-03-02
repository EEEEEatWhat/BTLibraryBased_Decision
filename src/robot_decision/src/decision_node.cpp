#include "decision_node.hpp"

namespace robot_decision
{
    DecisionNode::DecisionNode(const rclcpp::NodeOptions &options)
        :rclcpp::Node("decision_node",options) , options_(options)
    {
        RCLCPP_INFO(this->get_logger() , "Decision node...");

        blackboard_ = BT::Blackboard::create();
        
        RCLCPP_INFO(this->get_logger() , "starting...");
        if (!this->Decode_config_pose(blackboard_))
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

    bool DecisionNode::Decode_config_pose(BT::Blackboard::Ptr blackboard_)
    {
        // Variables defined in decision.yaml
        // goal_pose:
        std::map<std::string, geometry_msgs::msg::PoseStamped> poses_map = { {"enemy_bunker_pose", {}} , {"enemy_startup_pose", {}} , {"our_bunker_pose", {}}, {"patrol_1", {}}, {"patrol_2", {}}, {"patrol_3",{}} };

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

        
        // decision:
        uint16_t lowest_HP;
        double tuoluo_angular_vel;
        double gimbal_angular_vel;
        this->declare_parameter<uint16_t>("lowest_HP", 0);
        this->declare_parameter<double>("tuoluo_angular_vel", 0.0);
        this->declare_parameter<double>("gimbal_angular_vel", 0.0);
        this->get_parameter<uint16_t>("lowest_HP", lowest_HP);
        this->get_parameter<double>("tuoluo_angular_vel", tuoluo_angular_vel);
        this->get_parameter<double>("tuoluo_angular_vel", gimbal_angular_vel);
        blackboard_->set<uint16_t>("lowest_HP", lowest_HP);
        blackboard_->set<double>("tuoluo_angular_vel", tuoluo_angular_vel);
        blackboard_->set<double>("tuoluo_angular_vel", gimbal_angular_vel);

        // Variables from referee system or Sensor set in blackboard

        // 0x0001
        // 当前阶段剩余时间（秒）
        // 0x0003
        // 前哨站血量
        // 0x0101
        // 己方补给站前补血点的占领状态（1为已占领）
        // 己方补给站内部补血点的占领状态（1为已占领）
        // 己方2号环形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 己方3号梯形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 己方4号梯形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 0x0102
        // 补弹机器人信息
        // 0x0201
        // 本机器人当前血量
        // 0x0203
        // 本机器人位置x 坐标（米）
        // 本机器人位置y 坐标（米）
        // 本机器人测速模块朝向，单位：度。正北为0 度（待定）
        // 0x0204
        // 机器人回血增益
        // 0x0208
        // 剩余金币数量（待定）
        //0x0209
        // 己方哨兵巡逻区 // 梯高环高等增益点待定
        // 0x020D
        // 哨兵已经成功兑换的发弹量值（待定）
        // 0x120
        // 哨兵机器人是否确认复活
        // 哨兵机器人是否想要兑换立即复活
        // 哨兵想要兑换的发弹量值（开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量）
        // 哨兵想要远程兑换发弹量的请求次数（开局为0，修改此值即可请求远程兑换发弹量）
        // 哨兵想要远程兑换血量的请求次数（开局为0，修改此值即可请求远程兑换血量）
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.chassis_voltage", 0);
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.chassis_current", 0);
                // blackboard_->set<float>("PowerHeatDataStruct.chassis_power", 0);
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.buffer_energy", 0);
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_17mm_1_barrel_heat", 0);
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_17mm_2_barrel_heat", 0);
                // blackboard_->set<uint16_t>("PowerHeatDataStruct.shooter_42mm_barrel_heat", 0);
        // 自瞄flag
        blackboard_->set<uint8_t>("if_find_enemy", 0);
        blackboard_->set<std::string>("mode","0");
        return true;
    };

    void DecisionNode::RigisteredMapSolver()
    {

    };


//     // 测试用
    static const char* xml_text = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ExtSupplyProjectileActionCheck/>
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
                <Patrol_3/>
                <Patrol_5/>
                </Sequence>
            </Repeat>
            <Repeat num_cycles="2">
                <Sequence>
                <PatrolToSupply/>
                <Patrol_3/>
                <Patrol_4/>
                <Patrol_5/>
                </Sequence>
            </Repeat>
            <Repeat num_cycles="1000">
                <SetTuoluoStatus/>
            </Repeat>
        </Sequence>
    </BehaviorTree>
</root>
)";
    static const char* xml_text_2 = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence>
        <CheckMode/>
        <Switch3 case_1="1"
                case_2="2"
                case_3="3"
                variable="{mode}">
            <Repeat num_cycles="2">
                <SetTuoluoStatus/>
            </Repeat>
            <Repeat num_cycles="3">
                <Sequence>
                    <Patrol_1/>
                    <Patrol_2/>
                </Sequence>
            </Repeat>
            <Sensor/>
            <Stay/>
        </Switch3>
        </Sequence>
    </BehaviorTree>
</root>
)";

    void DecisionNode::Init()
    {
        auto node = std::make_shared<rclcpp::Node>("rclcpp_node");
        blackboard_->set<rclcpp::Node::SharedPtr>("node",node);
        Condition condition(blackboard_);
        actionParams.nh = node;
        actionParams.default_port_value = "BehaviorTreePose"; // name of the action server
        topicParams.nh = node;
        topicParams.default_port_value = "cmd_vel"; // name of the topic to publish
        topicParams2.nh = node;
        topicParams2.default_port_value = "control_id";
        actionParamsForPoses.nh = node;
        actionParamsForPoses.default_port_value = "BehaviorTreeSendPoses";
        factory_.registerSimpleCondition("GameStarted", [&](BT::TreeNode&) { return condition.Check_game_started(); });
        factory_.registerSimpleCondition("HPCheck", [&](BT::TreeNode&) { return condition.Check_blood(); });
        factory_.registerSimpleCondition("IfFindEnemy", [&](BT::TreeNode&) { return condition.Check_enemy(); });
        factory_.registerSimpleCondition("SetStrategy", [&](BT::TreeNode&) { return condition.Set_strategy(); });
        factory_.registerSimpleCondition("Stay", [&](BT::TreeNode&) { return condition.Stay(); });
        
        factory_.registerSimpleCondition("Sensor", [&](BT::TreeNode&) { return condition.Sensor(); });


        factory_.registerSimpleCondition("ExtSupplyProjectileActionCheck", [&](BT::TreeNode&) { return condition.Check_ext_supply_projectile(); });
        factory_.registerSimpleCondition("PowerHeatCheck", [&](BT::TreeNode&) { return condition.Check_power_heat(); });


        factory_.registerNodeType<robot_decision::GainBloodAction>("GainBlood");
        factory_.registerNodeType<robot_decision::GoEnemyBunker>("GoEnemyBunker", actionParams);
        factory_.registerNodeType<robot_decision::GoEnemyStartup>("GoEnemyStartup", actionParams);
        factory_.registerNodeType<robot_decision::GoOurBunker>("GoOurBunker", actionParams);
        factory_.registerNodeType<robot_decision::PatrolToSupplyAction>("PatrolToSupply", actionParams);
        factory_.registerNodeType<robot_decision::SetTuoluoStatus>("SetTuoluoStatus", topicParams);
        factory_.registerNodeType<robot_decision::CheckMode>("CheckMode", topicParams2);
        factory_.registerNodeType<robot_decision::Wandering>("Wandering",actionParamsForPoses);
        // Action: SearchEnemy,

        factory_.registerNodeType<robot_decision::Patrol_1>("Patrol_1", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_2>("Patrol_2", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_3>("Patrol_3", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_4>("Patrol_4", actionParams);
        factory_.registerNodeType<robot_decision::Patrol_5>("Patrol_5", actionParams);
        // factory_.registerBehaviorTreeFromFile(xml_file_path);
        // tree_ = factory_.createTree("mainTree",blackboard_);
        
        tree_ = factory_.createTreeFromText(xml_text,blackboard_);
        sleep(1);
        while (rclcpp::ok())
        {
            tree_.tickWhileRunning();
        };
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
