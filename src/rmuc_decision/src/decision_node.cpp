#include "decision_node.hpp"

namespace rmuc_decision
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
        this->Rigister_MapSolver();

        this->Init_behaviortree();
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
        std::map<std::string, geometry_msgs::msg::PoseStamped> poses_map = 
            { {"enemy_door", {}} , {"enemy_outpost", {}} , {"enemy_keystone_heights", {}} , {"our_keystone_heights", {}} , {"our_outpost", {}} };

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
            std::string z_key = pose_key + ".z";
            std::string yaw_key = pose_key + ".yaw";

            this->declare_parameter<double>(x_key, 0.0);
            this->declare_parameter<double>(y_key, 0.0);
            this->declare_parameter<double>(z_key, 0.0);
            this->declare_parameter<double>(yaw_key, 0.0);
            
            this->get_parameter<double>(x_key, temp_pose.pose.position.x);
            this->get_parameter<double>(y_key, temp_pose.pose.position.y);
            this->get_parameter<double>(y_key, temp_pose.pose.position.z);
            this->get_parameter<double>(yaw_key, temp_yaw);


            tf_qnt.setRPY(0, 0, temp_yaw);
            temp_pose.header.set__frame_id("map").set__stamp(rclcpp::Clock().now());
            temp_pose.pose.set__orientation(convert<tf2::Quaternion, geometry_msgs::msg::Quaternion>(tf_qnt, geo_qnt, true));
    
            blackboard_->set<geometry_msgs::msg::PoseStamped>(pose_key,temp_pose);
        }

        // decision:
        blackboard_->set<std::string>("my_color", "red"); 
        blackboard_->set<std::string>("openstage_strategy", "-1");
        blackboard_->set<std::string>("outpost_status", "ALL_ALIVE");
        blackboard_->set<std::string>("game_stage", "unready");

        uint16_t lowest_HP;
        double tuoluo_angular_vel;
        this->declare_parameter<uint16_t>("lowest_HP", 0);
        this->declare_parameter<double>("tuoluo_angular_vel", 0.0);
        this->get_parameter<uint16_t>("lowest_HP", lowest_HP);
        this->get_parameter<double>("tuoluo_angular_vel", tuoluo_angular_vel);
        blackboard_->set<uint16_t>("lowest_HP", lowest_HP);
        blackboard_->set<double>("tuoluo_angular_vel", tuoluo_angular_vel);

        // Variables from referee system or Sensor set in blackboard

        // 0x0001
        //      当前比赛阶段（4为比赛中） uint8_t game_progress : 4; 
        //      当前阶段剩余时间（秒）
        // 0x0003
        //      红方前哨站血量
        //      蓝方前哨站血量
        // 己方补给站前补血点的占领状态（1为已占领）
        // 己方补给站内部补血点的占领状态（1为已占领）
        // 己方2号环形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 己方3号梯形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 己方4号梯形高地的占领状态（1 为已被己方占领，2 为已被对方占领）
        // 本机器人当前血量
        // 本机器人位置x 坐标（米）
        // 本机器人位置y 坐标（米）
        // 本机器人测速模块朝向，单位：度。正北为0 度（待定）
        // 机器人回血增益
        // 剩余金币数量（待定）
        // 己方哨兵巡逻区 // 梯高环高等增益点待定
        // 哨兵已经成功兑换的发弹量值（待定）
        // 0x0120
        //      哨兵机器人是否确认复活
        //      哨兵机器人是否想要兑换立即复活
        //      哨兵想要兑换的发弹量值（开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量）
        //      哨兵想要远程兑换发弹量的请求次数（开局为0，修改此值即可请求远程兑换发弹量）
        //      哨兵想要远程兑换血量的请求次数（开局为0，修改此值即可请求远程兑换血量）


        // 自瞄flag
        blackboard_->set<uint8_t>("if_find_enemy", 0);

        return true;
    };

    void DecisionNode::Rigister_MapSolver()
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

    void DecisionNode::Init_behaviortree()
    {
        auto node = std::make_shared<rclcpp::Node>("rclcpp_node");
        blackboard_->set<rclcpp::Node::SharedPtr>("node",node);
        Condition condition(blackboard_);
        actionParams.nh = node;
        actionParams.default_port_value = "BehaviorTreePose"; // name of the action server
        topicParams.nh = node;
        topicParams.default_port_value = "cmd_vel"; // name of the topic to publish
        // register nodes of behaviortree
        
        factory_.registerSimpleCondition("GameLastStage", [&](BT::TreeNode&) { return condition.Check_is_set_game_status_as_last(); });
        factory_.registerSimpleCondition("GameRunning", [&](BT::TreeNode&) { return condition.Check_game_running(); });
        factory_.registerSimpleCondition("GameStarted", [&](BT::TreeNode&) { return condition.Check_game_started(); });
        factory_.registerSimpleCondition("HPCheck", [&](BT::TreeNode&) { return condition.Check_blood(); });
        factory_.registerSimpleCondition("IfFindEnemy", [&](BT::TreeNode&) { return condition.Check_enemy(); });
        factory_.registerSimpleCondition("IsDeadCheck", [&](BT::TreeNode&) { return condition.Check_is_dead(); });
        factory_.registerSimpleCondition("IfGoEnemyDoor", [&](BT::TreeNode&) { return condition.Check_if_go_enemy_door(); });
        factory_.registerSimpleCondition("IfGoEnemyOutpost", [&](BT::TreeNode&) { return condition.Check_game_started(); });
        factory_.registerSimpleCondition("IfGoKeystoneHeights", [&](BT::TreeNode&) { return condition.Check_if_go_keystone_heights(); });
        factory_.registerSimpleCondition("IfGoOurOutpost", [&](BT::TreeNode&) { return condition.Check_if_go_our_outpost(); });
        factory_.registerSimpleCondition("IsOpenStageStrategySet", [&](BT::TreeNode&) { return condition.Check_openstage_strategy(); });
        factory_.registerSimpleCondition("IsSetGameStatusAsLast", [&](BT::TreeNode&) { return condition.Check_is_set_game_status_as_last(); });
        factory_.registerSimpleCondition("NavStatus", [&](BT::TreeNode&) { return condition.Check_Nav_status(); });
        factory_.registerSimpleCondition("SupplyZoneCheck", [&](BT::TreeNode&) { return condition.Check_supply_zone(); });
        factory_.registerSimpleCondition("DefaultSkip", [&](BT::TreeNode&) { return condition.Default_skip(); });
        factory_.registerSimpleCondition("OurOutpostCheck", [&](BT::TreeNode&) { return condition.Check_our_outpost(); });

        
        // // Action:ExecuteCommand
        factory_.registerNodeType<rmuc_decision::GainBloodAction>("GainBlood");
        factory_.registerNodeType<rmuc_decision::GoEnemyDoor>("GoEnemyDoor", actionParams);
        factory_.registerNodeType<rmuc_decision::GoEnemyOutpost>("GoEnemyOutpost", actionParams);
        factory_.registerNodeType<rmuc_decision::GoKeystoneHeights>("GoKeystoneHeights", actionParams);
        factory_.registerNodeType<rmuc_decision::GoOurOutpost>("GoOurOutpost", actionParams);
        factory_.registerNodeType<rmuc_decision::PatrolToSupplyAction>("PatrolToSupply", actionParams);
        factory_.registerNodeType<rmuc_decision::SetTuoluoStatus>("SetTuoluoStatus", topicParams);
        factory_.registerNodeType<rmuc_decision::Wandering>("wandering", topicParams);

        // Action:Patrol, RebornNow, SearchEnemy


        // factory_.registerBehaviorTreeFromFile(xml_file_path);
        // tree_ = factory_.createTree("mainTree",blackboard_);
        
        tree_ = factory_.createTreeFromText(xml_text,blackboard_);
        
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
    auto decision_node = std::make_shared<rmuc_decision::DecisionNode>(options);
    rclcpp::spin(decision_node);
    rclcpp::shutdown();
    return 0;
}
