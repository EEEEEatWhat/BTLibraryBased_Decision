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

#include "go_pub.hpp"
#include "plugins/action/gain_blood_or_bullet_action.hpp"


static const char* xml_text = R"(

<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <GainBloodOrBulletAction supply_pose = "{supply_pose}" if_supply="{if_supply}"/>
            <GoPublisher supply_pose = "{supply_pose}" if_supply="{if_supply}" born_pose="{born_pose}" if_patrol="{if_patrol}"/>
        </Sequence>
    </BehaviorTree>
</root>
)";


    int main(int argc, char **argv)
    {
        BT::BehaviorTreeFactory factory;
        rclcpp::init(argc, argv); // 初始化ROS2
        BT::NodeConfig config;
        BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();
        factory.registerNodeType<decision_behavior_tree::GainBloodOrBulletAction>("GainBloodOrBulletAction");
        factory.registerNodeType<decision_behavior_tree::GoPublisher>("GoPublisher");
        auto tree = factory.createTreeFromText(xml_text,blackboard_);
        std::cout << "1111111111" << std::endl;

        while (rclcpp::ok())
        {
        std::cout << "2222222222" << std::endl;

            tree.tickOnce();
        };

        // config.blackboard = blackboard_;
        // factory.registerNodeType<decision_behavior_tree::GainBloodOrBulletAction>("gain_blood_or_bullet", config);

        // BT::Tree tree = factory.createTreeFromFile("/home/hannah/BTLibraryBased_RobotDecision/src/decision/decision_behavior_tree/behavior_tree.xml");

        // tree.tickWhileRunning();

        // rclcpp::spin(node);
        rclcpp::shutdown();

        return 0;
    }

