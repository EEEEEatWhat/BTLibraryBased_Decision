#include "bt_test/gain_blood_or_bullet.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

#include <memory>

int main(int argc, char **argv)
{
    BT::BehaviorTreeFactory factory;
    rclcpp::init(argc, argv); // 初始化ROS2
    auto node = std::make_shared<rclcpp::Node>("gain_blood_or_bullet_node");
    BT::RosNodeParams params;
    
    params.nh = node;
    params.default_port_value = "go_service";

    //注册节点
    factory.registerNodeType<decision_behavior_tree::GainBloodOrBulletAction>("gain_blood_or_bullet", params);


    BT::Tree tree = factory.createTreeFromFile("/home/hannah/ws_bt/src/bt_test/src/tree.xml");

    tree.tickWhileRunning();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
