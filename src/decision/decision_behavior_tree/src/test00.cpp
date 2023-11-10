#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char **argv)
{
    BT::BehaviorTreeFactory factory;
    rclcpp::init(argc, argv); // 初始化ROS2

    // BT::Tree tree = factory.createTreeFromFile("/home/hannah/BTLibraryBased_RobotDecision/src/decision/decision_behavior_tree/behavior_tree.xml");

    // tree.tickWhileRunning();

    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}