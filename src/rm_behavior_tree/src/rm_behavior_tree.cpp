#include "rm_behavior_tree/rm_behavior_tree.hpp"
#include <thread>
int main(int argc, char const **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto rm_behavior_tree = std::make_shared<rm_behavior_tree::RMBehaviorTree>(options);
    rclcpp::spin(rm_behavior_tree);
    rclcpp::shutdown();
    return 0;
}
