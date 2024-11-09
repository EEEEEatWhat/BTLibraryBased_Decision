#include "rm_behavior_tree/plugins/action/nav2pose.hpp" 

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::Nav2Pose>("Nav2Pose");
}