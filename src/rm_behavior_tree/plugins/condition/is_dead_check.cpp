#include "rm_behavior_tree/plugins/condition/is_dead_check.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IsDeadCheck>("IsDeadCheck");
}