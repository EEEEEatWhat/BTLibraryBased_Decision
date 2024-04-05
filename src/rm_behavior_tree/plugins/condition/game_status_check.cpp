#include "rm_behavior_tree/plugins/condition/game_status_check.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::GameStatusCheck>("GameStatusCheck");
}