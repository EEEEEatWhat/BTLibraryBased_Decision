#include "rm_behavior_tree/plugins/condition/check_game_started.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::CheckGameStarted>("CheckGameStarted");
}