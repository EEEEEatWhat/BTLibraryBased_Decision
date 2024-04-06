#include "rm_behavior_tree/plugins/condition/set_enemy_goal.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SetEnemyGoal>("SetEnemyGoal");
}