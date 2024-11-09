#include "rm_behavior_tree/plugins/action/chassis_spin.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::ChassisSpin>("ChassisSpin");
}