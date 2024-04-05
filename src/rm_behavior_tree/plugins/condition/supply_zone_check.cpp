#include "rm_behavior_tree/plugins/condition/supply_zone_check.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::SupplyZoneCheck>("SupplyZoneCheck");
}