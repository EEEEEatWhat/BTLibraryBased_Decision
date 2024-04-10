#include "rm_behavior_tree/plugins/condition/reset_res_data.hpp"

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::ResetResData>("ResetResData");
}