cmake_minimum_required(VERSION 3.10)
project(rm_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

set(PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_action
    nav2_msgs
    behaviortree_cpp
    behaviortree_ros2
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    global_interfaces
    my_msg_interface
    auto_aim_interfaces
)

# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
    target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
    ament_target_dependencies(${target} ${PACKAGE_INCLUDE_DEPENDS})
    target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)

# Conditions
ament_auto_add_library(game_status_check SHARED 
    plugins/condition/game_status_check.cpp
)
add_target_dependencies(game_status_check)
ament_auto_add_library(hp_check SHARED 
    plugins/condition/hp_check.cpp
)
add_target_dependencies(hp_check)
ament_auto_add_library(supply_zone_check SHARED 
    plugins/condition/supply_zone_check.cpp
)
add_target_dependencies(supply_zone_check)
ament_auto_add_library(check_armors SHARED 
    plugins/condition/check_armors.cpp
)
add_target_dependencies(check_armors)
ament_auto_add_library(is_dead_check SHARED 
    plugins/condition/is_dead_check.cpp
)
add_target_dependencies(is_dead_check)
ament_auto_add_library(is_arrived SHARED 
    plugins/condition/is_arrived.cpp
)
add_target_dependencies(is_arrived)
ament_auto_add_library(reborn_now SHARED 
    plugins/condition/reborn_now.cpp
)
add_target_dependencies(reborn_now)
ament_auto_add_library(set_enemy_goal SHARED 
    plugins/condition/set_enemy_goal.cpp
)
add_target_dependencies(set_enemy_goal)
ament_auto_add_library(reset_res_data SHARED 
    plugins/condition/reset_res_data.cpp
)
add_target_dependencies(reset_res_data)
ament_auto_add_library(our_outpost_check SHARED 
    plugins/condition/our_outpost_check.cpp
)
add_target_dependencies(our_outpost_check)
ament_auto_add_library(check_game_running SHARED 
    plugins/condition/check_game_running.cpp
)
add_target_dependencies(check_game_running)

# Actions
ament_auto_add_library(send_sentrycmd SHARED 
    plugins/action/send_sentrycmd.cpp
)
add_target_dependencies(send_sentrycmd)
ament_auto_add_library(send_goal SHARED 
    plugins/action/send_goal.cpp
)
add_target_dependencies(send_goal)
ament_auto_add_library(set_own_status SHARED 
    plugins/action/set_own_status.cpp
)
add_target_dependencies(set_own_status)
ament_auto_add_library(fire_or_skip SHARED 
    plugins/action/fire_or_skip.cpp
)
add_target_dependencies(fire_or_skip)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)