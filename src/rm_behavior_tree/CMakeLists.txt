cmake_minimum_required(VERSION 3.8)
project(rm_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(global_interfaces REQUIRED)
find_package(my_msg_interface REQUIRED)
find_package(auto_aim_interfaces REQUIRED)

set(THIS_PACKAGE_DEPS
rclcpp
rclcpp_action
behaviortree_cpp
behaviortree_ros2
nav2_msgs
tf2_ros
tf2
tf2_geometry_msgs
geometry_msgs
global_interfaces
my_msg_interface
auto_aim_interfaces
)

include_directories(
  include
)

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()


add_library(fire_or_skip SHARED 
plugins/action/fire_or_skip.cpp
)
list(APPEND plugin_libs fire_or_skip)
add_library(send_goal SHARED 
plugins/action/send_goal.cpp
)
list(APPEND plugin_libs send_goal)
add_library(send_sentrycmd SHARED 
plugins/action/send_sentrycmd.cpp
)
list(APPEND plugin_libs send_sentrycmd)
add_library(set_own_status SHARED 
plugins/action/set_own_status.cpp
)
list(APPEND plugin_libs set_own_status)

add_library(game_status_check SHARED 
plugins/condition/game_status_check.cpp
)
list(APPEND plugin_libs game_status_check)
add_library(hp_check SHARED 
plugins/condition/hp_check.cpp
)
list(APPEND plugin_libs hp_check)
add_library(supply_zone_check SHARED 
plugins/condition/supply_zone_check.cpp
)
list(APPEND plugin_libs supply_zone_check)
add_library(check_armors SHARED 
plugins/condition/check_armors.cpp
)
list(APPEND plugin_libs check_armors)
add_library(is_dead_check SHARED 
plugins/condition/is_dead_check.cpp
)
list(APPEND plugin_libs is_dead_check)
add_library(is_arrived SHARED 
plugins/condition/is_arrived.cpp
)
list(APPEND plugin_libs is_arrived)
add_library(reborn_now SHARED 
plugins/condition/reborn_now.cpp
)
list(APPEND plugin_libs reborn_now)
add_library(set_enemy_goal SHARED 
plugins/condition/set_enemy_goal.cpp
)
list(APPEND plugin_libs set_enemy_goal)

add_executable(rm_behavior_tree src/rm_behavior_tree.cpp)
ament_target_dependencies(rm_behavior_tree
  ${THIS_PACKAGE_DEPS}
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${THIS_PACKAGE_DEPS})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(rm_behavior_tree ${bt_plugin})
endforeach()

install(TARGETS
  rm_behavior_tree
  fire_or_skip
  send_goal
  send_sentrycmd
  set_own_status
  game_status_check
  hp_check
  supply_zone_check
  check_armors
  is_dead_check
  is_arrived
  reborn_now
  set_enemy_goal
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()