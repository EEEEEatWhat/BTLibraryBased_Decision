cmake_minimum_required(VERSION 3.8)
project(bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(global_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_DEPS
rclcpp
behaviortree_cpp
std_msgs
behaviortree_ros2
global_interfaces
rclcpp_action
nav2_msgs
tf2
tf2_geometry_msgs
)

include_directories(
  include
  src/lib
  src/global_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source ,.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(test02 src/test02.cpp)
add_executable(go_server src/go_server.cpp)
add_executable(go_pub src/go_pub.cpp)

ament_target_dependencies(test02 
  ${THIS_PACKAGE_DEPS}
)
ament_target_dependencies(go_server 
  ${THIS_PACKAGE_DEPS}
)
ament_target_dependencies(go_pub
  ${THIS_PACKAGE_DEPS}
)
install(TARGETS
  test02
  go_server
  go_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
