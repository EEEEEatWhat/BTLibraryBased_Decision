cmake_minimum_required(VERSION 3.8)
project(goal_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(global_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(goalActionServer src/GoalActionServer.cpp)
add_executable(goalActionServerWithPoses src/GoalActionServer_poses.cpp)
ament_target_dependencies(
  goalActionServer
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  rclcpp_action
  global_interfaces
  nav2_msgs
)
ament_target_dependencies(
  goalActionServerWithPoses
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  rclcpp_action
  global_interfaces
  nav2_msgs
)

add_executable(client src/test_client.cpp)
ament_target_dependencies(
  client
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  rclcpp_action
  global_interfaces
)

install(TARGETS goalActionServer goalActionServerWithPoses client
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
