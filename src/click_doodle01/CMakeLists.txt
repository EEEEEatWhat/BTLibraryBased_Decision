cmake_minimum_required(VERSION 3.8)
project(click_doodle01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(global_interfaces REQUIRED)

add_executable(decision src/decision.cpp)
ament_target_dependencies(
  decision
  "rclcpp"
  "std_msgs"
  behaviortree_cpp
)
add_executable(dark_knight src/clickdoodle02.cpp)
ament_target_dependencies(
  dark_knight
  "rclcpp"
  "std_msgs"
  behaviortree_cpp
  base_interfaces_demo
  behaviortree_ros2
  nav2_msgs
  tf2
  tf2_geometry_msgs
  global_interfaces
)

install(TARGETS 
  decision
  dark_knight
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(decision
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/click_doodle01/
)
target_include_directories(dark_knight
  PRIVATE
    include
    ${PROJECT_SOURCE_DIR}/include/click_doodle01/
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
