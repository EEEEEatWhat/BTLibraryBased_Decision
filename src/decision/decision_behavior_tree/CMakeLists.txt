cmake_minimum_required(VERSION 3.8)
project(decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_path(yaml-cpp REQUIRED PATHS /home/hannah/BTLibraryBased_Decision/src/lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(global_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
# find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

set(THIS_PACKAGE_DEPS
rclcpp
behaviortree_cpp
std_msgs
nav2_msgs
behaviortree_ros2
global_interfaces
rclcpp_action
tf2
tf2_geometry_msgs
# yaml-cpp
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

# target_include_directories(${PROJECT_NAME} PRIVATE ${YAML_CPP_INCLUDE_DIR}) 


add_executable(decision_node src/decision_node.cpp)
# add_executable(manage
# include/manage/yamlsetup.h
# )
ament_target_dependencies(decision_node
  ${THIS_PACKAGE_DEPS}
  # ${YAMLCPP_LIBRARIES}
)
target_link_libraries(decision_node
  ${YAMLCPP_LIBRARIES}
)

install(TARGETS
  decision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
