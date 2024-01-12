cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(path_planner_node src/static_path_planner.cpp)
ament_target_dependencies(
  path_planner_node 
  rclcpp 
  geometry_msgs 
  moveit_ros_planning_interface)

add_executable(dynamic_path_planner_node src/dynamic_path_planner.cpp)
ament_target_dependencies(
  dynamic_path_planner_node 
  rclcpp 
  geometry_msgs 
  moveit_ros_planning_interface) 

install(TARGETS
  path_planner_node
  dynamic_path_planner_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
