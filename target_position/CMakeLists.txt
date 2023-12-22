cmake_minimum_required(VERSION 3.8)
project(target_position)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(mobile_manipulator_interfaces REQUIRED)

add_executable(target_server src/target_position.cpp)
ament_target_dependencies(
  target_server 
  rclcpp
  mobile_manipulator_interfaces)

install(TARGETS 
  target_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
