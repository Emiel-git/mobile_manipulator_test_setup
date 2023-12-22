cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mobile_manipulator_interfaces REQUIRED)
find_package(target_position_py)

add_executable(path_planning src/path_planning.cpp)
target_include_directories(path_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_planning PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  path_planning
  mobile_manipulator_interfaces
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS 
  path_planning
  DESTINATION lib/${PROJECT_NAME})

ament_package()
