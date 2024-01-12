#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <time.h>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/pose.h"

using namespace std;
using std::placeholders::_1;

double tar_pos_X = 0.2;
double tar_pos_Y = 0.2;
double tar_pos_Z = 0.2;
double tar_pos_x = 0;
double tar_pos_y = 0;
double tar_pos_z = 0;
double tar_pos_w = 1;

void target_position_callback(const geometry_msgs::msg::Pose & msg)
{
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I heard x: '%f'", msg.orientation.x);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I heard y: '%f'", msg.orientation.y);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I heard z: '%f'", msg.orientation.z);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I heard w: '%f'", msg.orientation.w);

  tar_pos_X = msg.position.x;
  tar_pos_Y = msg.position.y;
  tar_pos_Z = msg.position.z;
  tar_pos_x = msg.orientation.x;
  tar_pos_y = msg.orientation.y;
  tar_pos_z = msg.orientation.z;
  tar_pos_w = msg.orientation.w;

  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I set x: '%f'", tar_pos_x);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I set y: '%f'", tar_pos_y);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I set z: '%f'", tar_pos_z);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), "I set w: '%f'", tar_pos_w);
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "path_planner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("path_planner");
  RCLCPP_INFO(logger,"path_planner node created");

  // Create subscriber for target position
  auto const target_subscription = rclcpp::create_subscription<geometry_msgs::msg::Pose>(node,"/target_position",10,target_position_callback);

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  //Set speed and acceleration
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);


  RCLCPP_INFO(rclcpp::get_logger("path_planner"), " x: '%f'", tar_pos_x);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), " y: '%f'", tar_pos_y);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), " z: '%f'", tar_pos_z);
  RCLCPP_INFO(rclcpp::get_logger("path_planner"), " w: '%f'", tar_pos_w);
  
  // Set a target Poses
  auto picking_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = tar_pos_w;
    msg.orientation.x = tar_pos_x;
    msg.orientation.y = tar_pos_y;
    msg.orientation.z = tar_pos_z;
    msg.position.x = tar_pos_X;
    msg.position.y = tar_pos_Y;
    msg.position.z = tar_pos_Z;
    return msg;
  }();

  //start picking
  move_group_interface.setPoseTarget(picking_pose);

  // Create a plan to that target pose
  auto const [successP, planP] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(successP) {
    move_group_interface.execute(planP);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  };

  // Shutdown ROS
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};