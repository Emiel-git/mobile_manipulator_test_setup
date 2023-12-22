#include <memory>
#include <chrono>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "mobile_manipulator_interfaces/srv/set_target_position.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "path_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("path_planning");

  // Create target position client
  rclcpp::Client<mobile_manipulator_interfaces::srv::SetTargetPosition>::SharedPtr
    client = node->create_client<mobile_manipulator_interfaces::srv::SetTargetPosition>("/request_target_position"); 
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

 // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  auto result = client->create_response();
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"succes");
    // auto const target_pose = []{
    //   mobile_manipulator_interfaces::msg::Pose target_pose;
    //   target_pose.position.x = result.get()-> target_position.position.x;
    //   target_pose.position.y = result.get()-> target_position.position.y;
    //   target_pose.position.z = result.get()-> target_position.position.z;
    //   target_pose.orientation.x = result.get()-> target_position.orientation.x;
    //   target_pose.orientation.y = result.get()-> target_position.orientation.y;
    //   target_pose.orientation.z = result.get()-> target_position.orientation.z;
    //   target_pose.orientation.w = result.get()-> target_position.orientation.w;
    // };
    // return target_pose;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set target position");
    rclcpp::shutdown();
  }

  // move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}