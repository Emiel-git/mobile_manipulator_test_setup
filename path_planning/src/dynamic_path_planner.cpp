#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "moveit/move_group_interface/move_group_interface.h"
using std::placeholders::_1;



class PathPlanning : public rclcpp::Node
{
  public:
    PathPlanning()
    : Node("path_planner")
    {
        // create target position subsriber
        target_position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_position", 10, std::bind(&PathPlanning::target_position_callback, this, _1));
    }

  private:
    void target_position_callback(const geometry_msgs::msg::PoseStamped & msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_subscription_;
};



void plan_exec_path(geometry_msgs::msg::Pose picking_pose)
{
    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(std::make_shared<PathPlanning>(), "ur_manipulator");
    
    //Set speed and acceleration
    move_group_interface.setMaxVelocityScalingFactor(1.0);
    move_group_interface.setMaxAccelerationScalingFactor(1.0);

    // set target pose
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
    RCLCPP_ERROR(PathPlanning().get_logger(), "Planing failed!");
    };     
}; 

void PathPlanning::target_position_callback(const geometry_msgs::msg::PoseStamped & msg)
    {
        plan_exec_path(msg.pose);
    }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}