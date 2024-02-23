#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

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
        RCLCPP_INFO(this->get_logger(),"Path_planner node created");
    }

  private:
    void target_position_callback(const geometry_msgs::msg::PoseStamped & msg);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_subscription_;
};


class Planning : public moveit::planning_interface::MoveGroupInterface
{
    public:
        Planning()
        : MoveGroupInterface(std::make_shared<rclcpp::Node>("path_planner",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
            ), "ur_manipulator")
        {
            this->setMaxVelocityScalingFactor(1.0);
            this->setMaxAccelerationScalingFactor(1.0);
            this->setWorkspace(-1.5,-1.5,0,1.5,1.5,1.5);
        }
};


void plan_exec_path(geometry_msgs::msg::Pose target_pose)
{
    // create move_group_interface
    Planning move_group_interface;
    PathPlanning path_planner;

    move_group_interface.setPoseTarget(target_pose);

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
    RCLCPP_WARN(path_planner.get_logger(), "Planing failed!");
    };     
}; 


void collision_object()
{
    Planning move_group_interface;
    // Create collision object for the robot to avoid
    auto const collision_object = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "ground";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 10;
    primitive.dimensions[primitive.BOX_Y] = 10;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.01;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
    }();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}


void PathPlanning::target_position_callback(const geometry_msgs::msg::PoseStamped & msg)
    {   
        collision_object();
        plan_exec_path(msg.pose);
    }



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanning>());
    rclcpp::shutdown();
    return 0;
}