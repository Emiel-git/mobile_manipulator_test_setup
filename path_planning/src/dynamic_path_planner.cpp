#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

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
            this->create_ground_object();
        }

        void plan_exec_path(rclcpp::Logger logger){
            // Create a plan to that target pose
            auto const [successP, planP] = [&]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->plan(msg));
            return std::make_pair(ok, msg);
            }();

            // Execute the plan
            if(successP) {
            this->execute(planP);
            } else {
            RCLCPP_WARN(logger, "Planing failed!");
            };     
        }; 

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    private:
        void create_ground_object(){
            // Create ground collision object for the robot to avoid
            auto const collision_object = [frame_id = this->getPlanningFrame()] {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = frame_id;
            collision_object.id = "ground";
            shape_msgs::msg::SolidPrimitive primitive;

            // Define the size of the box in meters
            primitive.type = 1;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 10;
            primitive.dimensions[1] = 10;
            primitive.dimensions[2] = 0.01;

            // Define the pose of the box (relative to the frame_id)
            geometry_msgs::msg::Pose object_pose;
            object_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
            object_pose.position.x = 0.0;
            object_pose.position.y = 0.0;
            object_pose.position.z = -0.01;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(object_pose);
            collision_object.operation = collision_object.ADD;

            return collision_object;
            }();
	
            planning_scene_interface.applyCollisionObject(collision_object);
        }
};

class PathPlanningNode : public rclcpp::Node
{
  public:
    PathPlanningNode()
    : Node("path_planner")
    {
        // create target position subsriber
        target_position_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_position", 10, std::bind(&PathPlanningNode::target_position_callback, this, _1));
        collision_objects_subscription_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
            "/collision_objects", 10, std::bind(&PathPlanningNode::collision_objects_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(),"Path_planner node created");
    }

  private:
    Planning move_group_interface;
    void target_position_callback(const geometry_msgs::msg::PoseStamped & msg){
        move_group_interface.setPoseTarget(msg.pose);
        move_group_interface.plan_exec_path(this->get_logger());
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_subscription_;

    void collision_objects_callback(const moveit_msgs::msg::CollisionObject & msg){
        RCLCPP_INFO(this->get_logger(),"COLLISIONNNNNNNNNNNNNNNNNN");
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj = msg;
        collision_obj.header.frame_id = move_group_interface.getPlanningFrame();
        move_group_interface.planning_scene_interface.applyCollisionObject(collision_obj);
    }
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collision_objects_subscription_;
};

// void plan_exec_path(geometry_msgs::msg::Pose target_pose)
// {
//     // create move_group_interface
//     Planning move_group_interface;
//     PathPlanningNode path_planner;

//     move_group_interface.setPoseTarget(target_pose);
//     // Create a plan to that target pose
//     auto const [successP, planP] = [&move_group_interface]{
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//     }();

//     // Execute the plan
//     if(successP) {
//     move_group_interface.execute(planP);
//     } else {
//     RCLCPP_WARN(path_planner.get_logger(), "Planing failed!");
//     };     
// }; 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}