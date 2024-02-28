#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <tf2_ros/transform_listener.h>

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
            this->setPoseReferenceFrame("world");
            this->create_ground_object();
        }

        void cartesian_plan_exec_path(rclcpp::Logger logger){
            const std::vector<double>& joint_positions = this->getCurrentJointValues();
            for (size_t i = 0; i < joint_positions.size(); ++i)
            {
                RCLCPP_INFO(logger,"Joint %zu: %f", i, joint_positions[i]);
            }
            std::vector<geometry_msgs::msg::Pose> waypoints;
            double eef_step = 0.01;
            double jump_threshold = 0.0;
            // start_pose = this->getCurrentPose("tool0").pose;
            RCLCPP_INFO(logger,"Current Pose: %f, %f, %f", start_pose.position.x, start_pose.position.y, start_pose.position.z);

            waypoints.push_back(this->start_pose);
            waypoints.push_back(this->pose_target);
            RCLCPP_INFO(logger,"TARGET POSE = %f",waypoints[1].position.x);

            moveit_msgs::msg::RobotTrajectory trajectory;
            double fraction = this->computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
            RCLCPP_INFO(logger,"Cartesian path planning fraction: %.2f", fraction);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            this->execute(plan);
            // rclcpp::sleep_for(10s);
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
        geometry_msgs::msg::Pose start_pose;
        geometry_msgs::msg::Pose pose_target;
        float base_x;

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
        tool0_tf_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/tool0_pose", 10, std::bind(&PathPlanningNode::tool0_tf_callback, this, _1));
        
        RCLCPP_INFO(this->get_logger(),"Path_planner node created");
    }

  private:
    Planning move_group_interface;
    void target_position_callback(const geometry_msgs::msg::PoseStamped & msg){
        move_group_interface.pose_target = msg.pose;
        move_group_interface.pose_target.position.x -= move_group_interface.base_x;
        move_group_interface.cartesian_plan_exec_path(this->get_logger());
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_subscription_;

    void collision_objects_callback(const moveit_msgs::msg::CollisionObject & msg){
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj = msg;
        collision_obj.header.frame_id = move_group_interface.getPlanningFrame();
        move_group_interface.planning_scene_interface.applyCollisionObject(collision_obj);
    }
    rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collision_objects_subscription_;

    void tool0_tf_callback(const std_msgs::msg::Float32MultiArray & msg){
        move_group_interface.base_x = msg.data[0];
        move_group_interface.start_pose.position.x = msg.data[1];
        move_group_interface.start_pose.position.y = msg.data[2];
        move_group_interface.start_pose.position.z = msg.data[3];
        move_group_interface.start_pose.orientation.x = msg.data[4];
        move_group_interface.start_pose.orientation.y = msg.data[5];
        move_group_interface.start_pose.orientation.z = msg.data[6];
        move_group_interface.start_pose.orientation.w = msg.data[7];
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tool0_tf_subscription_;

};

int main(int argc, char * argv[])
{
    rclcpp::sleep_for(5s);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanningNode>());
    rclcpp::shutdown();
    return 0;
}