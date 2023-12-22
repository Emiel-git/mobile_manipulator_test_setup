#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mobile_manipulator_interfaces/msg/pose.hpp"
#include "mobile_manipulator_interfaces/srv/set_target_position.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class target_server : public rclcpp::Node
{
  public:
    target_server()
    : Node("target_position")
    {
      publisher_ = this->create_publisher<mobile_manipulator_interfaces::msg::Pose>("target_coordinates", 10);
      // service_ = this->create_service<mobile_manipulator_interfaces::srv::set_target_position>("set_target_position",&)
    }

  private:
    // void target_position_callback(this, request: mobile_manipulator_interfaces::srv::SetTargetPosition::Request, responds){

    // }
    // { // defining end-effector target position
    //   auto msg = mobile_manipulator_interfaces::msg::Pose();
    //   msg.position.x = 10.0;
    //   msg.position.y = 20.0;
    //   msg.position.z = 30.0;
    //   msg.orientation.x = 0;
    //   msg.orientation.y = 0;
    //   msg.orientation.z = 0;
    //   msg.orientation.w = 1;

    //   publisher_->publish(msg);
    // }

    rclcpp::Publisher<mobile_manipulator_interfaces::msg::Pose>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<target_server>());
  rclcpp::shutdown();
  return 0;
}