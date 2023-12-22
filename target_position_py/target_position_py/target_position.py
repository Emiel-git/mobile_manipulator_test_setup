#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mobile_manipulator_interfaces.msg import Pose
from mobile_manipulator_interfaces.srv import SetTargetPosition


class TargetPositionNode(Node):

    def __init__(self):
        super().__init__("target_position")
        self.get_logger().info("Target_position_node created.")
        self.pub_ = self.create_publisher(Pose, "target_postion", 10)
        self.timer_ = self.create_timer(0.1, self.publish_target_position)
        self.create_service(SetTargetPosition, "request_target_position", self.target_position_callback)    

    def publish_target_position(self):
        self.msg = Pose()
        self.msg.position.x = 10.0
        self.msg.position.y = 20.0
        self.msg.position.z = 30.0
        self.msg.orientation.x = 0.0
        self.msg.orientation.y = 0.0
        self.msg.orientation.z = 0.0
        self.msg.orientation.w = 1.0
        self.pub_.publish(self.msg)

    def target_position_callback(self, request:SetTargetPosition.Request, response: SetTargetPosition.Response):
        self.get_logger().info("request for target position")
        response.target_position = self.msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()