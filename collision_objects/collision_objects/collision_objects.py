#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from mobile_manipulator_interfaces.msg import Pose


class TargetPositionNode(Node):

    def __init__(self):
        super().__init__("collision_objects")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('X',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('Y',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('Z',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('p',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('y',rclpy.Parameter.Type.DOUBLE_ARRAY)
            ])
        self.pub_ = self.create_publisher(Pose, "/target_position", 10)
        self.get_logger().info("Target_position_node created.")
        self.timer_ = self.create_timer(2.0, self.publish_target_position)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [qx, qy, qz, qw]
    
    def publish_target_position(self):
        roll = self.get_parameter('r').get_parameter_value().double_value
        pitch = self.get_parameter('p').get_parameter_value().double_value
        yaw = self.get_parameter('y').get_parameter_value().double_value

        qx,qy,qz,qw = self.get_quaternion_from_euler(roll, pitch, yaw)

        self.msg = Pose()
        self.msg.position.x = self.get_parameter('X').get_parameter_value().double_value
        self.msg.position.y = self.get_parameter('Y').get_parameter_value().double_value
        self.msg.position.z = self.get_parameter('Z').get_parameter_value().double_value
        self.msg.orientation.x = qx
        self.msg.orientation.y = qy
        self.msg.orientation.z = qz
        self.msg.orientation.w = qw
        self.pub_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()