#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetPositionNode(Node):

    def __init__(self):
        super().__init__("target_position")
        self.declare_parameter('X', 0.0)
        self.declare_parameter('Y', 0.0)
        self.declare_parameter('Z', 0.0)
        self.declare_parameter('r', 0.0)
        self.declare_parameter('p', 0.0)
        self.declare_parameter('y', 0.0)
                       
        self.pub_ = self.create_publisher(PoseStamped, "/target_position", 10)
        self.get_logger().info("Target_position_node created.")
        self.timer_ = self.create_timer(0.5, self.publish_target_position)

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
        X = self.get_parameter('X').get_parameter_value().double_value
        Y = self.get_parameter('Y').get_parameter_value().double_value
        Z = self.get_parameter('Z').get_parameter_value().double_value
        
        qx,qy,qz,qw = self.get_quaternion_from_euler(roll, pitch, yaw)

        self.msg = PoseStamped()
        self.msg.header.frame_id = "/target"
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.pose.position.x = X * 10e-3
        self.msg.pose.position.y = Y * 10e-3
        self.msg.pose.position.z = Z * 10e-3
        self.msg.pose.orientation.x = qx
        self.msg.pose.orientation.y = qy
        self.msg.pose.orientation.z = qz
        self.msg.pose.orientation.w = qw
        self.pub_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()