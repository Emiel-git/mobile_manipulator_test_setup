#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
# from enum import Enum


class CollisionObjects(Node):

    def __init__(self):
        super().__init__("collision_objects")
        self.get_logger().info("collision_objects_node created")

        self.declare_parameter('object_names', ["block","spere","cylinder","cone"])
        self.declare_parameter('shape_types', [1,2,3,4])
        self.declare_parameter('shape_dimensions_0', [0.5, 0.25, 0.25, 0.25])
        self.declare_parameter('shape_dimensions_1', [0.5, 0.0, 1.0, 1.0])
        self.declare_parameter('shape_dimensions_2', [0.5, 0.0, 0.0, 0.0])
        self.declare_parameter('X', [0.0, 1.0, 2.0, 3.0])
        self.declare_parameter('Y', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('Z', [3.0, 3.0, 3.0, 3.0])
        self.declare_parameter('r', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('p', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('y', [0.0, 0.0, 0.0, 0.0])

        self.pub_ = self.create_publisher(CollisionObject, "/collision_objects", 10)
        sample_frequency = 1/50		#50Hz
        self.timer_ = self.create_timer(sample_frequency, self.publish_collision_objects)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
        return [qx, qy, qz, qw]
    
    def publish_collision_objects(self):
        object_names = self.get_parameter('object_names').get_parameter_value().string_array_value
        shape_types = self.get_parameter('shape_types').get_parameter_value().integer_array_value
        shape_dimensions_0 = self.get_parameter('shape_dimensions_0').get_parameter_value().double_array_value
        shape_dimensions_1 = self.get_parameter('shape_dimensions_1').get_parameter_value().double_array_value
        shape_dimensions_2 = self.get_parameter('shape_dimensions_2').get_parameter_value().double_array_value
        Xs = self.get_parameter('X').get_parameter_value().double_array_value
        Ys = self.get_parameter('Y').get_parameter_value().double_array_value
        Zs = self.get_parameter('Z').get_parameter_value().double_array_value
        rs = self.get_parameter('r').get_parameter_value().double_array_value
        ps = self.get_parameter('p').get_parameter_value().double_array_value 
        ys = self.get_parameter('y').get_parameter_value().double_array_value

        for i in range(len(object_names)):
            qx,qy,qz,qw = self.get_quaternion_from_euler(rs[i], ps[i], ys[i])
            collision_obj = CollisionObject()
            collision_obj.id = object_names[i]

            primitive = SolidPrimitive()
            primitive.type = shape_types[i]
            
            obj_dimensions = []
            if shape_types[i] == 1:
                obj_dimensions.append(shape_dimensions_0[i])
                obj_dimensions.append(shape_dimensions_1[i])
                obj_dimensions.append(shape_dimensions_2[i])
            elif shape_types[i] == 2:
                obj_dimensions.append(shape_dimensions_0[i])
            elif shape_types[i] == 3 or shape_types[i] == 4:
                obj_dimensions.append(shape_dimensions_0[i])
                obj_dimensions.append(shape_dimensions_1[i])
            else:
                self.get_logger().error("invalid shape type input")
            primitive.dimensions = obj_dimensions

            obj_pose = Pose()
            obj_pose.position.x = Xs[i]
            obj_pose.position.y = Ys[i]
            obj_pose.position.z = Zs[i]
            obj_pose.orientation.x = qx
            obj_pose.orientation.y = qy
            obj_pose.orientation.z = qz
            obj_pose.orientation.w = qw

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(obj_pose)
            collision_obj.operation = collision_obj.ADD
            
            self.pub_.publish(collision_obj)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjects()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
