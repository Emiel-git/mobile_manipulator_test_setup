#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


class BaseMovementNode(Node):

    def __init__(self):
        super().__init__("base_movement")
        self.get_logger().info("base_movement_node created")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        self.start_time = self.get_clock().now().nanoseconds
        self.position = 0.0
        self.interval = self.start_time
        self.timer_ = self.create_timer(0.001, self.publish_base_position)

    def joint_state_callback(self, msg):
        # self.get_logger().info(str(msg))
        pass


    def publish_base_position(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        time = self.get_clock().now()
        t.header.stamp = time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # define time duration in sec
        duration = (time.nanoseconds-self.start_time)*1e-9
        velocity = self.generate_velocity(1.5,0.25,0.1,0.25,duration)
        # define translation
        self.position += velocity*0.001
        t.transform.translation.x = self.position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

    def generate_velocity(self, max_pos,max_vel,acc,decc,duration):
        # accelerate till max velocity is reached
        if self.position <= max_pos:
            velocity = acc * duration
            if velocity >= max_vel:
                velocity = max_vel
            # determine when to deccelerate
            t_acc= np.sqrt(2*velocity/acc)
            s_acc = 0.5*acc*t_acc*t_acc
            t_decc = np.sqrt(2*velocity/(decc))
            s_decc = velocity + 0.5*(-decc)*t_decc*t_acc
            s_const = max_pos-s_acc-s_decc
            t_const = s_const/max_vel
            start_decc_time = t_acc+t_const

            if self.position >= (max_pos-s_decc):
                velocity = velocity-(decc*(duration-start_decc_time))
        else:
            velocity = 0.0
        
        return velocity
        
        

        

def main(args=None):
    rclpy.init(args=args)
    node = BaseMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
