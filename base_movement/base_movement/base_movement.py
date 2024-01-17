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
        self.declare_parameter('max_pos', [1.5])
        self.declare_parameter('max_v', [0.25])
        self.declare_parameter('max_a', 0.5)
        self.declare_parameter('max_d', 0.1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(JointState,'/joint_states',self.joint_state_callback,10)
        
        self.start_time = self.get_clock().now().nanoseconds
        self.position = 0.0
        self.i = 0
        self.set_decc_time = 0
        self.interval = self.start_time

        self.timer_interval = 0.001
        self.timer_ = self.create_timer(self.timer_interval, self.publish_base_position)

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

        max_pos = self.get_parameter('max_pos').get_parameter_value().double_array_value
        max_v = self.get_parameter('max_v').get_parameter_value().double_array_value
        max_a = self.get_parameter('max_a').get_parameter_value().double_value
        max_d = self.get_parameter('max_d').get_parameter_value().double_value

        # define time duration in sec
        duration = (time.nanoseconds-self.start_time)*1e-9
        if self.i < len(max_pos)-1:
            velocity = self.generate_velocity(max_pos[self.i],max_v[self.i],max_a,0,duration)
        elif self.i== len(max_pos)-1:
            velocity = self.generate_velocity(max_pos[self.i],max_v[self.i],max_a,max_d,duration)
        else:
            velocity = 0.0
        # define translation
        self.position += velocity*self.timer_interval
        t.transform.translation.x = self.position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

    def generate_velocity(self, max_pos,max_vel,acc,decc,duration):
        # accelerate till max velocity is reached
        velocity = 0.0
        if self.position <= max_pos:
            velocity = acc * duration
            if velocity >= max_vel:
                velocity = max_vel    
            if decc == 0:
                return velocity
            else:
                t_decc = np.sqrt(2*velocity/(decc))
                s_decc = 0.5*decc*t_decc*t_decc

                if self.position >= (max_pos-s_decc):
                    if not self.set_decc_time:
                        self.start_decc_time = duration
                        self.set_decc_time = 1
                    velocity = velocity-(decc*(duration-self.start_decc_time))   

        else:
            self.i +=1
        return velocity
        

def main(args=None):
    rclpy.init(args=args)
    node = BaseMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
