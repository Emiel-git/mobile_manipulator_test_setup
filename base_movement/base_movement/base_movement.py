#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import time



class BaseMovementNode(Node):

    def __init__(self):
        super().__init__("base_movement")
        self.get_logger().info("base_movement_node created")
        self.declare_parameter('max_pos', [0.6,0.8,1.5])
        self.declare_parameter('max_v', [0.25,0.1,0.2])
        self.declare_parameter('max_a', [0.1,-0.01,0.25,-0.1])

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_ = self.create_publisher(Float64,"/base_velocity",10)
        

        self.start_time = self.get_clock().now().nanoseconds
        self.position = 0
        self.velocity = 0
        self.i = 0
        self.set_decc_time = 0
        self.interval = 0

        self.timer_interval = 1/950 # 950Hz
        self.timer_ = self.create_timer(self.timer_interval, self.publish_base_position)

    
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
        max_a = self.get_parameter('max_a').get_parameter_value().double_array_value

        # define time duration in sec
        duration = (time.nanoseconds-self.start_time)*1e-9
        if self.i <= len(max_pos)-1:
            self.velocity = self.generate_velocity(max_pos,max_v,max_a,duration)
        else:
            self.velocity = 0.0
        # define translation
        self.position += self.velocity*self.timer_interval
        t.transform.translation.x = self.position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        self.tf_broadcaster.sendTransform(t)
        msg = Float64()
        msg.data = self.velocity
        self.pub_.publish(msg)

    def generate_velocity(self, max_pos,max_vel,acc,duration):        
        # accelerate till max velocity is reached
        velocity = self.velocity
        if self.position <= max_pos[self.i]:
            velocity = self.velocity + acc[self.i] * (duration - self.interval)
            if velocity >= max_vel[self.i] and acc[self.i] > 0:
                velocity = max_vel[self.i]
            elif velocity <= max_vel[self.i] and acc[self.i] < 0:
                velocity = max_vel[self.i]

            if self.i < len(max_pos)-1:
                self.interval = duration
                return velocity
            elif not self.set_decc_time:
                decc = -acc[self.i+1]
                t_decc = np.sqrt(2*self.velocity/decc)
                s_decc = self.velocity*t_decc-0.5*decc*t_decc*t_decc
                if self.position >= (max_pos[self.i]-s_decc):
                    self.start_decc_time = duration
                    self.set_decc_time = 1
                    velocity = self.velocity-(decc*(duration-self.start_decc_time))
            else:
                decc = -acc[self.i+1]
                velocity = self.velocity-(decc*(duration-self.start_decc_time))
                if velocity < 0:
                    velocity = 0.0   
        else:
            self.i +=1
            
        self.interval = duration    
        return velocity
        

def main(args=None):
    time.sleep(5)
    rclpy.init(args=args)
    node = BaseMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
