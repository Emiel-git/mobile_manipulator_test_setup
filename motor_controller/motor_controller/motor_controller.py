#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import RPi.GPIO as GPIO

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.get_logger().info("motor_controller node created.")
        self.outpin =12
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.outpin, GPIO.OUT)
        self.pi_pwm = GPIO.PWM(self.outpin,8000)                #create PWM instance with frequency
        self.pi_pwm.start(0)                    #start PWM of required Duty Cycle 
        self.sub_ = self.create_subscription(Float64,"/base_velocity",self.velocity_callback,10)

    def velocity_callback(self,msg):
        velocity = msg.data
        pwm = self.remap_value(velocity)
        self.pi_pwm.ChangeDutyCycle(pwm)
        self.get_logger().info(str(pwm))

    def remap_value(self, value):
        max_speed = 0.5
        if -max_speed <= value <= max_speed:
            new_val = (value/(2*max_speed) + 0.5)*100
            return new_val
        else:
            self.get_logger().error("value out of range")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

