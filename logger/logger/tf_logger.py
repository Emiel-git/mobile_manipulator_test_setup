#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TfLogger(Node):

    def __init__(self):
        super().__init__("tf_logger")
        self.pub_ = self.create_subscription(PoseStamped,"/tf",tf_sub_callback,10)

    def tf_sub_callback(self,msg):
        