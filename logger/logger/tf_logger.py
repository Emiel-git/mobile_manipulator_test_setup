#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from geometry_msgs.msg import PoseStamped
import math 
import os
import csv
from datetime import datetime
 
 
class FrameListener(Node):
    def __init__(self):
        super().__init__('logger')
        self.get_logger().info("logger node created.")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_ = self.create_subscription(PoseStamped,"/target_position",self.target_position_callback,10)

        # Call on_timer function on a set interval
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.on_timer)
        
        # Current position and orientation of the target frame with respect to the 
        # reference frame. x and y are in meters, and yaw is in radians.
        time = self.get_clock().now().nanoseconds*1e-9
        dt = datetime.fromtimestamp(time).strftime('%Y-%m-%d_%H:%M:%S.%f')
        self.folder_name = f'{dt}.csv'


    def target_position_callback(self,msg):
        roll, pitch, yaw = self.euler_from_quaternion(
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)

        time = float(f"{msg.header.stamp.sec}{msg.header.stamp.nanosec}")*1e-9
        time_secs= datetime.fromtimestamp(time).strftime('%S.%f')  
        
        data = [
                time_secs,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                roll, 
                pitch, 
                yaw]
        
        file_name = "target_position-world"
        header = ['Time','X', 'Y','Z','r','p','y']
        self.create_or_open_csv(self.folder_name,file_name,header,data )
        

    def on_timer(self):
        data = self.find_transform("base_link")
        if data != None:
            file_name = "base_link-world"
            header = ['Time', 'X', 'Y','Z','r','p','y']
            data = data
            self.create_or_open_csv(self.folder_name,file_name,header,data )

        data = self.find_transform("flange")
        if data != None:
            file_name = "flange-world"
            header = ['Time', 'X', 'Y','Z','r','p','y']
            data = data
            self.create_or_open_csv(self.folder_name,file_name,header,data )
     

    def find_transform(self,target_frame):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = target_frame
        to_frame_rel = 'world'
    
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        time= self.get_clock().now().nanoseconds*1e-9
        time_secs= datetime.fromtimestamp(time).strftime('%S.%f')  
        roll, pitch, yaw = self.euler_from_quaternion(
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)  
        data = [
                time_secs,
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                roll, 
                pitch, 
                yaw]   
        return data
        
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians
  

    def create_or_open_csv(self, folder_path, file_name,header,data):
        # Create the specified folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        file_path = os.path.join(folder_path, file_name)
        file_exists = os.path.exists(file_path)

        with open(file_path, mode='a', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)

            if not file_exists:
                # Write headers for all table if the file is newly created
                csv_writer.writerow(header)

            csv_writer.writerow(data)

 
def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()