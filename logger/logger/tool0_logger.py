import rclpy
from rclpy.node import Node
from tf2_ros import TransformException 
from tf2_ros.transform_listener import TransformListener 
from tf2_ros.buffer import Buffer
from std_msgs.msg import Float32MultiArray

class TFListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Float32MultiArray, '/tool0_pose', 10)

        # Start listening to TF transform
        self.get_logger().info("Start listening to TF transform...")
        interval = 1/50
        self.timer = self.create_timer(interval, self.tf_callback)

    def tf_callback(self):
        try:
            # Get the latest transform from "world" to "tool0"
            transform_stamped_tool = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            transform_stamped_base = self.tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())

            # Create a PoseStamped message
            pose_msg = Float32MultiArray()
            pose_msg.data= [transform_stamped_base.transform.translation.x,
                            transform_stamped_tool.transform.translation.x,
                            transform_stamped_tool.transform.translation.y,
                            transform_stamped_tool.transform.translation.z,
                            transform_stamped_tool.transform.rotation.x,
                            transform_stamped_tool.transform.rotation.y,
                            transform_stamped_tool.transform.rotation.z,
                            transform_stamped_tool.transform.rotation.w]

            # Publish the PoseStamped message
            self.publisher.publish(pose_msg)

            # self.get_logger().info("Published PoseStamped message from 'world' to 'tool0'")
            
        except TransformException as ex:
            self.get_logger().warn(f"Failed to lookup TF transform: {ex}")
        
def main(args=None):
    rclpy.init(args=args)
    tf_listener_node = TFListenerNode()
    rclpy.spin(tf_listener_node)
    tf_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()