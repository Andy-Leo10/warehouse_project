import os
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time}] [{name}]: {message}'
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from math import atan2, pi
from rclpy.executors import MultiThreadedExecutor

class TFListener(Node):
    def __init__(self, node_name, reference_frame='robot_odom', target_frame='cart_frame'):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.reference_frame = reference_frame
        self.target_frame = target_frame
        
        # Create a timer with a callback that is called every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # Get the transformation from /robot_odom to /robot_base_footprint
            transform = self.tf_buffer.lookup_transform(self.reference_frame, self.target_frame, rclpy.time.Time())
            # self.get_logger().info(f"Transform: {transform}")
            
            # Extract frame IDs
            reference_frame = transform.header.frame_id
            target_frame = transform.child_frame_id
            
            # Extract timestamp
            timestamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9

            # Extract coordinates
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Extract quaternions
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))

            self.get_logger().info(f"Reference frame: {reference_frame}")
            self.get_logger().info(f"Target frame: {target_frame}")
            self.get_logger().info(f"Timestamp: {timestamp:.2f} seconds")
            self.get_logger().info(f"Coordinates: x={x:.2f}, y={y:.2f}")
            self.get_logger().info(f"Yaw angle: {yaw*180/pi:.2f} degrees")
            print("")
            
        except Exception as e:
            # Ignore exceptions for now
            self.get_logger().error(f"Failed to get transform from {self.reference_frame} to {self.target_frame}: {e}")
            pass
            
        #rclpy.spin_once(self)

def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor()

        tf_listener1 = TFListener('tf_listener1')
        executor.add_node(tf_listener1)

        tf_listener2 = TFListener('tf_listener2', 'map', 'robot_odom')
        executor.add_node(tf_listener2)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()