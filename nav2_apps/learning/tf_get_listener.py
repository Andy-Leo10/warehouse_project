import os
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{time}] [{name}]: {message}'
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
os.environ['RCUTILS_LOGGING_MIN_SEVERITY'] = 'DEBUG'
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from math import atan2, pi
from time import sleep

class TFListener(Node):
    def __init__(self, node_name, reference_frame='robot_odom', target_frame='cart_frame'):
        super().__init__(node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.reference_frame = reference_frame
        self.target_frame = target_frame

    def get_transform(self):
        # the method will try to get the transformation 
        try:
            # Check if the transformation is available
            if self.tf_buffer.can_transform(self.reference_frame, self.target_frame, rclpy.time.Time()):
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

                #self.get_logger().info(f"Reference frame: {reference_frame}")
                #self.get_logger().info(f"Target frame: {target_frame}")
                #self.get_logger().info(f"Timestamp: {timestamp:.2f} seconds")
                #self.get_logger().info(f"Coordinates: x={x:.2f}, y={y:.2f}")
                #self.get_logger().info(f"Yaw angle: {yaw*180/pi:.2f} degrees")
                #print("")
                self.get_logger().info(f"Transform was done successfully")
                return {
                    'reference_frame': reference_frame,
                    'target_frame': target_frame,
                    'timestamp': timestamp,
                    'coordinates': (x, y, z),
                    'yaw': yaw
                }
            else:
                self.get_logger().warn(f"No transform available from {self.reference_frame} to {self.target_frame}")
            
        except Exception as e:
            # Ignore exceptions for now
            self.get_logger().error(f"Failed to get transform from {self.reference_frame} to {self.target_frame}: {e}")
            pass
        
        rclpy.spin_once(self)

    def obtain_transform(self):
        # the method will obtain 1 transformation 
        transform_complete = False
        while transform_complete == False:
            transform = self.get_transform()
            if transform != None:
                transform_complete = True
                return transform
            else:
                sleep(0.1)
                pass

def main():
    try:
        rclpy.init()
        tf_listener1 = TFListener('tf_listener1')
        #for _ in range(10):
            #rclpy.spin_once(tf_listener1)
            #transform = tf_listener1.get_transform()
            #print(transform)
        t=tf_listener1.obtain_transform()
        print(t)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()