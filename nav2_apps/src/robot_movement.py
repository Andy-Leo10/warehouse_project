import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty as SrvEmpty
from time import sleep, time
from nav_msgs.msg import Odometry
from math import atan2, pi, fabs
from tf2_ros import TransformListener, Buffer

class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.MAX_LINEAR_SPEED = 0.2
        self.MAX_ANGULAR_SPEED = 0.25
        self.ZERO_LINEAR_SPEED = 0.0
        self.ZERO_ANGULAR_SPEED = 0.0
        # service for localization
        self.reinitialize_service_client = self.create_client(SrvEmpty, '/reinitialize_global_localization')
        #subscriber for odom
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.positionX = None
        self.positionY = None
        self.angle_yaw = 0.0
        #timer for control
        self.timer_period = 0.1  # seconds
        self.timer=self.create_timer(self.timer_period, self.timer_callback)
        self.timer.cancel()
        self.requested_yaw_angle = False
        self.requested_x_position = False
        self.requested_y_position = False
        self.requested_yaw_angle_desired = None
        self.requested_x_position_desired = None
        self.requested_y_position_desired = None
        self.movement_completed = True
        #tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def achieve_yaw_angle(self,yaw_angle_desired):
        self.movement_complete = False
        #requested_yaw_angle_desired in radians!
        self.requested_yaw_angle = True
        self.requested_yaw_angle_desired = yaw_angle_desired*pi/180
        self.timer.reset()
        
    def achieve_x_position(self,x_position_desired):
        self.movement_complete = False
        self.requested_x_position = True
        self.requested_x_position_desired = x_position_desired
        self.timer.reset()
        
    def achieve_y_position(self,y_position_desired):
        self.movement_complete = False
        self.requested_y_position = True
        self.requested_y_position_desired = y_position_desired
        self.timer.reset()
    
    def timer_callback(self):
        if self.requested_yaw_angle:
            if self.controller_kp(self.angle_yaw, self.requested_yaw_angle_desired, 
                                  kp=1.0, tolerance=1.0*pi/180, control_type='yaw_angle'):
                self.requested_yaw_angle = False
                self.movement_complete = True
                self.timer.cancel()
        elif self.requested_x_position:
            if self.controller_kp(self.positionX, self.requested_x_position_desired, 
                                  kp=1.0, tolerance=0.01, control_type='x_position'):
                self.requested_x_position = False
                self.movement_complete = True
                self.timer.cancel()
        elif self.requested_y_position:
            if self.controller_kp(self.positionY, self.requested_y_position_desired, 
                                  kp=1.0, tolerance=0.01, control_type='y_position'):
                self.requested_y_position = False
                self.movement_complete = True
                self.timer.cancel()
        else:
            self.timer.cancel()
    
    def wait_for_movement_completion(self):
        rclpy.spin_once(self)  # Process callbacks
        while not self.movement_complete:
            sleep(0.1)  # Sleep for a short time to avoid busy waiting
            rclpy.spin_once(self)  # Process callbacks
              
    def odom_callback(self, msg):
        self.positionX = msg.pose.pose.position.x
        self.positionY = msg.pose.pose.position.y
        #calculate orientation
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        '''
        formulas quaternion to euler
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        '''
        self.angle_yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        #print(f'x: {self.positionX:.2f}, y: {self.positionY:.2f}, yaw: {self.angle_yaw*180/pi:.2f}')
        
        
    def reinitialize_localization(self):
        request = SrvEmpty.Request()
        future = self.reinitialize_service_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('Successfully called service /reinitialize_global_localization')
            sleep(11)
        else:
            print('Failed to call service /reinitialize_global_localization')
                
    def move_forward(self, move_time):
        twist = Twist()
        twist.linear.x = self.MAX_LINEAR_SPEED
        end_time = time() + move_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after moving
        twist.linear.x = 0.0
        self.publisher.publish(twist)
    
    def move_backward(self, move_time):
        twist = Twist()
        twist.linear.x = -self.MAX_LINEAR_SPEED
        end_time = time() + move_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after moving
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
    def spin(self, spin_time, turn_factor):
        twist = Twist()
        twist.angular.z = turn_factor*self.MAX_ANGULAR_SPEED
        end_time = time() + spin_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after spinning
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
    def saturate(self, var, min_val, max_val):
        if var > max_val:
            return max_val
        elif var < min_val:
            return min_val
        else:
            return var
        
    def robot_move(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = self.saturate(linear_speed, -self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        twist.angular.z = self.saturate(angular_speed, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
        self.publisher.publish(twist)
        
    def controller_kp(self, control_var, desired_var, kp, tolerance, control_type='yaw_angle'):
        error = desired_var - control_var
        control_speed = kp * error
        
        if control_type == 'yaw_angle':
            self.robot_move(self.ZERO_LINEAR_SPEED, control_speed)
        elif control_type == 'x_position':
            #vector arrow pointing to + x axis when yaw [-90,90]
            if self.angle_yaw > -pi/2 and self.angle_yaw < pi/2:
                self.robot_move(control_speed, self.ZERO_ANGULAR_SPEED)
            else:
                self.robot_move(-control_speed, self.ZERO_ANGULAR_SPEED)
        elif control_type == 'y_position':
            #vector arrow pointing to + y axis when yaw [0,180]
            if self.angle_yaw > 0 and self.angle_yaw < pi:
                self.robot_move(control_speed, self.ZERO_ANGULAR_SPEED)
            else:
                self.robot_move(-control_speed, self.ZERO_ANGULAR_SPEED)
        else:
            self.get_logger().info(">>>>>>>>> Wrong control type!")
            return False
        
        if fabs(error) < tolerance:
            self.get_logger().info("desired value achieved!")
            self.robot_move(self.ZERO_LINEAR_SPEED, self.ZERO_ANGULAR_SPEED)
            return True
        else:
            if control_type == 'yaw_angle':
                self.get_logger().info(f"error: {error*180/pi:.2f}, control_speed: {control_speed:.2f}")
            else:
                self.get_logger().info(f"error: {error:.2f}, control_speed: {control_speed:.2f}")
            return False
    
    def get_transform(self, reference_frame='robot_odom', target_frame='cart_frame'):
        try:
            # Get the transformation from reference_frame to target_frame
            transform = self.tf_buffer.lookup_transform(reference_frame, target_frame, rclpy.time.Time())

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

            return {
                'reference_frame': reference_frame,
                'target_frame': target_frame,
                'timestamp': timestamp,
                'coordinates': (x, y, z),
                'yaw': yaw
            }
        except Exception as e:
            self.get_logger().error(f"Failed to get transform from {reference_frame} to {target_frame}: {e}")
            return None