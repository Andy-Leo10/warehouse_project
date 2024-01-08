import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty as SrvEmpty
from time import sleep, time
from nav_msgs.msg import Odometry
from math import atan2, pi, fabs
from tf2_ros import TransformListener, Buffer

class RobotMovement(Node):
    def __init__(self, use_sim_time=True):
        super().__init__('robot_movement_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.MAX_LINEAR_SPEED = 0.2
        self.MAX_ANGULAR_SPEED = 0.25
        self.ZERO_LINEAR_SPEED = 0.0
        self.ZERO_ANGULAR_SPEED = 0.0
        #timer for control
        self.timer_period = 0.025  # seconds
        self.timer=self.create_timer(self.timer_period, self.timer_callback)
        self.timer.cancel()
        self.requested_yaw_angle = False
        self.requested_x_position = False
        self.requested_y_position = False
        self.requested_yaw_angle_desired = None
        self.requested_x_position_desired = None
        self.requested_y_position_desired = None
        self.kp = 0.5
        self.movement_completed = False
        #frames
        self.use_sim_time = use_sim_time
        if self.use_sim_time:
            self.reference_frame = 'robot_base_link'
            self.target_frame = 'cart_frame'
        else:   
            self.reference_frame = 'robot_base_footprint'
            self.target_frame = 'robot_cart_laser'
        #tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.relative_x_position = None
        self.relative_y_position = None
        self.relative_yaw_angle = None
        self.pos_x_accomplished = False
        self.pos_y_accomplished = False
        self.yaw_accomplished_1 = False
        self.yaw_accomplished_2 = False
        self.extra_move_accomplished = False
    
    def timer_callback(self):
        print(" ")
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
                self.get_logger().info(f"Coordinates: x={x:.2f}, y={y:.2f}")
                self.get_logger().info(f"Yaw angle: {yaw*180/pi:.2f} degrees")
                self.get_logger().info(f"Transform was done successfully")
                self.relative_x_position = x
                self.relative_y_position = y
                self.relative_yaw_angle = yaw
                
                #control x position: minimize x distance
                if self.pos_x_accomplished == False:
                    if self.controller_kp(desired_var=0.05, control_var=self.relative_x_position, kp=self.kp, tolerance=0.01, control_type='x_position'):
                        self.pos_x_accomplished = True
                        self.get_logger().info(">>>>>>>>> X position accomplished!")
                #control yaw angle 1: minimize yaw angle
                elif self.yaw_accomplished_1 == False:
                    if self.controller_kp(desired_var=0.0*pi/180, control_var=self.relative_yaw_angle, kp=self.kp*2, tolerance=1.0*pi/180, control_type='yaw_angle'):
                        self.yaw_accomplished_1 = True
                        self.get_logger().info(">>>>>>>>> Yaw angle accomplished!")
                #control y position: minimize y distance
                elif self.pos_y_accomplished == False:
                    if self.controller_kp(desired_var=0.55, control_var=self.relative_x_position, kp=self.kp, tolerance=0.01, control_type='y_position'):
                        self.pos_y_accomplished = True
                        self.get_logger().info(">>>>>>>>> Y position accomplished!")
                #control yaw angle 1: minimize yaw angle
                elif self.yaw_accomplished_2 == False:
                    if self.controller_kp(desired_var=0.0*pi/180, control_var=self.relative_yaw_angle, kp=self.kp*2, tolerance=1.0*pi/180, control_type='yaw_angle'):
                        self.yaw_accomplished_2 = True
                        self.get_logger().info(">>>>>>>>> Yaw angle accomplished!")
                #do extra move
                elif self.extra_move_accomplished == False:
                    self.move_forward(move_time=3.7)
                    self.extra_move_accomplished = True
                    self.get_logger().info(">>>>>>>>> Extra move accomplished!")
                else:
                    self.get_logger().info(">>>>>>>>> All control accomplished!")
                    self.disable_control()
                    self.movement_completed = True
            else:
                self.get_logger().warn(f"No transform available from {self.reference_frame} to {self.target_frame}")        
        except Exception as e:
            # Ignore exceptions for now
            self.get_logger().error(f"Failed to get transform from {self.reference_frame} to {self.target_frame}: {e}")
            pass
    
    def wait_for_movement_completion(self, timeout=120.0):
        start_time = time()
        rclpy.spin_once(self)  # Process callbacks
        while not self.movement_completed:
            if time() - start_time > timeout:
                raise TimeoutError("Movement did not complete within the specified timeout")
            sleep(0.025)  # Sleep for a short time to match the timer period
            rclpy.spin_once(self)  # Process callbacks
            
    def controller_kp(self, control_var, desired_var, kp, tolerance, control_type='yaw_angle'):
        error = -(desired_var - control_var)
        control_speed = kp * error
        
        if control_type == 'yaw_angle':
            self.robot_move(self.ZERO_LINEAR_SPEED, control_speed)
        elif control_type == 'x_position':
            self.robot_move(control_speed, self.ZERO_ANGULAR_SPEED)
        elif control_type == 'y_position':
            self.robot_move(control_speed, self.ZERO_ANGULAR_SPEED)
        else:
            self.get_logger().info(">>>>>>>>> Wrong control type!")
            return False
        
        if fabs(error) < tolerance:
            self.get_logger().info("desired value achieved!")
            self.robot_move(self.ZERO_LINEAR_SPEED, self.ZERO_ANGULAR_SPEED)
            return True
        else:
            if control_type == 'yaw_angle':
                self.get_logger().info(f"error[°]: {error*180/pi:.2f}, control_speed: {control_speed:.2f}")
            else:
                self.get_logger().info(f"error[d]: {error:.2f}, control_speed: {control_speed:.2f}")
            return False
        
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
        
    def enable_control(self):
        self.timer.reset()
    
    def disable_control(self):
        self.timer.cancel()
  