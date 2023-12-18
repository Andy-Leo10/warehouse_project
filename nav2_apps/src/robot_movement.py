import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty as SrvEmpty
from time import sleep, time

class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement_node')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.linear_speed = 0.2
        self.angular_speed = 0.25
        # service for localization
        self.reinitialize_service_client = self.create_client(SrvEmpty, '/reinitialize_global_localization')
    
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
        twist.linear.x = self.linear_speed
        end_time = time() + move_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after moving
        twist.linear.x = 0.0
        self.publisher.publish(twist)
    
    def move_backward(self, move_time):
        twist = Twist()
        twist.linear.x = -self.linear_speed
        end_time = time() + move_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after moving
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        
    def spin(self, spin_time, turn_factor):
        twist = Twist()
        twist.angular.z = turn_factor*self.angular_speed
        end_time = time() + spin_time
        while time() < end_time:
            self.publisher.publish(twist)
            sleep(0.1)  # Adjust this value to change the frequency of publishing
        # Stop the robot after spinning
        twist.angular.z = 0.0
        self.publisher.publish(twist)