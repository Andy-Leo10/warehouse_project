import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from rclpy.node import Node
from std_msgs.msg import Empty, String
from time import sleep, time
from geometry_msgs.msg import Polygon, Point32
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty as SrvEmpty

class ApiController:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def go_to(self, x, y, yaw, destination_name):
        while True:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.z = math.sin(yaw / 2)
            goal_pose.pose.orientation.w = math.cos(yaw / 2)
            
            self.navigator.goToPose(goal_pose)

            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 17 == 0:
                    remaining_time_s = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                    print('Remaining time for <{}> (x={}, y={}, yaw={}): {:.1f} s'.format(destination_name, x, y, yaw, remaining_time_s))

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
                        self.navigator.cancelTask()

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'----------------------------- Goal succeeded! {destination_name} -----------------------')
                break
            elif result == TaskResult.CANCELED:
                print(f'----------------------------- Goal was canceled! {destination_name} -----------------------')
                break
            elif result == TaskResult.FAILED:
                print(f'----------------------------- Goal failed! {destination_name} ----------------------')
                print(f'----------------------------- AGAIN!!!!!!!!!!!!  ----------------------')
            else:
                print('Goal has an invalid return status!')
                self.navigator.lifecycleShutdown()

class RobotElevator(Node):
    def __init__(self, wait_time=2, use_sim_time=True):
        super().__init__('robot_elevator_node')
        self.wait_time = wait_time
        self.use_sim_time = use_sim_time
        if self.use_sim_time:
            self.publisher_down = self.create_publisher(Empty, '/elevator_down', 10)
            self.publisher_up = self.create_publisher(Empty, '/elevator_up', 10)
        else:
            self.publisher_down = self.create_publisher(String, '/elevator_down', 10)
            self.publisher_up = self.create_publisher(String, '/elevator_up', 10)

    def publish_down(self):
        if self.use_sim_time:
            msg = Empty()
        else:
            msg = String()
            msg.data = ''    
        print('Publishing elevator down')
        self.publisher_down.publish(msg)
        sleep(self.wait_time)

    def publish_up(self):
        if self.use_sim_time:
            msg = Empty()
        else:
            msg = String()
            msg.data = ''
        print('Publishing elevator up')
        self.publisher_up.publish(msg)
        sleep(self.wait_time)

class RobotFootprint(Node):
    def __init__(self):
        super().__init__('robot_footprint_node')
        self.publisher_global = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.publisher_local = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

    def update_footprint_normal(self):
        footprint = Polygon()
        sqr_side = 0.500
        footprint.points = [
            Point32(x= sqr_side/2, y= sqr_side/2),
            Point32(x= sqr_side/2, y=-sqr_side/2),
            Point32(x=-sqr_side/2, y=-sqr_side/2),
            Point32(x=-sqr_side/2, y= sqr_side/2)
        ]
        print('Publishing footprint normal')
        self.publisher_global.publish(footprint)
        self.publisher_local.publish(footprint)
        sleep(1)
        
    def update_footprint_cart(self):
        footprint = Polygon()
        sqr_side = 0.800
        footprint.points = [
            Point32(x= sqr_side/2, y= sqr_side/2),
            Point32(x= sqr_side/2, y=-sqr_side/2),
            Point32(x=-sqr_side/2, y=-sqr_side/2),
            Point32(x=-sqr_side/2, y= sqr_side/2)
        ]
        print('Publishing footprint cart')
        self.publisher_global.publish(footprint)
        self.publisher_local.publish(footprint)
        sleep(1)

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
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move shelf to shipping position')
    parser.add_argument('--use_sim_time', type=str, default=True, help='Use simulation (Gazebo) clock if true')
    
    api = ApiController()
    use_sim_time = parser.parse_args().use_sim_time.lower() in ['true', '1', 't', 'y', 'yes']
    robot_elevator = RobotElevator(wait_time=10, use_sim_time=use_sim_time)
    robot_footprint = RobotFootprint()
    robot_movement = RobotMovement()
    
    if use_sim_time:
        loading_position = (1.921, 1.853, -0.801, 'loading position')
        middle_position = (-1.465,-1.252,-0.871, 'middle position')
        shipping_position = (0.742, -3.852, -0.801, 'shipping position')
        init_position = (-2.363, -1.845, 0.783, 'initial position')
    else:
        loading_position = (4.420,-1.390,-1.787, 'loading position')
        middle_position = (0.981,-0.262,-1.897, 'middle position')
        shipping_position = (0.233,-2.867,-1.790, 'shipping position')
        init_position = (-0.145,-0.031,-0.123, 'initial position')
    
    robot_footprint.update_footprint_normal()
    # step1
    api.go_to(*loading_position)
    robot_movement.move_forward(move_time=6)
    robot_elevator.publish_up()
    robot_footprint.update_footprint_cart()
    robot_movement.move_backward(move_time=8)
    robot_movement.spin(spin_time=8, turn_factor=-1)
    # step2
    api.go_to(*middle_position)
    api.go_to(*shipping_position)
    robot_elevator.publish_down()
    robot_footprint.update_footprint_normal()
    robot_movement.move_backward(move_time=6)
    # step3
    api.go_to(*init_position)
    