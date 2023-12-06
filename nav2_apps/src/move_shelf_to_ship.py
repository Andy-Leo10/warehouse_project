import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from rclpy.node import Node
from std_msgs.msg import Empty, String
from time import sleep
from geometry_msgs.msg import Polygon, Point32

class ApiController:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def go_to(self, x, y, yaw, destination_name):
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

                #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #    goal_pose.pose.position.x = -3.0
                #    self.navigator.goToPose(goal_pose)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'-----------------------------Goal succeeded! {destination_name} -----------------------')
        elif result == TaskResult.CANCELED:
            print(f'-----------------------------Goal was canceled! {destination_name} -----------------------')
        elif result == TaskResult.FAILED:
            print(f'-----------------------------Goal failed! {destination_name} ----------------------')
        else:
            print('Goal has an invalid return status!')

        #self.navigator.lifecycleShutdown()

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

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move shelf to shipping position')
    parser.add_argument('--use_sim_time', type=str, default=True, help='Use simulation (Gazebo) clock if true')
    
    api = ApiController()
    use_sim_time = parser.parse_args().use_sim_time.lower() in ['true', '1', 't', 'y', 'yes']
    robot_elevator = RobotElevator(wait_time=2, use_sim_time=use_sim_time)
    robot_footprint = RobotFootprint()
    
    if use_sim_time:
        loading_position = (1.858, 2.256, -0.783, 'loading position')
        shipping_position = (0.742, -3.852, 2.350, 'shipping position')
        init_position = (-2.363, -1.845, 0.783, 'initial position')
        middle_position = (0.518,0.618,-2.360, 'middle position')
    else:
        loading_position = (1.1, 1.1, -0.783, 'loading position')
        shipping_position = (0.742, -1.5, 2.350, 'shipping position')
        init_position = (-1.363, -0.45, 0.783, 'initial position')
        middle_position = (0.518,0.618,-2.360, 'middle position')

    robot_footprint.update_footprint_normal()
    # step1
    api.go_to(*loading_position)
    #robot_elevator.publish_up()
    robot_footprint.update_footprint_cart()
    # step2
    api.go_to(*middle_position)
    api.go_to(*shipping_position)
    #robot_elevator.publish_down()
    robot_footprint.update_footprint_normal()
    # step3
    api.go_to(*init_position)
    
        
        