import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math

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
            if feedback and i % 5 == 0:
                remaining_time_s = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print('Remaining time for <{}> (x={}, y={}, yaw={}): {:.1f} s'.format(destination_name, x, y, yaw, remaining_time_s))

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
                    self.navigator.cancelTask()

                #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #    goal_pose.pose.position.x = -3.0
                #    self.navigator.goToPose(goal_pose)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        #self.navigator.lifecycleShutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move shelf to shipping position')
    parser.add_argument('--use_sim_time', type=str, default=True, help='Use simulation (Gazebo) clock if true')
    
    api = ApiController()
    use_sim_time = parser.parse_args().use_sim_time.lower() in ['true', '1', 't', 'y', 'yes']
    
    if use_sim_time:
        loading_position = (1.858, 2.256, -0.783, 'loading position')
        shipping_position = (0.742, -3.852, 2.350, 'shipping position')
        init_position = (-2.363, -1.845, 0.783, 'initial position')
    else:
        loading_position = (1.1, 1.1, -0.783, 'loading position')
        shipping_position = (0.742, -1.5, 2.350, 'shipping position')
        init_position = (-1.363, -0.45, 0.783, 'initial position')

    api.go_to(*loading_position)
    api.go_to(*shipping_position)
    api.go_to(*init_position)
