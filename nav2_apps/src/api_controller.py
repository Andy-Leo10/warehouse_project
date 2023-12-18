from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math

class ApiController:
    def __init__(self):
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