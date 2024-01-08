from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import math
from time import sleep
# for lifecycle
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
# for pose estimate 
from geometry_msgs.msg import PoseStamped

class ApiController(Node):
    def __init__(self):
        super().__init__('api_controller')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.client_lifecycle_localization = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes')
        self.client_lifecycle_pathplanner = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_pathplanner/manage_nodes')
    
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
                
    def lifecycle_localization_deactivate_node(self):
        req = ManageLifecycleNodes.Request()
        req.command = 1  # Use 1 for DEACTIVATE
        future = self.client_lifecycle_localization.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('>>> localization node deactivated')
        else:
            print('>>> failed to deactivate localization node')
    
    def lifecycle_localization_activate_node(self):
        req = ManageLifecycleNodes.Request()
        req.command = 2  # Use 2 for ACTIVATE
        future = self.client_lifecycle_localization.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('>>> localization node activated')
        else:
            print('>>> failed to activate localization node')
    
    def lifecycle_pathplanner_deactivate_node(self):
        req = ManageLifecycleNodes.Request()
        req.command = 1  # Use 1 for DEACTIVATE
        future = self.client_lifecycle_pathplanner.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('>>> pathplanner node deactivated')
        else:
            print('>>> failed to deactivate pathplanner node')

    def lifecycle_pathplanner_activate_node(self):
        req = ManageLifecycleNodes.Request()
        req.command = 2  # Use 2 for ACTIVATE
        future = self.client_lifecycle_pathplanner.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('>>> pathplanner node activated')
        else:
            print('>>> failed to activate pathplanner node')

    def pose_estimate(self, x, y, yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.navigator.setInitialPose(initial_pose)
    
if __name__ == '__main__':
    rclpy.init()
    api_controller = ApiController()
    api_controller.pose_estimate(0.0, 0.0, 0.0)
    sleep(5)
    api_controller.pose_estimate(-2.025, 2.006, 2.322)
    sleep(5)
    api_controller.pose_estimate(3.543, -4.066, 0.814)
    sleep(5)
    api_controller.lifecycle_pathplanner_deactivate_node()
    sleep(5)
    api_controller.lifecycle_pathplanner_activate_node()
    rclpy.shutdown()
