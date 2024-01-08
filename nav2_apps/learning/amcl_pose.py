#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        self.d = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('Pose: "%s"' % msg.pose.pose.position)

def main(args=None):
    rclpy.init(args=args)

    pose_reader = PoseReader()

    rclpy.spin(pose_reader)

    pose_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()