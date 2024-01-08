import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Polygon, Point32
from time import sleep, time

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