import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from time import sleep, time

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