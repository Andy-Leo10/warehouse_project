import argparse
import rclpy
from api_controller import ApiController
from robot_elevator import RobotElevator
from robot_footprint import RobotFootprint
from robot_movement2 import RobotMovement
from time import sleep
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move shelf to shipping position')
    parser.add_argument('--use_sim_time', type=str, default=True, help='Use simulation (Gazebo) clock if true')
    
    rclpy.init()
    api = ApiController()
    use_sim_time = parser.parse_args().use_sim_time.lower() in ['true', '1', 't', 'y', 'yes']
    robot_elevator = RobotElevator(wait_time=10, use_sim_time=use_sim_time)
    robot_footprint = RobotFootprint()
    robot_movement = RobotMovement(use_sim_time=use_sim_time)
    
    if use_sim_time:
        loading_position = (1.166, 1.501, 0.783, 'loading position')
        middle_position = (-1.465,-1.252,-0.871, 'middle position')
        shipping_position = (0.742, -3.852, -0.801, 'shipping position')
        init_position = (-2.363, -1.845, 0.783, 'initial position')
    else:
        loading_position = (3.495,-1.390,-0.180, 'loading position')#try#3.495,-1.390,-0.180    #og#3.538,-0.934,-0.123
        middle_position = (0.981,-0.262,-1.897, 'middle position')
        shipping_position = (0.233,-2.867,-1.790, 'shipping position')
        init_position = (-0.145,-0.031,-0.123, 'initial position')
    
    robot_footprint.update_footprint_normal()
    
    if use_sim_time:
        print('>>> IMPROVING LOCALIZATION')
        #robot_movement.spin(spin_time=30, turn_factor=-1)
        # step1
        api.go_to(*loading_position)
        api.lifecycle_localization_deactivate_node()
        api.lifecycle_pathplanner_deactivate_node()
        print('>>> EXECUTING TRANSFORM ... AND GOING TO THE CART')
        robot_movement.enable_control()
        robot_movement.wait_for_movement_completion()
        robot_elevator.publish_up()
        robot_footprint.update_footprint_cart()
        robot_movement.move_backward(move_time=8)
        robot_movement.spin(spin_time=11, turn_factor=-1)
        robot_movement.move_forward(move_time=2)
        api.lifecycle_localization_activate_node()
        api.lifecycle_pathplanner_activate_node()
        sleep(8)
        # step2
        api.go_to(*middle_position)
        api.go_to(*shipping_position)
        robot_elevator.publish_down()
        robot_footprint.update_footprint_normal()
        robot_movement.move_backward(move_time=6)
        # step3
        api.go_to(*init_position)
    else:
        print('>>> IMPROVING LOCALIZATION')
        robot_movement.spin(spin_time=24, turn_factor=-1)
        # step1
        api.go_to(*loading_position)
        print('>>> EXECUTING TRANSFORM ... AND GOING TO THE CART')
        robot_movement.enable_control()
        robot_movement.wait_for_movement_completion()
        robot_elevator.publish_up()
        robot_footprint.update_footprint_cart()
        robot_movement.move_backward(move_time=6)
        robot_movement.spin(spin_time=8, turn_factor=-1)
        robot_movement.move_forward(move_time=1.7)
        # step2
        api.go_to(*middle_position)
        api.go_to(*shipping_position)
        robot_elevator.publish_down()
        robot_footprint.update_footprint_normal()
        robot_movement.move_backward(move_time=6)
        # step3
        api.go_to(*init_position)
        
'''
python3 ros2_ws/src/warehouse_project/nav2_apps/src/move_shelf_to_ship.py --use_sim_time False
'''
