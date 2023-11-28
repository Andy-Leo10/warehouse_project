# warehouse_project

## Mandatory
+ Start the simulation in ROS1
```
source ~/simulation_ws/devel/setup.bash
roslaunch rb1_base_gazebo warehouse_rb1.launch
```
+ Start the ROS1 bridge
```
source ~/catkin_ws/devel/setup.bash
roslaunch load_params load_params_base.launch
source /opt/ros/galactic/setup.bash
ros2 run ros1_bridge parameter_bridge
```

## 1 Mapping Launch files
- [x] Start mapping
```
ros2 launch cartographer_slam cartographer.launch.py
ros2 run nav2_map_server map_saver_cli -f NAME
```
- [x] Providing map
```
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_sim.yaml
ros2 launch map_server map_server.launch.py map_file:=warehouse_map_real.yaml
```

## 2 Localization Launch files
- [x] Start localization
```
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

## 3 Navigation Launch files
- [x] Pre approach
```
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
ros2 launch path_planner_server pathplanner.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```

## Others
```
ros2 run tf2_tools view_frames
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel
```