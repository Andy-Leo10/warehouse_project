import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
#essential libraries for rviz
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
#conditions
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    controller_yaml_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml_1 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    controller_yaml_2 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller2.yaml')
    bt_navigator_yaml_2 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator2.yaml')
    planner_yaml_2 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server2.yaml')
    recovery_yaml_2 = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery2.yaml')
    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    package_name_arg = DeclareLaunchArgument("package_name", default_value="path_planner_server")
    rviz_config_file_arg= DeclareLaunchArgument("rviz_config_file", default_value="config1.rviz")
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = LaunchConfiguration("package_name")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    
    #rviz
    start_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_name),
                'launch',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={
            'package_description': package_name,
            'rviz_config_file_name': rviz_config_file,
            'use_sim_time': use_sim_time
            }.items()
    )

    return LaunchDescription([
        # Include the launch argument in the launch description
        use_sim_time_arg,
        #package_name_arg,
        #rviz_config_file_arg,

        #start_rviz_launch,
        #LogInfo(msg='----------------------waiting for RVIZ------------------------'),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml_1,
                                {'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[planner_yaml_2,
                                {'use_sim_time': use_sim_time}],
                    condition=UnlessCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                #----------------------------------------------#
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml_1,
                                {'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_sim_time),                                
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[controller_yaml_2,
                                {'use_sim_time': use_sim_time}],
                    condition=UnlessCondition(use_sim_time),                                
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                #----------------------------------------------#
                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    output='screen',
                    parameters=[recovery_yaml_1,
                                {'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    output='screen',
                    parameters=[recovery_yaml_2,
                                {'use_sim_time': use_sim_time}],
                    condition=UnlessCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                #----------------------------------------------#
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml_1,
                                {'use_sim_time': use_sim_time}],
                    condition=IfCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[bt_navigator_yaml_2,
                                {'use_sim_time': use_sim_time}],
                    condition=UnlessCondition(use_sim_time), 
                    remappings=[('/cmd_vel', '/robot/cmd_vel')],
                    ),
                #----------------------------------------------#
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_pathplanner',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': True},
                                {'node_names': ['planner_server',
                                                'controller_server',
                                                'recoveries_server',
                                                'bt_navigator']}])
            ]
        )

    ])
