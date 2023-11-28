import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
#essential libraries for rviz
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare the launch argument
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True',
        description='Use simulation (Gazebo) clock if true')
    map_file_arg = DeclareLaunchArgument('map_file',default_value='warehouse_map_sim.yaml',
        description='Map file to provide')
    package_name_arg = DeclareLaunchArgument("package_name", default_value="map_server")
    rviz_config_file_arg= DeclareLaunchArgument("rviz_config_file", default_value="config1.rviz")
    # Use the launch argument in the node configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    map_dir = os.path.join(get_package_share_directory('map_server'), 'config/')
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
        map_file_arg,
        package_name_arg,
        rviz_config_file_arg,

        start_rviz_launch,
        LogInfo(msg='----------------------waiting for RVIZ------------------------'),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}, 
                                {'yaml_filename':[map_dir, map_file]} 
                               ]),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_mapper',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': True},
                                {'node_names': ['map_server']}]) 
            ]
        )           
    ])