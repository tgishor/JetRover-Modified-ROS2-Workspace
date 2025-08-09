#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    """
    Launch file for multi-robot formation system.
    Starts both leader and follower robots with formation control.
    """
    
    # Get package directories
    formation_package = get_package_share_directory('multi_robot_formation')
    navigation_package = get_package_share_directory('navigation')
    slam_package = get_package_share_directory('slam')
    
    # Declare launch arguments
    leader_name_arg = DeclareLaunchArgument(
        'leader_name',
        default_value='leader',
        description='Namespace for leader robot'
    )
    
    follower_name_arg = DeclareLaunchArgument(
        'follower_name', 
        default_value='follower',
        description='Namespace for follower robot'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(slam_package, 'maps', 'map_01.yaml'),
        description='Path to map file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    leader_name = LaunchConfiguration('leader_name')
    follower_name = LaunchConfiguration('follower_name')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Leader robot launch (SLAM mode for mapping)
    leader_group = GroupAction([
        PushRosNamespace(leader_name),
        
        # Leader robot base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_package, 'launch', 'include', 'robot.launch.py')
            ),
            launch_arguments={
                'robot_name': leader_name,
                'master_name': leader_name,
                'sim': 'false'
            }.items()
        ),
        
        # Leader SLAM
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_package, 'launch', 'include', 'slam_base.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'map_frame': 'leader/map',
                        'odom_frame': 'leader/odom', 
                        'base_frame': 'leader/base_footprint',
                        'scan_topic': 'leader/scan',
                        'enable_save': 'true'
                    }.items()
                )
            ]
        )
    ])
    
    # Follower robot launch (Navigation mode using leader's map)
    follower_group = GroupAction([
        PushRosNamespace(follower_name),
        
        # Follower robot base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_package, 'launch', 'include', 'robot.launch.py')
            ),
            launch_arguments={
                'robot_name': follower_name,
                'master_name': leader_name,  # Use leader as master
                'sim': 'false'
            }.items()
        ),
        
        # Follower navigation (delayed to ensure leader map is available)
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(navigation_package, 'launch', 'include', 'bringup.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'map': map_file,
                        'params_file': os.path.join(navigation_package, 'config', 'nav2_params.yaml'),
                        'namespace': follower_name,
                        'use_namespace': 'true',
                        'autostart': 'true',
                        'use_teb': 'true'
                    }.items()
                )
            ]
        )
    ])
    
    # Formation control nodes
    formation_nodes = [
        # AprilTag detector
        Node(
            package='multi_robot_formation',
            executable='apriltag_detector',
            name='apriltag_detector',
            namespace=follower_name,
            parameters=[
                os.path.join(formation_package, 'config', 'formation_params.yaml'),
                {
                    'robot_namespace': follower_name,
                    'camera_frame': f'{follower_name}/camera_link'
                }
            ],
            output='screen'
        ),
        
        # Follower localization
        Node(
            package='multi_robot_formation',
            executable='follower_localization', 
            name='follower_localization',
            namespace=follower_name,
            parameters=[
                os.path.join(formation_package, 'config', 'formation_params.yaml'),
                {
                    'robot_namespace': follower_name,
                    'leader_namespace': leader_name,
                    'global_frame': 'map',
                    'base_frame': f'{follower_name}/base_footprint',
                    'camera_frame': f'{follower_name}/camera_link'
                }
            ],
            output='screen'
        ),
        
        # Formation controller
        Node(
            package='multi_robot_formation',
            executable='formation_controller',
            name='formation_controller', 
            namespace=follower_name,
            parameters=[
                os.path.join(formation_package, 'config', 'formation_params.yaml'),
                {
                    'robot_namespace': follower_name,
                    'leader_namespace': leader_name
                }
            ],
            output='screen'
        )
    ]
    
    # Static transforms for AprilTag
    apriltag_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='leader_apriltag_tf',
        arguments=[
            '0', '0', '0.15',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw
            f'{leader_name}/base_footprint',
            f'{leader_name}/apriltag_link'
        ]
    )
    
    return LaunchDescription([
        leader_name_arg,
        follower_name_arg,
        map_file_arg,
        use_sim_time_arg,
        
        leader_group,
        follower_group,
        apriltag_tf,
        
        # Start formation control after robots are initialized
        TimerAction(
            period=20.0,
            actions=formation_nodes
        )
    ])
