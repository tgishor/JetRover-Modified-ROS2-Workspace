#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for follower robot formation control only.
    Use this when leader robot is already running.
    """
    
    # Get package directories
    formation_package = get_package_share_directory('multi_robot_formation')
    
    # Declare launch arguments
    follower_name_arg = DeclareLaunchArgument(
        'follower_name',
        default_value='follower',
        description='Namespace for follower robot'
    )
    
    leader_name_arg = DeclareLaunchArgument(
        'leader_name',
        default_value='leader', 
        description='Namespace for leader robot'
    )
    
    formation_distance_arg = DeclareLaunchArgument(
        'formation_distance',
        default_value='0.5',
        description='Formation distance in meters'
    )
    
    tag_id_arg = DeclareLaunchArgument(
        'tag_id',
        default_value='0',
        description='AprilTag ID on leader robot'
    )
    
    # Get launch configurations
    follower_name = LaunchConfiguration('follower_name')
    leader_name = LaunchConfiguration('leader_name')
    formation_distance = LaunchConfiguration('formation_distance')
    tag_id = LaunchConfiguration('tag_id')
    
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
                    'leader_tag_id': tag_id,
                    'camera_frame': [follower_name, '/camera_link']
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
                    'base_frame': [follower_name, '/base_footprint'],
                    'camera_frame': [follower_name, '/camera_link']
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
                    'leader_namespace': leader_name,
                    'formation_distance': formation_distance
                }
            ],
            output='screen'
        )
    ]
    
    return LaunchDescription([
        follower_name_arg,
        leader_name_arg,
        formation_distance_arg,
        tag_id_arg,
        
        # Start formation control nodes
        TimerAction(
            period=2.0,  # Small delay to ensure robot is ready
            actions=formation_nodes
        )
    ])
