#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """
    Launch AprilTag alignment system.
    Robot finds tag, moves to 2m distance, centers itself.
    """
    
    # Get package directory
    formation_package = get_package_share_directory('multi_robot_formation')
    
    # Get robot name from environment
    try:
        robot_name = os.environ['HOST'].strip('/')
    except KeyError:
        robot_name = 'robot_1'
        print(f"HOST not set, using default: {robot_name}")
    
    # Get launch configurations
    target_distance = LaunchConfiguration('target_distance', default='2.0').perform(context)
    tag_id = LaunchConfiguration('tag_id', default='2').perform(context)
    
    print(f"🤖 AprilTag Alignment Starting:")
    print(f"   Robot: {robot_name}")
    print(f"   Target distance: {target_distance}m")
    print(f"   Tag ID: {tag_id}")
    
    # Launch nodes
    nodes = [
        # AprilTag detector
        Node(
            package='multi_robot_formation',
            executable='apriltag_detector',
            name='apriltag_detector',
            namespace=robot_name,
            parameters=[
                os.path.join(formation_package, 'config', 'formation_params.yaml'),
                {
                    'robot_namespace': robot_name,
                    'leader_tag_id': int(tag_id),
                    'camera_frame': f'{robot_name}/camera_link'
                }
            ],
            output='screen'
        ),
        
        # Alignment controller
        Node(
            package='multi_robot_formation',
            executable='apriltag_alignment_controller',
            name='apriltag_alignment_controller',
            namespace=robot_name,
            parameters=[
                {
                    'robot_namespace': robot_name,
                    'target_distance': float(target_distance),
                    'distance_tolerance': 0.05,  # 5cm
                    'center_tolerance': 0.02,    # 2cm
                    'max_linear_velocity': 0.2,
                    'max_angular_velocity': 0.5
                }
            ],
            output='screen'
        )
    ]
    
    return nodes


def generate_launch_description():
    """Generate launch description for AprilTag alignment."""
    
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='2.0',
        description='Target distance to maintain from AprilTag (meters)'
    )
    
    tag_id_arg = DeclareLaunchArgument(
        'tag_id',
        default_value='2',
        description='AprilTag ID to track'
    )
    
    return LaunchDescription([
        target_distance_arg,
        tag_id_arg,
        
        # Start alignment system
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=launch_setup)]
        )
    ])
