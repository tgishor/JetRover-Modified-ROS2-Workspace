#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """
    Launch formation control using ROS apriltag_ros package.
    This is more reliable than the Python apriltag library.
    """
    
    # Get package directory
    formation_package = get_package_share_directory('multi_robot_formation')
    
    # Get robot names from environment variables
    try:
        follower_name = os.environ['HOST'].strip('/')
        leader_name = os.environ['MASTER'].strip('/')
    except KeyError as e:
        print(f"ERROR: Environment variable {e} not set!")
        return []
    
    # Get launch configurations
    tag_id = LaunchConfiguration('tag_id', default='0').perform(context)
    formation_distance = LaunchConfiguration('formation_distance', default='0.5').perform(context)
    
    print(f"🤖 Formation Control Starting (ROS AprilTag):")
    print(f"   Follower: {follower_name}")
    print(f"   Leader: {leader_name}")
    print(f"   Distance: {formation_distance}m")
    print(f"   Tag ID: {tag_id}")
    
    # AprilTag ROS node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace=follower_name,
        parameters=[{
            'image_transport': 'raw',
            'family': 'tag36h11',
            'size': 0.10,  # 10cm tag size
            'max_hamming': 0,
            'z_up': True,
        }],
        remappings=[
            ('image_rect', f'/{follower_name}/camera/image_raw'),
            ('camera_info', f'/{follower_name}/camera/camera_info'),
        ],
        output='screen'
    )
    
    # Formation control nodes
    formation_nodes = [
        apriltag_node,
        
        # AprilTag detector (will use ROS detections)
        Node(
            package='multi_robot_formation',
            executable='apriltag_detector',
            name='apriltag_detector',
            namespace=follower_name,
            parameters=[
                os.path.join(formation_package, 'config', 'formation_params.yaml'),
                {
                    'robot_namespace': follower_name,
                    'leader_tag_id': int(tag_id),
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
                    'leader_namespace': leader_name,
                    'formation_distance': float(formation_distance)
                }
            ],
            output='screen'
        )
    ]
    
    return formation_nodes


def generate_launch_description():
    """Generate launch description for ROS apriltag-based formation control."""
    
    tag_id_arg = DeclareLaunchArgument(
        'tag_id',
        default_value='0',
        description='AprilTag ID on leader robot'
    )
    
    formation_distance_arg = DeclareLaunchArgument(
        'formation_distance',
        default_value='0.5',
        description='Formation distance in meters'
    )
    
    return LaunchDescription([
        tag_id_arg,
        formation_distance_arg,
        
        TimerAction(
            period=2.0,
            actions=[OpaqueFunction(function=launch_setup)]
        )
    ])
