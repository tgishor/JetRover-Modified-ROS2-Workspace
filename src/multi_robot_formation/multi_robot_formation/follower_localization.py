#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
import numpy as np
import math


class FollowerLocalization(Node):
    """
    Follower robot localization node.
    Uses AprilTag detections to determine follower's position in the global map frame.
    """

    def __init__(self):
        super().__init__('follower_localization')
        
        # Parameters
        self.declare_parameter('robot_namespace', 'follower')
        self.declare_parameter('leader_namespace', 'leader')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('tag_to_robot_offset_x', -0.20)  # Tag is 20cm behind robot center
        self.declare_parameter('tag_to_robot_offset_y', 0.0)
        self.declare_parameter('tag_to_robot_offset_z', 0.15)   # Tag height above ground
        self.declare_parameter('localization_confidence_threshold', 0.1)
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.leader_namespace = self.get_parameter('leader_namespace').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        
        # Tag offset from robot center (tag position relative to robot base)
        self.tag_offset_x = self.get_parameter('tag_to_robot_offset_x').get_parameter_value().double_value
        self.tag_offset_y = self.get_parameter('tag_to_robot_offset_y').get_parameter_value().double_value
        self.tag_offset_z = self.get_parameter('tag_to_robot_offset_z').get_parameter_value().double_value
        
        self.confidence_threshold = self.get_parameter('localization_confidence_threshold').get_parameter_value().double_value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # State variables
        self.is_localized = False
        self.last_tag_detection = None
        self.leader_pose_in_map = None
        
        # Clean namespaces (remove leading/trailing slashes)
        clean_robot_ns = self.robot_namespace.strip('/')
        clean_leader_ns = self.leader_namespace.strip('/')
        
        # Subscribers
        self.tag_pose_sub = self.create_subscription(
            PoseStamped,
            f'/{clean_robot_ns}/leader_tag_pose',
            self.tag_pose_callback,
            10
        )
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{clean_robot_ns}/initialpose',
            10
        )
        
        self.follower_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{clean_robot_ns}/estimated_pose',
            10
        )
        
        # Timer for periodic localization updates
        self.timer = self.create_timer(0.1, self.update_localization)  # 10Hz
        
        self.get_logger().info(f'Follower localization initialized for {self.robot_namespace}')
        self.get_logger().info(f'Looking for leader: {self.leader_namespace}')

    def tag_pose_callback(self, msg):
        """Handle incoming AprilTag pose detections."""
        self.last_tag_detection = msg
        self.get_logger().debug('Received tag detection')

    def update_localization(self):
        """Main localization update loop."""
        if self.last_tag_detection is None:
            return
            
        try:
            # Get leader's current pose in the global map
            leader_pose = self.get_leader_pose_in_map()
            if leader_pose is None:
                return
                
            # Calculate follower's pose based on tag detection
            follower_pose = self.calculate_follower_pose(self.last_tag_detection, leader_pose)
            if follower_pose is None:
                return
                
            # Publish the estimated pose
            self.follower_pose_pub.publish(follower_pose)
            
            # If not yet localized, publish initial pose for AMCL
            if not self.is_localized:
                self.publish_initial_pose(follower_pose)
                self.is_localized = True
                self.get_logger().info('Follower robot localized successfully!')
                
        except Exception as e:
            self.get_logger().error(f'Error in localization update: {str(e)}')

    def get_leader_pose_in_map(self):
        """Get the leader robot's current pose in the global map frame."""
        try:
            # Look up transform from map to leader's base frame
            clean_leader_ns = self.leader_namespace.strip('/')
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                f'{clean_leader_ns}/{self.base_frame}',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Convert transform to pose
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'Could not get leader pose: {str(e)}')
            return None

    def calculate_follower_pose(self, tag_detection, leader_pose):
        """Calculate follower's pose in the global map based on tag detection and leader pose."""
        try:
            # Step 1: Transform tag pose from camera frame to follower's base frame
            tag_in_base = self.transform_pose_to_base_frame(tag_detection)
            if tag_in_base is None:
                return None
            
            # Step 2: Calculate leader's actual position from tag position
            # The tag is offset from the leader's center
            leader_actual_pos = self.calculate_leader_position_from_tag(
                tag_in_base, leader_pose
            )
            
            # Step 3: Calculate follower's position relative to leader
            follower_pose = self.calculate_follower_global_pose(
                leader_actual_pos, tag_in_base
            )
            
            return follower_pose
            
        except Exception as e:
            self.get_logger().error(f'Error calculating follower pose: {str(e)}')
            return None

    def transform_pose_to_base_frame(self, tag_pose):
        """Transform tag pose from camera frame to robot base frame."""
        try:
            # Look up transform from base to camera
            clean_robot_ns = self.robot_namespace.strip('/')
            transform = self.tf_buffer.lookup_transform(
                f'{clean_robot_ns}/{self.base_frame}',
                tag_pose.header.frame_id,
                tag_pose.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Transform the pose
            tag_in_base = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)
            return tag_in_base
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'Could not transform tag pose: {str(e)}')
            return None

    def calculate_leader_position_from_tag(self, tag_in_base, leader_pose_in_map):
        """Calculate the actual leader robot position from the detected tag position."""
        # The tag is mounted with a known offset from the robot center
        # We need to account for this offset to get the true robot position
        
        # Get leader's orientation in map frame
        leader_yaw = self.quaternion_to_yaw(leader_pose_in_map.pose.orientation)
        
        # Calculate the actual leader position accounting for tag offset
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        
        # Transform tag offset to map frame
        leader_x = leader_pose_in_map.pose.position.x
        leader_y = leader_pose_in_map.pose.position.y
        
        return [leader_x, leader_y, leader_yaw]

    def calculate_follower_global_pose(self, leader_position, tag_in_base):
        """Calculate follower's global pose based on detected tag and leader position."""
        leader_x, leader_y, leader_yaw = leader_position
        
        # Tag position relative to follower base
        tag_x = tag_in_base.pose.position.x
        tag_y = tag_in_base.pose.position.y
        
        # Calculate tag's global position (where follower sees the tag)
        # The tag is at the leader's position plus offset
        cos_leader = math.cos(leader_yaw)
        sin_leader = math.sin(leader_yaw)
        
        tag_global_x = leader_x + (self.tag_offset_x * cos_leader - self.tag_offset_y * sin_leader)
        tag_global_y = leader_y + (self.tag_offset_x * sin_leader + self.tag_offset_y * cos_leader)
        
        # Now calculate follower's position
        # If tag is at tag_global_x,y and follower sees it at tag_x,y relative to its base,
        # then follower is at tag_global - tag_relative_to_follower
        
        # We need to account for follower's orientation
        # Estimate follower orientation from tag detection
        tag_angle = math.atan2(tag_y, tag_x)
        follower_yaw = leader_yaw + math.pi - tag_angle  # Approximately facing toward leader
        
        cos_follower = math.cos(follower_yaw)
        sin_follower = math.sin(follower_yaw)
        
        # Calculate follower position
        follower_x = tag_global_x - (tag_x * cos_follower - tag_y * sin_follower)
        follower_y = tag_global_y - (tag_x * sin_follower + tag_y * cos_follower)
        
        # Create pose message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = follower_x
        pose.pose.position.y = follower_y
        pose.pose.position.z = 0.0
        
        # Set orientation
        quat = self.yaw_to_quaternion(follower_yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose

    def publish_initial_pose(self, pose):
        """Publish initial pose for AMCL localization."""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header = pose.header
        initial_pose.pose.pose = pose.pose
        
        # Set covariance (diagonal matrix with reasonable uncertainty)
        covariance = [0.0] * 36
        covariance[0] = 0.25   # x variance
        covariance[7] = 0.25   # y variance
        covariance[35] = 0.1   # yaw variance
        initial_pose.pose.covariance = covariance
        
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info(f'Published initial pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]  # [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    
    node = FollowerLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
