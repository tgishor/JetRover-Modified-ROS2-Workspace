#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
import tf2_ros
from tf2_ros import TransformListener, Buffer
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FormationController(Node):
    """
    Formation controller for follower robot.
    Maintains formation by following the leader at a specified distance while avoiding obstacles.
    """

    def __init__(self):
        super().__init__('formation_controller')
        
        # Parameters
        self.declare_parameter('robot_namespace', 'follower')
        self.declare_parameter('leader_namespace', 'leader')
        self.declare_parameter('formation_distance', 0.5)  # 50cm behind leader
        self.declare_parameter('formation_angle', 3.14159)  # π radians (180°) - directly behind
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('distance_tolerance', 0.1)
        self.declare_parameter('angle_tolerance', 0.2)
        self.declare_parameter('obstacle_distance_threshold', 0.5)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('control_frequency', 10.0)
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.leader_namespace = self.get_parameter('leader_namespace').get_parameter_value().string_value
        self.formation_distance = self.get_parameter('formation_distance').get_parameter_value().double_value
        self.formation_angle = self.get_parameter('formation_angle').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.angle_tolerance = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_distance_threshold').get_parameter_value().double_value
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').get_parameter_value().bool_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.current_pose = None
        self.leader_pose = None
        self.laser_data = None
        self.formation_active = False
        self.last_tag_time = None
        self.tag_lost_timeout = 2.0  # seconds
        
        # PID controllers for formation keeping
        self.distance_pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
        self.angle_pid = PIDController(kp=2.0, ki=0.1, kd=0.1)
        
        # QoS profiles
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_namespace}/estimated_pose',
            self.pose_callback,
            10
        )
        
        self.tag_pose_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_namespace}/leader_tag_pose',
            self.tag_pose_callback,
            10
        )
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_namespace}/scan',
            self.laser_callback,
            qos_profile
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_namespace}/cmd_vel',
            10
        )
        
        self.formation_goal_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_namespace}/formation_goal',
            10
        )
        
        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, 
            self.control_loop
        )
        
        self.get_logger().info(f'Formation controller initialized for {self.robot_namespace}')
        self.get_logger().info(f'Target formation: {self.formation_distance}m at {math.degrees(self.formation_angle)}°')

    def pose_callback(self, msg):
        """Update current robot pose."""
        self.current_pose = msg

    def tag_pose_callback(self, msg):
        """Handle AprilTag detection - indicates leader is visible."""
        self.last_tag_time = self.get_clock().now()
        self.formation_active = True

    def laser_callback(self, msg):
        """Update laser scan data for obstacle avoidance."""
        self.laser_data = msg

    def control_loop(self):
        """Main control loop for formation keeping."""
        if not self.formation_active or self.current_pose is None:
            return
            
        # Check if tag is lost
        if self.last_tag_time is not None:
            time_since_tag = (self.get_clock().now() - self.last_tag_time).nanoseconds / 1e9
            if time_since_tag > self.tag_lost_timeout:
                self.handle_tag_lost()
                return
        
        try:
            # Get leader position
            leader_pose = self.get_leader_pose()
            if leader_pose is None:
                return
            
            # Calculate desired formation position
            formation_goal = self.calculate_formation_goal(leader_pose)
            if formation_goal is None:
                return
            
            # Publish formation goal for visualization
            self.formation_goal_pub.publish(formation_goal)
            
            # Calculate control commands
            cmd_vel = self.calculate_control_commands(formation_goal)
            
            # Apply obstacle avoidance if enabled
            if self.enable_obstacle_avoidance and self.laser_data is not None:
                cmd_vel = self.apply_obstacle_avoidance(cmd_vel)
            
            # Publish commands
            self.cmd_vel_pub.publish(cmd_vel)
            
        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')

    def get_leader_pose(self):
        """Get leader's current pose."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.leader_namespace}/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            return None

    def calculate_formation_goal(self, leader_pose):
        """Calculate the desired formation position relative to leader."""
        try:
            # Get leader's orientation
            leader_yaw = self.quaternion_to_yaw(leader_pose.pose.orientation)
            
            # Calculate formation position
            formation_yaw = leader_yaw + self.formation_angle
            
            goal_x = leader_pose.pose.position.x + self.formation_distance * math.cos(formation_yaw)
            goal_y = leader_pose.pose.position.y + self.formation_distance * math.sin(formation_yaw)
            
            # Create goal pose
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0.0
            
            # Follower should face toward leader
            desired_yaw = math.atan2(
                leader_pose.pose.position.y - goal_y,
                leader_pose.pose.position.x - goal_x
            )
            
            quat = self.yaw_to_quaternion(desired_yaw)
            goal.pose.orientation.x = quat[0]
            goal.pose.orientation.y = quat[1]
            goal.pose.orientation.z = quat[2]
            goal.pose.orientation.w = quat[3]
            
            return goal
            
        except Exception as e:
            self.get_logger().error(f'Error calculating formation goal: {str(e)}')
            return None

    def calculate_control_commands(self, goal):
        """Calculate velocity commands to reach formation goal."""
        cmd = Twist()
        
        # Calculate errors
        dx = goal.pose.position.x - self.current_pose.pose.position.x
        dy = goal.pose.position.y - self.current_pose.pose.position.y
        distance_error = math.sqrt(dx*dx + dy*dy)
        
        current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
        goal_yaw = self.quaternion_to_yaw(goal.pose.orientation)
        angle_error = self.normalize_angle(goal_yaw - current_yaw)
        
        # Calculate angle to goal
        angle_to_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_goal - current_yaw)
        
        # Control logic
        if distance_error > self.distance_tolerance:
            # Move toward goal position
            linear_vel = self.distance_pid.update(distance_error)
            angular_vel = self.angle_pid.update(heading_error)
            
            # If we need to turn significantly, reduce linear velocity
            if abs(heading_error) > self.angle_tolerance:
                linear_vel *= (1.0 - abs(heading_error) / math.pi)
                
        else:
            # At position, adjust orientation
            linear_vel = 0.0
            angular_vel = self.angle_pid.update(angle_error)
        
        # Apply velocity limits
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        cmd.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        return cmd

    def apply_obstacle_avoidance(self, cmd_vel):
        """Apply obstacle avoidance to the velocity commands."""
        if self.laser_data is None:
            return cmd_vel
        
        # Find minimum distance in front sectors
        ranges = np.array(self.laser_data.ranges)
        ranges = np.where(np.isinf(ranges), self.laser_data.range_max, ranges)
        
        # Check front, left-front, and right-front sectors
        n_ranges = len(ranges)
        front_sector = ranges[n_ranges//2 - 30:n_ranges//2 + 30]  # ~60° front sector
        left_sector = ranges[:60]  # Left side
        right_sector = ranges[-60:]  # Right side
        
        min_front_dist = np.min(front_sector) if len(front_sector) > 0 else float('inf')
        min_left_dist = np.min(left_sector) if len(left_sector) > 0 else float('inf')
        min_right_dist = np.min(right_sector) if len(right_sector) > 0 else float('inf')
        
        # Apply avoidance behavior
        if min_front_dist < self.obstacle_threshold:
            # Obstacle in front - stop or slow down
            cmd_vel.linear.x *= max(0.0, (min_front_dist - 0.2) / (self.obstacle_threshold - 0.2))
            
            # Turn away from obstacle
            if min_left_dist > min_right_dist:
                cmd_vel.angular.z += 0.5  # Turn left
            else:
                cmd_vel.angular.z -= 0.5  # Turn right
        
        # Side obstacle avoidance
        if min_left_dist < self.obstacle_threshold * 0.7:
            cmd_vel.angular.z -= 0.3  # Turn right
        elif min_right_dist < self.obstacle_threshold * 0.7:
            cmd_vel.angular.z += 0.3  # Turn left
        
        return cmd_vel

    def handle_tag_lost(self):
        """Handle situation when AprilTag is lost."""
        self.get_logger().warn('AprilTag lost - switching to recovery behavior')
        
        # Stop the robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Could implement search behavior here
        # For now, just stop and wait
        self.formation_active = False

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class PIDController:
    """Simple PID controller."""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def update(self, error, dt=0.1):
        """Update PID controller with current error."""
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        if self.max_output is not None:
            output = max(-self.max_output, min(self.max_output, output))
        
        # Update for next iteration
        self.prev_error = error
        
        return output

    def reset(self):
        """Reset PID controller state."""
        self.prev_error = 0.0
        self.integral = 0.0


def main(args=None):
    rclpy.init(args=args)
    
    node = FormationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
