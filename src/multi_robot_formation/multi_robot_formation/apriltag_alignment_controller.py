#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import math
import signal
import sys

class AprilTagAlignmentController(Node):
    """
    Controller to align robot with AprilTag:
    1. Find AprilTag
    2. Move to 2m distance 
    3. Center alignment (X = 0)
    """

    def __init__(self):
        super().__init__('apriltag_alignment_controller')
        
        # Parameters
        self.declare_parameter('target_distance', 2.0)  # 2 meters
        self.declare_parameter('distance_tolerance', 0.05)  # 5cm tolerance
        self.declare_parameter('center_tolerance', 0.005)   # 5mm ultra-precise centering tolerance
        self.declare_parameter('max_linear_velocity', 0.1)   # Fixed speed - no variable control
        self.declare_parameter('max_angular_velocity', 0.1) # Much slower turn speed
        self.declare_parameter('robot_namespace', 'robot_1')
        
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.distance_tolerance = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.center_tolerance = self.get_parameter('center_tolerance').get_parameter_value().double_value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # State variables
        self.tag_pose = None
        self.is_aligned = False
        self.alignment_state = "SEARCHING"  # SEARCHING, STABILIZING, MOVING_TO_DISTANCE, CENTERING, ALIGNED
        self.log_counter = 0  # Counter for periodic logging
        self.movement_complete = False  # Flag to stop all movement when target reached
        
        # Distance smoothing to reduce noise
        self.distance_history = []
        self.max_history = 5  # Keep last 5 readings for smoothing
        
        # X-position smoothing for ultra-precise centering
        self.x_history = []
        self.max_x_history = 10  # More smoothing for precise centering
        
        # Stabilization timer - wait 3 seconds before moving
        self.stabilization_start_time = None
        self.stabilization_duration = 3.0  # 3 seconds
        
        # PID controllers - Much gentler gains
        self.distance_kp = 0.15  # Reduced from 0.8 for smoother movement
        self.center_kp = 0.3     # Reduced from 1.5 for smoother centering
        
        # Clean namespace
        clean_namespace = self.robot_namespace.strip('/')
        
        # Subscribers
        self.tag_pose_sub = self.create_subscription(
            PoseStamped,
            f'/{clean_namespace}/leader_tag_pose',
            self.tag_pose_callback,
            10
        )
        
        # Publishers (match Mecanum robot controller topic)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{clean_namespace}/controller/cmd_vel',  # Match your teleop topic
            10
        )
        
        self.alignment_status_pub = self.create_publisher(
            Bool,
            f'/{clean_namespace}/alignment_complete',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info(f'🎯 AprilTag Alignment Controller Started')
        self.get_logger().info(f'📏 Target distance: {self.target_distance}m')
        self.get_logger().info(f'⚖️ Center tolerance: ±{self.center_tolerance*1000:.1f}mm (ultra-precise)')
        self.get_logger().info(f'🔍 State: {self.alignment_state}')
        self.get_logger().info('🛑 Press Ctrl+C for emergency stop')

    def tag_pose_callback(self, msg):
        """Receive AprilTag pose data."""
        self.tag_pose = msg
        
        # Extract position data
        x = msg.pose.position.x
        y = msg.pose.position.y  
        z = msg.pose.position.z
        distance = math.sqrt(x*x + y*y + z*z)
        
        self.get_logger().debug(
            f'📍 Tag position: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, Distance={distance:.3f}m'
        )

    def smooth_distance(self, new_distance):
        """Smooth distance measurements to reduce noise and oscillation."""
        # Add new measurement to history
        self.distance_history.append(new_distance)
        
        # Keep only recent measurements
        if len(self.distance_history) > self.max_history:
            self.distance_history.pop(0)
        
        # Return moving average
        return sum(self.distance_history) / len(self.distance_history)

    def smooth_x_position(self, new_x):
        """Smooth X position measurements for ultra-precise centering."""
        # Add new measurement to history
        self.x_history.append(new_x)
        
        # Keep only recent measurements
        if len(self.x_history) > self.max_x_history:
            self.x_history.pop(0)
        
        # Return moving average for precise centering
        return sum(self.x_history) / len(self.x_history)

    def control_loop(self):
        """Main control loop for alignment."""
        # Check if we have tag data
        if self.tag_pose is None:
            # No tag detected - stay in SEARCHING state and don't move
            if self.alignment_state != "SEARCHING":
                self.alignment_state = "SEARCHING"
                self.get_logger().info("🏷️ Tag lost! Returning to SEARCHING state...")
            
            # Send explicit stop command and return
            self.stop_robot()
            
            # Debug logging every 2 seconds
            if self.log_counter % 20 == 0:  # Every 2 seconds
                self.get_logger().info("🔍 SEARCHING - Robot stationary, waiting for AprilTag detection...")
            self.log_counter += 1
            return
        
        # We have a tag - get current position
        raw_x = self.tag_pose.pose.position.x
        z = self.tag_pose.pose.position.z
        raw_distance = math.sqrt(raw_x*raw_x + z*z)  # Use X-Z plane distance for alignment
        
        # Smooth both distance and X position for ultra-precision
        distance = self.smooth_distance(raw_distance)
        x = self.smooth_x_position(raw_x)  # Ultra-smooth X for precise centering
        
        # State machine for alignment (we know tag_pose is not None here)
        cmd_vel = Twist()
        
        if self.alignment_state == "SEARCHING":
            # Tag found! Transition to stabilization phase
            self.alignment_state = "STABILIZING"
            self.movement_complete = False
            self.stabilization_start_time = self.get_clock().now()
            self.get_logger().info(f"🔍➡️⏱️ Tag found! Current distance: {distance:.3f}m - Stabilizing for 3 seconds before moving...")
            # Don't send any movement commands during stabilization
            cmd_vel = Twist()  # Stay still during stabilization
            
        elif self.alignment_state == "STABILIZING":
            # Wait for 3 seconds before starting to move
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.stabilization_start_time).nanoseconds / 1e9
            
            remaining_time = self.stabilization_duration - elapsed_time
            
            if remaining_time > 0:
                # Still stabilizing - stay still and show countdown
                cmd_vel = Twist()  # Complete stop
                if self.log_counter % 10 == 0:  # Update every 1 second
                    self.get_logger().info(f"⏱️ Stabilizing... {remaining_time:.1f}s remaining (Current: {distance:.3f}m, Target: {self.target_distance}m)")
            else:
                # Stabilization complete - start moving
                self.alignment_state = "MOVING_TO_DISTANCE"
                self.get_logger().info(f"⏱️➡️🎯 Stabilization complete! Starting movement to {self.target_distance}m...")
                cmd_vel = Twist()  # One more cycle of stillness before starting
            
        elif self.alignment_state == "MOVING_TO_DISTANCE":
            distance_error = abs(distance - self.target_distance)
            
            # Check if we've reached the target distance
            if distance_error < self.distance_tolerance:
                # Distance reached - now start centering
                self.alignment_state = "CENTERING"
                cmd_vel = Twist()  # Stop for one cycle before centering
                self.get_logger().info(f"🎯➡️⚖️ Distance reached! Starting centering... (X={x:+.3f}m)")
            else:
                # Move towards target distance
                cmd_vel = self.move_to_distance(distance)
                
        elif self.alignment_state == "CENTERING":
            center_error = abs(x)
            distance_error = abs(distance - self.target_distance)
            
            # Check if we're centered
            if center_error < self.center_tolerance:
                # Centered! Check if distance is still good
                if distance_error < self.distance_tolerance:
                    self.alignment_state = "ALIGNED"
                    self.movement_complete = True
                    cmd_vel = Twist()  # Complete stop
                    x_mm = x * 1000  # Convert to millimeters for precision display
                    self.get_logger().info(f"⚖️✅ PERFECTLY ALIGNED! Distance: {distance:.3f}m, X: {x_mm:+.1f}mm")
                else:
                    # Centered but distance drifted - go back to distance adjustment
                    self.alignment_state = "MOVING_TO_DISTANCE"
                    self.get_logger().info(f"⚖️➡️🎯 Centered but distance off, readjusting...")
            else:
                # Move sideways to center
                cmd_vel = self.center_alignment(x, distance)
                
        elif self.alignment_state == "ALIGNED":
            # Perfect alignment achieved - stay stopped
            cmd_vel = Twist()  # Always send stop command
            
            # Only log occasionally
            if self.log_counter % 50 == 0:  # Every 5 seconds
                x_mm = x * 1000  # Convert to millimeters for precision display
                self.get_logger().info(f"✅ PERFECTLY ALIGNED - Distance: {distance:.3f}m, X: {x_mm:+.1f}mm")
            
            # Check if we've drifted and need realignment
            distance_error = abs(distance - self.target_distance)
            center_error = abs(x)
            
            if distance_error > self.distance_tolerance * 2:
                self.alignment_state = "MOVING_TO_DISTANCE"
                self.movement_complete = False
                self.get_logger().warn(f"⚠️ Distance drifted ({distance:.3f}m)! Re-engaging...")
            elif center_error > self.center_tolerance * 2:
                self.alignment_state = "CENTERING"
                self.movement_complete = False
                self.get_logger().warn(f"⚠️ Center drifted ({x:+.3f}m)! Re-centering...")
        
        # Publish commands and status
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Publish alignment status
        status_msg = Bool()
        status_msg.data = (self.alignment_state == "ALIGNED")
        self.alignment_status_pub.publish(status_msg)
        
        # Log status
        self.log_status(x, distance)

    def move_to_distance(self, current_distance):
        """BANG-BANG CONTROL: Full speed (0.1 m/s) until target, then immediate stop."""
        cmd = Twist()
        
        distance_error = current_distance - self.target_distance
        
        # Log movement status
        if self.log_counter % 10 == 0:  # Every 1 second
            self.get_logger().info(f"📏 Moving to target: Current={current_distance:.3f}m, Target={self.target_distance}m, Error={distance_error:+.3f}m")
        
        # BANG-BANG CONTROL: Full speed or complete stop
        if abs(distance_error) > self.distance_tolerance:
            # Move at FULL SPEED in correct direction
            if distance_error < 0:
                # Too close - move BACKWARD at full speed
                linear_x = -self.max_linear_vel  # Full speed backward
                self.get_logger().info(f"🔙 Too close! Moving BACKWARD at FULL SPEED ({linear_x:.3f} m/s)")
            else:
                # Too far - move FORWARD at full speed
                linear_x = self.max_linear_vel   # Full speed forward
                self.get_logger().info(f"🔜 Too far! Moving FORWARD at FULL SPEED ({linear_x:.3f} m/s)")
                
            cmd.linear.x = linear_x
            cmd.linear.y = 0.0  # No sideways movement
            cmd.angular.z = 0.0 # No rotation
        else:
            # Within tolerance - IMMEDIATE STOP
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("🛑 IMMEDIATE STOP - Target distance reached!")
            
        return cmd

    def adjust_distance(self, current_distance, x_offset):
        """Adjust robot distance to target (Mecanum drive)."""
        cmd = Twist()
        
        distance_error = current_distance - self.target_distance
        
        # Very gentle forward/backward movement
        linear_x = -self.distance_kp * distance_error
        
        # Apply velocity limits and add deadband to prevent jittery movement
        if abs(linear_x) < 0.01:  # Deadband to prevent tiny movements
            linear_x = 0.0
        linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))
        
        # Very gentle sideways movement for centering
        linear_y = -self.center_kp * 0.2 * x_offset  # Even more gentle
        
        # Apply deadband and limits for sideways movement
        if abs(linear_y) < 0.005:  # Deadband for centering
            linear_y = 0.0
        linear_y = max(-self.max_linear_vel * 0.5, min(self.max_linear_vel * 0.5, linear_y))
        
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = 0.0
        
        return cmd

    def center_alignment(self, x_offset, current_distance):
        """Center robot alignment (X = 0) using Mecanum sideways movement."""
        cmd = Twist()
        
        # BANG-BANG CENTERING: Full speed until target, then immediate stop
        # MECANUM ADVANTAGE: Direct sideways movement without rotation!
        
        # BANG-BANG CONTROL for centering
        if abs(x_offset) > self.center_tolerance:
            # Move at FULL SIDEWAYS SPEED in correct direction
            if x_offset > 0:
                # Tag is to the right - move LEFT at full speed
                linear_y = -self.max_linear_vel  # Full speed left
                direction = "LEFT"
            else:
                # Tag is to the left - move RIGHT at full speed  
                linear_y = self.max_linear_vel   # Full speed right
                direction = "RIGHT"
        else:
            # Within tolerance - IMMEDIATE STOP
            linear_y = 0.0
            direction = "STOP"
        
        # No distance adjustment during centering - focus only on centering
        linear_x = 0.0  # Pure sideways movement only
        
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = 0.0  # No rotation needed!
        
        # Log bang-bang centering with clear status
        if self.log_counter % 10 == 0:  # Every 1 second
            x_mm = x_offset * 1000  # Convert to millimeters for precision display
            if direction == "STOP":
                self.get_logger().info(f"⚖️ CENTERING COMPLETE: X={x_mm:+.1f}mm → STOP!")
            else:
                self.get_logger().info(f"⚖️ Bang-Bang Centering: X={x_mm:+.1f}mm → {direction} at FULL SPEED (0.1 m/s)")
        
        return cmd

    def maintain_alignment(self, x_offset, current_distance):
        """Maintain perfect alignment using Mecanum precision."""
        cmd = Twist()
        
        distance_error = current_distance - self.target_distance
        
        # Very fine adjustments only with larger deadbands
        linear_x = -self.distance_kp * 0.1 * distance_error  # Minimal fine-tuning
        linear_y = -self.center_kp * 0.1 * x_offset          # Minimal fine-tuning
        
        # Larger deadbands for stable alignment
        if abs(linear_x) < 0.01:
            linear_x = 0.0
        if abs(linear_y) < 0.005:
            linear_y = 0.0
        
        # Very conservative limits
        linear_x = max(-self.max_linear_vel * 0.2, min(self.max_linear_vel * 0.2, linear_x))
        linear_y = max(-self.max_linear_vel * 0.2, min(self.max_linear_vel * 0.2, linear_y))
        
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = 0.0
        
        return cmd

    def stop_robot(self):
        """Stop robot movement."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all movement."""
        self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED!')
        
        # Send multiple stop commands to ensure robot stops
        for _ in range(5):
            cmd = Twist()  # All zeros
            self.cmd_vel_pub.publish(cmd)
        
        self.get_logger().info('🛑 Robot stopped. Safe to exit.')

    def log_status(self, x, distance):
        """Log current alignment status."""
        distance_error = abs(distance - self.target_distance)
        center_error = abs(x)
        
        self.log_counter += 1
        if self.log_counter % 30 == 0:  # Log every 3 seconds (30 * 0.1s = 3s)
            self.get_logger().info(
                f'🎯 {self.alignment_state} | '
                f'Distance: {distance:.3f}m (target: {self.target_distance}m, error: {distance_error:.3f}m) | '
                f'Center: {x:+.3f}m (error: {center_error:.3f}m)'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = AprilTagAlignmentController()
    
    def signal_handler(signum, frame):
        """Handle Ctrl+C and other signals for emergency stop."""
        node.get_logger().warn('🚨 Signal received! Emergency stopping...')
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    # Register signal handlers for emergency stop
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Terminate
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('🚨 Keyboard interrupt! Emergency stopping...')
        node.emergency_stop()
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
