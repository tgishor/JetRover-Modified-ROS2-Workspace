#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from servo_controller_msgs.msg import ServosPosition, ServoPosition
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
        self.declare_parameter('center_tolerance', 0.002)   # 5mm ultra-precise centering tolerance
        self.declare_parameter('max_linear_velocity', 0.08)  # Faster speed for quicker alignment
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
        
        # Alignment lock to prevent immediate re-engagement
        self.alignment_lock_time = None
        self.alignment_lock_duration = 5.0  # 5 seconds lock after alignment
        
        # PID controllers - Much gentler gains
        self.distance_kp = 0.15  # Reduced from 0.8 for smoother movement
        self.center_kp = 0.3     # Reduced from 1.5 for smoother centering
        
        # Clean namespace and handle empty namespace
        clean_namespace = self.robot_namespace.strip('/')
        if not clean_namespace:
            clean_namespace = 'robot_1'  # Default if empty
        
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
        
        # Arm control publisher
        servo_topic = f'/{clean_namespace}/servo_controller'
        self.arm_pub = self.create_publisher(
            ServosPosition,
            servo_topic,
            10
        )
        self.get_logger().info(f'🦾 Arm publisher created on topic: {servo_topic}')
        
        # Arm formation tracking
        self.search_formation_set = False
        self.home_formation_set = False
        self.search_revert_phase = 0  # 0=not started, 1=joints_4_3_moving, 2=joint_2_moving, 3=complete
        self.search_revert_start_time = None
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info(f'🎯 AprilTag Alignment Controller Started')
        self.get_logger().info(f'📏 Target distance: {self.target_distance}m')
        self.get_logger().info(f'⚖️ Center tolerance: ±{self.center_tolerance*1000:.1f}mm (ultra-precise)')
        self.get_logger().info(f'🔍 State: {self.alignment_state}')
        self.get_logger().info('🛑 Press Ctrl+C for emergency stop')
        
        # Set initial arm search formation
        self.get_logger().info('🦾 Initializing arm to search formation...')
        self.set_search_formation()

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
                self.search_formation_set = False  # Reset search formation flag
                self.home_formation_set = False    # Reset home formation flag
                self.search_revert_phase = 0       # Reset sequential revert phase
                self.search_revert_start_time = None
                self.get_logger().info("🏷️ Tag lost! Returning to SEARCHING state...")
            
            # Set arm to search formation during SEARCHING phase
            self.set_search_formation()
            
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
                    self.alignment_lock_time = self.get_clock().now()  # Lock alignment for 5 seconds
                    cmd_vel = Twist()  # Complete stop
                    x_mm = x * 1000  # Convert to millimeters for precision display
                    self.get_logger().info(f"⚖️✅ PERFECTLY ALIGNED! Distance: {distance:.3f}m, X: {x_mm:+.1f}mm - LOCKED for 5s")
                    
                    # Move arm to home position now that robot is perfectly aligned
                    self.set_home_formation()
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
            
            # Check if alignment lock is still active
            current_time = self.get_clock().now()
            if self.alignment_lock_time is not None:
                elapsed_lock_time = (current_time - self.alignment_lock_time).nanoseconds / 1e9
                if elapsed_lock_time < self.alignment_lock_duration:
                    # Still locked - don't check for drift, just stay aligned
                    remaining_lock = self.alignment_lock_duration - elapsed_lock_time
                    if self.log_counter % 50 == 0:  # Log lock status occasionally
                        self.get_logger().info(f"🔒 ALIGNMENT LOCKED - {remaining_lock:.1f}s remaining")
                    return  # Exit early, don't check drift
            
            # Lock expired - now check for SERIOUS drift only
            distance_error = abs(distance - self.target_distance)
            center_error = abs(x)
            
            # Only re-engage if EXTREMELY drifted (not just sensor noise)
            if distance_error > 0.30:  # 30cm drift before re-engaging (very large)
                self.alignment_state = "MOVING_TO_DISTANCE"
                self.movement_complete = False
                self.alignment_lock_time = None  # Reset lock
                self.get_logger().warn(f"⚠️ Distance EXTREMELY drifted ({distance:.3f}m)! Re-engaging...")
            elif center_error > 0.10:  # 10cm drift before re-centering (very large)
                self.alignment_state = "CENTERING"
                self.movement_complete = False
                self.alignment_lock_time = None  # Reset lock
                self.get_logger().warn(f"⚠️ Center EXTREMELY drifted ({x:+.3f}m)! Re-centering...")
        
        # Publish commands and status
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Publish alignment status
        status_msg = Bool()
        status_msg.data = (self.alignment_state == "ALIGNED")
        self.alignment_status_pub.publish(status_msg)
        
        # Log status
        self.log_status(x, distance)

    def move_to_distance(self, current_distance):
        """Move robot to exact target distance and stop."""
        cmd = Twist()
        
        distance_error = current_distance - self.target_distance
        
        # Log less frequently to reduce noise in output
        if self.log_counter % 10 == 0:  # Every 1 second instead of every 0.1s
            self.get_logger().info(f"📏 Moving to target: Current={current_distance:.3f}m, Target={self.target_distance}m, Error={distance_error:+.3f}m")
        
        # More conservative proportional control with deadband
        if abs(distance_error) > self.distance_tolerance:
            # Use much smaller gain to prevent overshooting
            conservative_kp = 0.05  # Very gentle approach
            
            # FIXED: Calculate movement direction correctly
            # distance_error = current - target
            # If current=1.7m, target=2.0m → error=-0.3m (too close, need to move backward)
            # If current=2.3m, target=2.0m → error=+0.3m (too far, need to move forward)
            
            if distance_error < 0:
                # Too close (current < target) - need to move BACKWARD 
                # ROBOT COORDINATE CHECK: Try NEGATIVE linear.x for backward movement
                linear_x = distance_error * conservative_kp  # Negative for backward movement
                linear_x = max(-self.max_linear_vel * 0.5, linear_x)  # Limit backward speed
                self.get_logger().info(f"🔙 Too close! Moving BACKWARD (linear.x={linear_x:.3f}) m/s")
            else:
                # Too far (current > target) - need to move FORWARD
                # ROBOT COORDINATE CHECK: Try POSITIVE linear.x for forward movement  
                linear_x = distance_error * conservative_kp  # Positive for forward movement
                linear_x = min(self.max_linear_vel * 0.5, linear_x)   # Limit forward speed
                self.get_logger().info(f"🔜 Too far! Moving FORWARD (linear.x={linear_x:.3f}) m/s")
            
                         # Ensure minimum speed of 3 decimal places for motor to work
            if abs(linear_x) < 0.005:
                if distance_error < 0:
                    linear_x = -0.005  # Minimum backward speed (3 decimals)
                else:
                    linear_x = 0.005   # Minimum forward speed (3 decimals)
                
            cmd.linear.x = linear_x
            cmd.linear.y = 0.0  # No sideways movement - just distance
            cmd.angular.z = 0.0 # No rotation
        else:
            # Within tolerance - stop completely
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            
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
        
        # Focus on centering with sideways movement
        # MECANUM ADVANTAGE: Direct sideways movement without rotation!
        
        # Faster centering with better motor compatibility
        if abs(x_offset) > 0.03:  # Far from center (>3cm)
            center_kp = 0.25  # Much more aggressive for speed
        elif abs(x_offset) > 0.01:  # Medium distance (1-3cm)
            center_kp = 0.15  # Moderate but faster approach
        else:  # Very close to center (<1cm)
            center_kp = 0.08  # Still responsive but controlled
        
        # Calculate sideways movement for centering
        # Positive X = tag is to the right, need to move left (negative linear.y)
        # Negative X = tag is to the left, need to move right (positive linear.y)
        linear_y = -center_kp * x_offset
        
        # Ensure minimum 3-decimal velocity for motor operation
        if abs(linear_y) > 0.0 and abs(linear_y) < 0.005:
            if linear_y > 0:
                linear_y = 0.005  # Minimum right movement
            else:
                linear_y = -0.005  # Minimum left movement
        
        # Better deadband - stop only when very close
        if abs(x_offset) < self.center_tolerance:
            linear_y = 0.0
        
        # Faster speed limits for quicker centering
        max_center_speed = self.max_linear_vel * 0.6  # 60% of max speed for faster centering
        linear_y = max(-max_center_speed, min(max_center_speed, linear_y))
        
        # Very minor distance maintenance during centering
        distance_error = current_distance - self.target_distance
        if abs(distance_error) > self.distance_tolerance * 0.5:  # Only if significantly off
            linear_x = distance_error * 0.02  # Very gentle distance correction
            if abs(linear_x) < 0.005:  # Deadband
                linear_x = 0.0
            linear_x = max(-self.max_linear_vel * 0.2, min(self.max_linear_vel * 0.2, linear_x))
        else:
            linear_x = 0.0  # Don't adjust distance if close enough
        
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = 0.0  # No rotation needed!
        
        # Log centering progress with ultra-precise display
        if self.log_counter % 10 == 0:  # Every 1 second
            direction = "LEFT" if linear_y < 0 else "RIGHT" if linear_y > 0 else "STOP"
            x_mm = x_offset * 1000  # Convert to millimeters for precision display
            self.get_logger().info(f"⚖️ Ultra-Centering: X={x_mm:+.1f}mm → {direction} (speed={linear_y:+.4f}m/s)")
        
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

    def set_search_formation(self):
        """Set arm to search formation with sequential movement for clearance."""
        if self.search_formation_set:
            return  # Already set, don't repeat
            
        # Check if we need sequential movement (coming from home position)
        if self.home_formation_set:
            # We're coming from home position - use sequential movement for clearance
            self.get_logger().info(f"🦾 Coming from HOME - Using sequential movement for clearance... (phase={self.search_revert_phase})")
            current_time = self.get_clock().now()
            
            if self.search_revert_phase == 0:
                # Phase 1: Move Joint4 and Joint3 first (for clearance)
                self.get_logger().info("🦾 Search formation Phase 1: Moving Joint4 & Joint3 first...")
                
                msg = ServosPosition()
                msg.position_unit = 'pulse'
                msg.duration = 1.0  # 1 second movement
                
                # Move Joint4 and Joint3 first
                phase1_positions = {
                    4: 825,   # Joint4: 825 (Wrist1)
                    3: 200,   # Joint3: 200 (Elbow)
                }
                
                for servo_id, position in phase1_positions.items():
                    position = max(50, min(950, position))
                    servo = ServoPosition()
                    servo.id = servo_id
                    servo.position = float(position)
                    msg.position.append(servo)
                    self.get_logger().info(f"🔧 DEBUG: Added servo {servo_id} -> {position} to message")
                
                self.arm_pub.publish(msg)
                self.search_revert_phase = 1
                self.search_revert_start_time = current_time
                self.get_logger().info("🦾 Phase 1 sent: Joint4=825, Joint3=200")
                self.get_logger().info(f"🔧 DEBUG: Published message with {len(msg.position)} servos, duration={msg.duration}, unit={msg.position_unit}")
                
            elif self.search_revert_phase == 1:
                # Wait 1.5 seconds before moving Joint2
                if self.search_revert_start_time is not None:
                    elapsed = (current_time - self.search_revert_start_time).nanoseconds / 1e9
                    if self.log_counter % 10 == 0:  # Log every 1 second
                        self.get_logger().info(f"🕐 Waiting for Joint2... {elapsed:.1f}s / 1.5s")
                    if elapsed >= 1.5:
                        # Phase 2: Move Joint2 last
                        self.get_logger().info("🦾 Search formation Phase 2: Moving Joint2 last...")
                        
                        msg = ServosPosition()
                        msg.position_unit = 'pulse'
                        msg.duration = 1.0  # 1 second movement
                        
                        # Move Joint2 last
                        servo = ServoPosition()
                        servo.id = 2
                        servo.position = float(75)  # Joint2: 75
                        msg.position.append(servo)
                        
                        self.arm_pub.publish(msg)
                        self.search_revert_phase = 2
                        self.get_logger().info("🦾 Phase 2 sent: Joint2=75")
                        self.get_logger().info(f"🔧 DEBUG: Published Joint2 message with {len(msg.position)} servos, duration={msg.duration}, unit={msg.position_unit}")
                        
            elif self.search_revert_phase == 2:
                # Wait 1.2 seconds for final movement to complete
                if self.search_revert_start_time is not None:
                    elapsed = (current_time - self.search_revert_start_time).nanoseconds / 1e9
                    if elapsed >= 2.7:  # 1.5 + 1.2 = 2.7 total
                        self.search_revert_phase = 3
                        self.search_formation_set = True
                        self.get_logger().info("🦾 ✅ Search formation COMPLETE: Sequential movement finished!")
        else:
            # Initial search formation setup or not coming from home - move all at once
            self.get_logger().info("🦾 Initial search formation setup - All joints at once...")
            
            msg = ServosPosition()
            msg.position_unit = 'pulse'
            msg.duration = 2.0  # 2 second smooth movement
            
            # Search formation positions - all at once for initial setup
            search_positions = {
                2: 75,    # Joint2: 75
                3: 200,   # Joint3: 200  
                4: 825,   # Joint4: 825
            }
            
            for servo_id, position in search_positions.items():
                position = max(50, min(950, position))
                servo = ServoPosition()
                servo.id = servo_id
                servo.position = float(position)
                msg.position.append(servo)
            
            self.arm_pub.publish(msg)
            self.search_formation_set = True
            self.get_logger().info("🦾 Initial search formation set: Joint2=75, Joint3=200, Joint4=825")

    def set_home_formation(self):
        """Set arm to home formation (all joints 500) after tag found and stable."""
        if self.home_formation_set:
            return  # Already set, don't repeat
            
        self.get_logger().info("🏠 Setting arm to HOME formation...")
        
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = 1.5  # 1.5 second smooth movement to home
        
        # Home formation positions (all joints at 500)
        home_positions = {
            1: 500,   # Joint1: 500 (Base)
            2: 500,   # Joint2: 500 (Shoulder)
            3: 500,   # Joint3: 500 (Elbow)
            4: 500,   # Joint4: 500 (Wrist1)
            5: 500,   # Joint5: 500 (Wrist2)
            10: 500   # Gripper: 500
        }
        
        for servo_id, position in home_positions.items():
            # Clamp position to safe range
            position = max(50, min(950, position))
            
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(position)
            msg.position.append(servo)
        
        self.arm_pub.publish(msg)
        self.home_formation_set = True
        
        self.get_logger().info("🏠 Home formation set: All joints at 500 pulses")

    def test_single_servo(self, servo_id, position):
        """Test method to move a single servo for debugging."""
        self.get_logger().info(f"🔧 TEST: Moving servo {servo_id} to position {position}")
        
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = 1.0
        
        servo = ServoPosition()
        servo.id = servo_id
        servo.position = float(position)
        msg.position.append(servo)
        
        self.arm_pub.publish(msg)
        self.get_logger().info(f"🔧 TEST: Command sent for servo {servo_id}")

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
