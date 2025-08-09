#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from apriltag import apriltag
import math

class VisualAlignmentMonitor(Node):
    """
    Visual monitor that shows:
    1. Live camera feed
    2. AprilTag detection overlay
    3. Real-time alignment metrics
    4. Target zones and progress indicators
    """

    def __init__(self):
        super().__init__('visual_alignment_monitor')
        
        self.declare_parameter('robot_namespace', 'robot_1')
        self.declare_parameter('target_distance', 2.0)
        self.declare_parameter('target_tag_id', 2)
        
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.target_tag_id = self.get_parameter('target_tag_id').get_parameter_value().integer_value
        
        # Clean namespace and handle empty namespace
        clean_namespace = self.robot_namespace.strip('/')
        if not clean_namespace:
            clean_namespace = 'robot_1'  # Default if empty
        
        # OpenCV and AprilTag setup
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")
        
        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.tag_size = 0.10  # 10cm
        
        # State tracking
        self.current_tag_pose = None
        self.alignment_status = False
        self.alignment_state = "SEARCHING"
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 10)
        self.tag_pose_sub = self.create_subscription(
            PoseStamped, f'/{clean_namespace}/leader_tag_pose', self.tag_pose_callback, 10)
        self.alignment_status_sub = self.create_subscription(
            Bool, f'/{clean_namespace}/alignment_complete', self.alignment_status_callback, 10)
        
        self.get_logger().info("📺 Visual Alignment Monitor Started")
        self.get_logger().info(f"🎯 Target: TAG36H11-{self.target_tag_id} at {self.target_distance}m")
        self.get_logger().info("👁️ Press 'q' to quit, 'f' for fullscreen")

    def camera_info_callback(self, msg):
        """Update camera intrinsics."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def tag_pose_callback(self, msg):
        """Update current tag pose."""
        self.current_tag_pose = msg

    def alignment_status_callback(self, msg):
        """Update alignment status."""
        self.alignment_status = msg.data

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            display_image = np.copy(rgb_image)
            
            # Detect AprilTags for visual overlay
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            detections = self.detector.detect(gray)
            
            # Draw AprilTag detections
            display_image = self.draw_apriltag_overlay(display_image, detections)
            
            # Draw alignment UI
            display_image = self.draw_alignment_ui(display_image)
            
            # Draw target zones
            display_image = self.draw_target_zones(display_image)
            
            # Draw metrics panel
            display_image = self.draw_metrics_panel(display_image)
            
            # Show the image
            display_bgr = cv2.cvtColor(display_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("AprilTag Alignment Monitor", display_bgr)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('f'):
                # Toggle fullscreen (simple implementation)
                cv2.setWindowProperty("AprilTag Alignment Monitor", 
                                    cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                
        except Exception as e:
            self.get_logger().error(f"Display error: {e}")

    def draw_apriltag_overlay(self, image, detections):
        """Draw AprilTag detection overlay."""
        for detection in detections:
            if isinstance(detection, dict):
                tag_id = detection.get('id', -1)
                tag_center = detection['center']
                tag_corners = detection['lb-rb-rt-lt']
                
                # Color coding: target tag vs others
                if tag_id == self.target_tag_id:
                    corner_color = (0, 255, 0)  # Green for target
                    text_color = (0, 255, 0)
                    thickness = 3
                else:
                    corner_color = (255, 255, 0)  # Yellow for others
                    text_color = (255, 255, 0)
                    thickness = 2
                
                # Draw corners
                for corner in tag_corners:
                    cv2.circle(image, (int(corner[0]), int(corner[1])), 4, corner_color, -1)
                
                # Draw bounding box
                corners_int = np.array([[int(c[0]), int(c[1])] for c in tag_corners])
                cv2.polylines(image, [corners_int], True, corner_color, thickness)
                
                # Draw center
                center_pt = (int(tag_center[0]), int(tag_center[1]))
                cv2.circle(image, center_pt, 6, (255, 0, 0), -1)
                
                # Draw ID and status
                cv2.putText(image, f'ID: {tag_id}', 
                           (center_pt[0] - 30, center_pt[1] - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
                
                if tag_id == self.target_tag_id:
                    cv2.putText(image, 'TARGET', 
                               (center_pt[0] - 35, center_pt[1] + 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        return image

    def draw_target_zones(self, image):
        """Draw target alignment zones."""
        h, w = image.shape[:2]
        
        # Center line (X = 0 target)
        center_x = w // 2
        cv2.line(image, (center_x, 0), (center_x, h), (0, 255, 255), 2)
        cv2.putText(image, 'CENTER TARGET', (center_x + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Distance zones (approximate based on field of view)
        # These are rough estimates - actual zones depend on camera FOV
        zone_2m = int(h * 0.4)  # Rough estimate for 2m distance appearance
        zone_1m = int(h * 0.6)  # Rough estimate for 1m distance
        
        cv2.line(image, (0, zone_2m), (w, zone_2m), (0, 255, 0), 2)
        cv2.putText(image, f'{self.target_distance}m TARGET', (10, zone_2m - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.line(image, (0, zone_1m), (w, zone_1m), (255, 255, 0), 1)
        cv2.putText(image, '~1m REF', (10, zone_1m - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return image

    def draw_alignment_ui(self, image):
        """Draw alignment status indicators."""
        h, w = image.shape[:2]
        
        # Status indicator
        status_color = (0, 255, 0) if self.alignment_status else (255, 0, 0)
        status_text = "ALIGNED ✓" if self.alignment_status else "ALIGNING..."
        
        cv2.rectangle(image, (w - 200, 10), (w - 10, 50), status_color, -1)
        cv2.putText(image, status_text, (w - 190, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Progress bars for alignment
        if self.current_tag_pose:
            self.draw_progress_bars(image)
        
        return image

    def draw_progress_bars(self, image):
        """Draw progress bars for distance and centering."""
        h, w = image.shape[:2]
        x = self.current_tag_pose.pose.position.x
        z = self.current_tag_pose.pose.position.z
        distance = math.sqrt(x*x + z*z)
        
        # Distance progress bar
        bar_y = h - 100
        bar_width = 200
        bar_height = 20
        bar_x = w - bar_width - 20
        
        # Distance bar background
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (64, 64, 64), -1)
        
        # Distance progress (normalized to 0-4m range)
        distance_progress = min(1.0, distance / 4.0)
        progress_width = int(bar_width * distance_progress)
        
        # Color based on target distance
        distance_error = abs(distance - self.target_distance)
        if distance_error < 0.1:
            bar_color = (0, 255, 0)  # Green - good
        elif distance_error < 0.3:
            bar_color = (255, 255, 0)  # Yellow - close
        else:
            bar_color = (255, 0, 0)  # Red - far
            
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + progress_width, bar_y + bar_height), bar_color, -1)
        
        # Distance text
        cv2.putText(image, f'Distance: {distance:.2f}m (target: {self.target_distance}m)', 
                   (bar_x, bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Center alignment bar
        center_bar_y = bar_y + 40
        cv2.rectangle(image, (bar_x, center_bar_y), (bar_x + bar_width, center_bar_y + bar_height), (64, 64, 64), -1)
        
        # Center progress (normalized to ±0.5m range)
        center_offset = max(-0.5, min(0.5, x))
        center_progress = (center_offset + 0.5) / 1.0  # Normalize to 0-1
        center_pos = int(bar_width * center_progress)
        
        # Center indicator
        if abs(x) < 0.05:
            center_color = (0, 255, 0)  # Green - centered
        elif abs(x) < 0.15:
            center_color = (255, 255, 0)  # Yellow - close
        else:
            center_color = (255, 0, 0)  # Red - off center
            
        cv2.circle(image, (bar_x + center_pos, center_bar_y + bar_height//2), 8, center_color, -1)
        
        # Center line
        center_line_x = bar_x + bar_width // 2
        cv2.line(image, (center_line_x, center_bar_y), (center_line_x, center_bar_y + bar_height), (0, 255, 255), 2)
        
        # Center text
        cv2.putText(image, f'Center: {x:+.3f}m (target: 0.000m)', 
                   (bar_x, center_bar_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_metrics_panel(self, image):
        """Draw detailed metrics panel."""
        h, w = image.shape[:2]
        
        # Semi-transparent overlay
        overlay = image.copy()
        cv2.rectangle(overlay, (10, h - 150), (350, h - 10), (0, 0, 0), -1)
        image = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
        
        y_pos = h - 130
        line_height = 25
        
        cv2.putText(image, "ALIGNMENT METRICS:", (20, y_pos), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_pos += line_height
        
        if self.current_tag_pose:
            x = self.current_tag_pose.pose.position.x
            z = self.current_tag_pose.pose.position.z
            distance = math.sqrt(x*x + z*z)
            
            cv2.putText(image, f"Distance: {distance:.3f}m", (20, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_pos += line_height
            
            cv2.putText(image, f"X-offset: {x:+.3f}m", (20, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_pos += line_height
            
            cv2.putText(image, f"Z-depth:  {z:+.3f}m", (20, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
        else:
            cv2.putText(image, "No tag detected", (20, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        return image

def main():
    rclpy.init()
    node = VisualAlignmentMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
