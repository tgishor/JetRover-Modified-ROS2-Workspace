#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from apriltag import apriltag
import math

class AprilTagPoseAnalyzer(Node):
    def __init__(self):
        super().__init__('apriltag_pose_analyzer')
        
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Tag parameters
        self.tag_size = 0.10  # 10cm tag size (adjust if different)
        self.target_tag_id = 2
        
        # Subscribe to camera and camera info
        self.image_sub = self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 10)
        
        self.get_logger().info("📊 AprilTag Pose Analyzer started")
        self.get_logger().info(f"🎯 Target tag: TAG36H11-{self.target_tag_id}")
        self.get_logger().info(f"📏 Tag size: {self.tag_size*100}cm")

    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera info."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("📷 Camera calibration received")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
            
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            result_image = np.copy(rgb_image)
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            
            # Detect AprilTags
            detections = self.detector.detect(gray)
            
            for detection in detections:
                if isinstance(detection, dict):
                    tag_id = detection.get('id', -1)
                    
                    if tag_id == self.target_tag_id:
                        # Calculate pose information
                        pose_data = self.calculate_tag_pose(detection)
                        
                        if pose_data:
                            # Draw visualization
                            result_image = self.draw_pose_info(result_image, detection, pose_data)
                            
                            # Print detailed pose information
                            self.print_pose_analysis(pose_data)
            
            # Display result
            display_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("AprilTag Pose Analysis", display_image)
            cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def calculate_tag_pose(self, detection):
        """Calculate complete pose information from tag detection."""
        try:
            # Get tag data
            tag_center = detection['center']
            tag_corners = detection['lb-rb-rt-lt']
            
            # Extract corners
            lb = tag_corners[0]  # left-bottom
            rb = tag_corners[1]  # right-bottom  
            rt = tag_corners[2]  # right-top
            lt = tag_corners[3]  # left-top
            
            # Create corners array for PnP
            corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
            
            # Define 3D object points for the tag
            half_size = self.tag_size / 2.0
            OBJP = np.array([
                [-half_size, -half_size, 0],  # left-bottom
                [ half_size, -half_size, 0],  # right-bottom
                [-half_size,  half_size, 0],  # left-top
                [ half_size,  half_size, 0],  # right-top
                [ 0,  0,  0]                  # center
            ], dtype=np.float32)
            
            # Solve PnP to get pose
            ret, rvec, tvec = cv2.solvePnP(OBJP, corners, self.camera_matrix, self.dist_coeffs)
            
            if ret:
                # Extract position
                x, y, z = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                
                # Calculate distance
                distance = math.sqrt(x*x + y*y + z*z)
                
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Extract Euler angles (roll, pitch, yaw)
                roll, pitch, yaw = self.rotation_matrix_to_euler(rotation_matrix)
                
                # Calculate relative angle in camera view
                angle_in_view = math.atan2(x, z)  # Angle from camera center
                
                return {
                    'position': {'x': x, 'y': y, 'z': z},
                    'distance': distance,
                    'angles': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
                    'angle_in_view': angle_in_view,
                    'rotation_matrix': rotation_matrix,
                    'rvec': rvec,
                    'tvec': tvec,
                    'corners_2d': corners,
                    'tag_center': tag_center
                }
                
        except Exception as e:
            self.get_logger().error(f"Pose calculation error: {e}")
            return None

    def rotation_matrix_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)."""
        # Extract Euler angles from rotation matrix
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])  # roll
            y = math.atan2(-R[2,0], sy)     # pitch  
            z = math.atan2(R[1,0], R[0,0])  # yaw
        else:
            x = math.atan2(-R[1,2], R[1,1])  # roll
            y = math.atan2(-R[2,0], sy)      # pitch
            z = 0                            # yaw
            
        return math.degrees(x), math.degrees(y), math.degrees(z)

    def print_pose_analysis(self, pose_data):
        """Print detailed pose analysis to console."""
        pos = pose_data['position']
        angles = pose_data['angles']
        
        print("\n" + "="*60)
        print(f"🎯 APRILTAG POSE ANALYSIS (ID: {self.target_tag_id})")
        print("="*60)
        
        print("📍 POSITION (relative to camera):")
        print(f"   X: {pos['x']:+7.3f}m  (Left/Right - negative=left, positive=right)")
        print(f"   Y: {pos['y']:+7.3f}m  (Up/Down - negative=down, positive=up)")  
        print(f"   Z: {pos['z']:+7.3f}m  (Forward/Back - positive=away from camera)")
        
        print(f"\n📏 DISTANCE: {pose_data['distance']:.3f}m ({pose_data['distance']*100:.1f}cm)")
        
        print(f"\n📐 ORIENTATION:")
        print(f"   Roll:  {angles['roll']:+7.1f}° (rotation around Z-axis)")
        print(f"   Pitch: {angles['pitch']:+7.1f}° (rotation around Y-axis)")
        print(f"   Yaw:   {angles['yaw']:+7.1f}° (rotation around X-axis)")
        
        print(f"\n🎯 VIEWING ANGLE: {math.degrees(pose_data['angle_in_view']):+6.1f}° from camera center")
        
        print("\n🤖 FORMATION CONTROL INFO:")
        print(f"   • Robot can maintain {pose_data['distance']:.2f}m distance")
        print(f"   • Tag is {pos['x']:+.2f}m to the {'right' if pos['x'] > 0 else 'left'}")
        print(f"   • Robot should turn {math.degrees(pose_data['angle_in_view']):+.1f}° to face tag")
        
        if pose_data['distance'] > 0.3:
            print(f"   • Move FORWARD {pose_data['distance']-0.5:.2f}m to maintain 50cm formation")
        elif pose_data['distance'] < 0.7:
            print(f"   • Move BACKWARD {0.5-pose_data['distance']:.2f}m to maintain 50cm formation")
        else:
            print(f"   • Distance OK for 50cm formation")
            
        print("="*60)

    def draw_pose_info(self, image, detection, pose_data):
        """Draw pose information on the image."""
        tag_center = detection['center']
        tag_corners = detection['lb-rb-rt-lt']
        pos = pose_data['position']
        
        # Draw tag detection
        for i, corner in enumerate(tag_corners):
            cv2.circle(image, (int(corner[0]), int(corner[1])), 4, (0, 255, 255), -1)
        
        # Draw center
        center_pt = (int(tag_center[0]), int(tag_center[1]))
        cv2.circle(image, center_pt, 6, (255, 0, 0), -1)
        
        # Draw bounding box
        corners_int = np.array([[int(c[0]), int(c[1])] for c in tag_corners])
        cv2.polylines(image, [corners_int], True, (0, 255, 0), 2)
        
        # Draw coordinate axes
        axis_length = 50
        axis_3d = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],  # X-axis (red)
            [0, axis_length, 0],  # Y-axis (green)  
            [0, 0, -axis_length]  # Z-axis (blue)
        ], dtype=np.float32) / 1000.0  # Convert to meters
        
        axis_2d, _ = cv2.projectPoints(axis_3d, pose_data['rvec'], pose_data['tvec'], 
                                      self.camera_matrix, self.dist_coeffs)
        axis_2d = axis_2d.reshape(-1, 2).astype(int)
        
        # Draw axes
        cv2.arrowedLine(image, tuple(axis_2d[0]), tuple(axis_2d[1]), (0, 0, 255), 3)  # X-red
        cv2.arrowedLine(image, tuple(axis_2d[0]), tuple(axis_2d[2]), (0, 255, 0), 3)  # Y-green
        cv2.arrowedLine(image, tuple(axis_2d[0]), tuple(axis_2d[3]), (255, 0, 0), 3)  # Z-blue
        
        # Draw text info
        y_offset = 30
        cv2.putText(image, f"Distance: {pose_data['distance']:.2f}m", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        y_offset += 25
        cv2.putText(image, f"Position: ({pos['x']:+.2f}, {pos['y']:+.2f}, {pos['z']:+.2f})", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        y_offset += 20
        cv2.putText(image, f"Angle: {math.degrees(pose_data['angle_in_view']):+.1f}deg", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        return image

def main():
    rclpy.init()
    node = AprilTagPoseAnalyzer()
    
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
