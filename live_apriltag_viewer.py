#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from apriltag import apriltag

class LiveAprilTagViewer(Node):
    def __init__(self):
        super().__init__('live_apriltag_viewer')
        
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")  # Same as your AR app
        
        # Subscribe to camera (same topic as your AR app)
        self.image_sub = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("📹 Live AprilTag viewer started")
        self.get_logger().info("📷 Camera: /depth_cam/rgb/image_raw")
        self.get_logger().info("🎯 Looking for tag36h11 family")
        self.get_logger().info("⌨️  Press 'q' to quit, 'space' to pause")

    def image_callback(self, msg):
        try:
            # Convert image (same as AR app)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            result_image = np.copy(rgb_image)
            
            # Convert to gray for detection
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            
            # Detect AprilTags (same as AR app)
            detections = self.detector.detect(gray)
            
            # Draw detections (same style as AR app)
            if detections:
                for detection in detections:
                    if isinstance(detection, dict):
                        # Get tag info
                        tag_id = detection.get('id', -1)
                        tag_center = detection['center']
                        tag_corners = detection['lb-rb-rt-lt']
                        
                        # Extract corners
                        lb = tag_corners[0]  # left-bottom
                        rb = tag_corners[1]  # right-bottom  
                        rt = tag_corners[2]  # right-top
                        lt = tag_corners[3]  # left-top
                        
                        # Draw corners (same as AR app lines 216-219)
                        cv2.circle(result_image, (int(lb[0]), int(lb[1])), 3, (0, 255, 255), -1)
                        cv2.circle(result_image, (int(lt[0]), int(lt[1])), 3, (0, 255, 255), -1)
                        cv2.circle(result_image, (int(rb[0]), int(rb[1])), 3, (0, 255, 255), -1)
                        cv2.circle(result_image, (int(rt[0]), int(rt[1])), 3, (0, 255, 255), -1)
                        
                        # Draw center
                        cv2.circle(result_image, (int(tag_center[0]), int(tag_center[1])), 5, (255, 0, 0), -1)
                        
                        # Draw bounding box
                        corners_int = np.array([[int(c[0]), int(c[1])] for c in tag_corners])
                        cv2.polylines(result_image, [corners_int], True, (0, 255, 0), 2)
                        
                        # Draw ID text
                        cv2.putText(result_image, f'ID: {tag_id}', 
                                   (int(tag_center[0]) - 30, int(tag_center[1]) - 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                        
                        # Log detection
                        self.get_logger().info(f"🎯 Detected TAG36H11-{tag_id} at ({tag_center[0]:.1f}, {tag_center[1]:.1f})")
                        
                        # Highlight if it's tag ID 2
                        if tag_id == 2:
                            cv2.rectangle(result_image, 
                                        (int(tag_center[0]) - 60, int(tag_center[1]) - 60),
                                        (int(tag_center[0]) + 60, int(tag_center[1]) + 60),
                                        (0, 0, 255), 3)
                            cv2.putText(result_image, 'TARGET TAG!', 
                                       (int(tag_center[0]) - 50, int(tag_center[1]) + 80),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Add info overlay
            cv2.putText(result_image, f'Tags found: {len(detections)}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(result_image, 'Press Q to quit', 
                       (10, result_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show image (convert back to BGR for OpenCV display)
            display_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("Live AprilTag Detection", display_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("🛑 Quitting...")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

def main():
    rclpy.init()
    node = LiveAprilTagViewer()
    
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
