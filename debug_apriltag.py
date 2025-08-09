#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from apriltag import apriltag

class DebugAprilTag(Node):
    def __init__(self):
        super().__init__('debug_apriltag')
        
        self.bridge = CvBridge()
        self.detector = apriltag("tag36h11")  # Same as your AR app
        
        # Subscribe to camera (same topic as your AR app)
        self.image_sub = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.debug_callback,
            10
        )
        
        self.get_logger().info("🔍 Debug AprilTag detector started")
        self.get_logger().info("📷 Listening to /depth_cam/rgb/image_raw")

    def debug_callback(self, msg):
        try:
            # Convert image (same as AR app)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            
            # Detect tags (same as AR app)
            detections = self.detector.detect(gray)
            
            # Print what we find
            if detections:
                self.get_logger().info(f"🎯 Found {len(detections)} tags!")
                for i, detection in enumerate(detections):
                    self.get_logger().info(f"   Tag {i}:")
                    self.get_logger().info(f"     Type: {type(detection)}")
                    self.get_logger().info(f"     Keys: {detection.keys() if isinstance(detection, dict) else 'Not a dict'}")
                    if isinstance(detection, dict):
                        tag_id = detection.get('id', 'NO_ID')
                        self.get_logger().info(f"     ID: {tag_id}")
                        if 'center' in detection:
                            center = detection['center']
                            self.get_logger().info(f"     Center: ({center[0]:.1f}, {center[1]:.1f})")
            else:
                self.get_logger().info("🔍 No tags detected")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

def main():
    rclpy.init()
    node = DebugAprilTag()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
