#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleAprilTagTest(Node):
    def __init__(self):
        super().__init__('simple_apriltag_test')
        
        self.get_logger().info("🧪 Starting AprilTag availability test...")
        
        # Test 1: Import test
        self.test_imports()
        
        # Test 2: Package availability 
        self.test_packages()
        
        self.get_logger().info("✅ Test completed")

    def test_imports(self):
        """Test if we can import apriltag message types"""
        try:
            from apriltag_ros.msg import AprilTagDetectionArray
            self.get_logger().info("✅ apriltag_ros.msg import SUCCESS")
        except ImportError as e:
            self.get_logger().error(f"❌ apriltag_ros.msg import FAILED: {e}")
            
        try:
            from apriltag_msgs.msg import AprilTagDetection
            self.get_logger().info("✅ apriltag_msgs import SUCCESS") 
        except ImportError as e:
            self.get_logger().error(f"❌ apriltag_msgs import FAILED: {e}")

    def test_packages(self):
        """Test package executables"""
        import subprocess
        try:
            result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
            packages = result.stdout
            
            if 'apriltag_ros' in packages:
                self.get_logger().info("✅ apriltag_ros package FOUND")
            else:
                self.get_logger().error("❌ apriltag_ros package NOT FOUND")
                
            if 'apriltag_msgs' in packages:
                self.get_logger().info("✅ apriltag_msgs package FOUND")
            else:
                self.get_logger().error("❌ apriltag_msgs package NOT FOUND")
                
        except Exception as e:
            self.get_logger().error(f"❌ Package check failed: {e}")

def main():
    rclpy.init()
    node = SimpleAprilTagTest()
    
    # Run for a short time then exit
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
