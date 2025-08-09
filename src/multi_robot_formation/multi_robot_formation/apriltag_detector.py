#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer

# Try to import apriltag_ros messages
try:
    from apriltag_ros.msg import AprilTagDetectionArray
    ROS_APRILTAG_AVAILABLE = True
except ImportError:
    print("apriltag_ros messages not found")
    ROS_APRILTAG_AVAILABLE = False
    AprilTagDetectionArray = None

try:
    from apriltag import apriltag  # Use same import as your working AR app
    PYTHON_APRILTAG_AVAILABLE = True
except ImportError:
    print("Python apriltag library not found, will use ROS apriltag_ros instead")
    apriltag = None
    PYTHON_APRILTAG_AVAILABLE = False


class AprilTagDetector(Node):
    """
    AprilTag detector node for follower robot.
    Detects AprilTags on the leader robot and publishes pose information.
    """

    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Parameters
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('leader_tag_id', 0)
        self.declare_parameter('tag_size', 0.10)  # 10cm tag size in meters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('robot_namespace', 'follower')
        
        self.tag_family = self.get_parameter('tag_family').get_parameter_value().string_value
        self.leader_tag_id = self.get_parameter('leader_tag_id').get_parameter_value().integer_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # Initialize AprilTag detector (same as your working AR app)
        if PYTHON_APRILTAG_AVAILABLE:
            try:
                self.detector = apriltag(self.tag_family)  # Use same syntax as your AR app
                self.use_python_apriltag = True
                self.get_logger().info("Using Python apriltag library (AR app style)")
            except Exception as e:
                self.get_logger().warn(f"Python apriltag failed: {e}, falling back to ROS apriltag_ros")
                self.use_python_apriltag = False
        else:
            self.use_python_apriltag = False
            self.get_logger().info("Using ROS apriltag_ros package")
        
        # Check if ROS apriltag is available when needed
        if not self.use_python_apriltag and not ROS_APRILTAG_AVAILABLE:
            self.get_logger().error("Neither Python apriltag nor ROS apriltag_ros available!")
            raise ImportError("No AprilTag detection method available")
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Clean namespace (remove leading/trailing slashes)
        clean_namespace = self.robot_namespace.strip('/')
        
        # Subscribers
        if self.use_python_apriltag:
            # Subscribe to raw image for Python detection (use actual camera topics)
            self.image_sub = self.create_subscription(
                Image,
                '/depth_cam/rgb/image_raw',  # Use actual camera topic
                self.image_callback,
                10
            )
            
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/depth_cam/rgb/camera_info',  # Use actual camera topic  
                self.camera_info_callback,
                10
            )
        else:
            # Subscribe to apriltag_ros detections
            if ROS_APRILTAG_AVAILABLE:
                self.apriltag_sub = self.create_subscription(
                    AprilTagDetectionArray,
                    f'/{clean_namespace}/tag_detections',
                    self.apriltag_ros_callback,
                    10
                )
        
        # Publishers
        self.tag_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{clean_namespace}/leader_tag_pose',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            f'/{clean_namespace}/apriltag_debug_image',
            10
        )
        
        self.get_logger().info(f'AprilTag detector initialized for {self.robot_namespace}')
        self.get_logger().info(f'Looking for tag family: {self.tag_family}, ID: {self.leader_tag_id}')

    def camera_info_callback(self, msg):
        """Update camera intrinsics from camera info."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def apriltag_ros_callback(self, msg):
        """Handle AprilTag detections from apriltag_ros package."""
        for detection in msg.detections:
            if detection.id[0] == self.leader_tag_id:
                # Convert detection to our pose format
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose = detection.pose.pose.pose
                
                # Publish the pose
                self.tag_pose_pub.publish(pose_msg)
                
                self.get_logger().debug(
                    f'Leader tag detected via ROS: x={pose_msg.pose.position.x:.3f}, '
                    f'y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}'
                )
                break

    def image_callback(self, msg):
        """Process incoming camera images to detect AprilTags (Python apriltag only)."""
        if not self.use_python_apriltag:
            return
            
        if self.camera_matrix is None:
            self.get_logger().warn('Camera info not received yet')
            return
            
        try:
            # Convert ROS image to OpenCV (same as AR app)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            rgb_image = np.array(cv_image, dtype=np.uint8)
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            
            # Detect AprilTags (same method as your AR app)
            results = self.detector.detect(gray)
            
            # Process each detected tag (handle dictionary format like AR app)
            for detection in results:
                # Handle dictionary format (like your AR app)
                if isinstance(detection, dict):
                    tag_id = detection.get('id', -1)
                    if tag_id == self.leader_tag_id:
                        self.process_leader_tag_dict(detection, cv_image, msg.header)
                else:
                    # Handle object format
                    if hasattr(detection, 'tag_id') and detection.tag_id == self.leader_tag_id:
                        self.process_leader_tag(detection, cv_image, msg.header)
                    
            # Publish debug image
            if results:
                debug_image = self.draw_detections(cv_image, results)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_leader_tag_dict(self, detection, image, header):
        """Process the detected leader tag from dictionary format (like AR app)."""
        try:
            # Get tag data from dictionary (same format as your AR app)
            tag_center = detection['center']
            tag_corners = detection['lb-rb-rt-lt']  # Same as AR app
            
            # Extract corners: lb, rb, rt, lt
            lb = tag_corners[0]  # left-bottom
            rb = tag_corners[1]  # right-bottom  
            rt = tag_corners[2]  # right-top
            lt = tag_corners[3]  # left-top
            
            # Create corners array (same as AR app line 221)
            corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
            
            # Define 3D object points for the tag (same as AR app OBJP)
            OBJP = np.array([[-1, -1,  0],
                             [ 1, -1,  0], 
                             [-1,  1,  0],
                             [ 1,  1,  0],
                             [ 0,  0,  0]], dtype=np.float32)
            
            # Scale by tag size
            OBJP = OBJP * (self.tag_size / 2.0)
            
            # Solve PnP (same as AR app line 225)
            ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_matrix, self.dist_coeffs)
            
            if ret:
                # Convert to pose message
                pose_msg = PoseStamped()
                pose_msg.header = header
                pose_msg.header.frame_id = self.camera_frame
                
                # Position
                pose_msg.pose.position.x = float(tvecs[0][0])
                pose_msg.pose.position.y = float(tvecs[1][0])
                pose_msg.pose.position.z = float(tvecs[2][0])
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvecs)
                quat = self.rotation_matrix_to_quaternion(rotation_matrix)
                
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                
                # Publish the pose
                self.tag_pose_pub.publish(pose_msg)
                
                self.get_logger().debug(
                    f'Leader tag detected (ID {detection.get("id", -1)}): '
                    f'x={tvecs[0][0]:.3f}, y={tvecs[1][0]:.3f}, z={tvecs[2][0]:.3f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing leader tag dict: {str(e)}')

    def process_leader_tag(self, detection, image, header):
        """Process the detected leader tag and estimate its pose."""
        try:
            # Get tag corners
            corners = detection.corners.reshape(-1, 2)
            
            # Define 3D object points for the tag (centered at origin)
            half_size = self.tag_size / 2.0
            object_points = np.array([
                [-half_size, -half_size, 0],
                [ half_size, -half_size, 0],
                [ half_size,  half_size, 0],
                [-half_size,  half_size, 0]
            ], dtype=np.float32)
            
            # Solve PnP to get pose
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                corners,
                self.camera_matrix,
                self.dist_coeffs
            )
            
            if success:
                # Convert to pose message
                pose_msg = PoseStamped()
                pose_msg.header = header
                pose_msg.header.frame_id = self.camera_frame
                
                # Position
                pose_msg.pose.position.x = float(tvec[0][0])
                pose_msg.pose.position.y = float(tvec[1][0])
                pose_msg.pose.position.z = float(tvec[2][0])
                
                # Convert rotation vector to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quat = self.rotation_matrix_to_quaternion(rotation_matrix)
                
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                
                # Publish the pose
                self.tag_pose_pub.publish(pose_msg)
                
                self.get_logger().debug(
                    f'Leader tag detected at: x={tvec[0][0]:.3f}, '
                    f'y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error processing leader tag: {str(e)}')

    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2  # s = 4 * qw
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # s = 4 * qx
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # s = 4 * qy
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # s = 4 * qz
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
            
        return [qx, qy, qz, qw]

    def draw_detections(self, image, detections):
        """Draw detection results on image for debugging."""
        debug_image = image.copy()
        
        for detection in detections:
            if isinstance(detection, dict):
                # Handle dictionary format (like AR app)
                tag_corners = detection['lb-rb-rt-lt']
                tag_center = detection['center']
                tag_id = detection.get('id', -1)
                
                # Draw corners
                for i in range(4):
                    pt1 = tuple(tag_corners[i].astype(int))
                    pt2 = tuple(tag_corners[(i + 1) % 4].astype(int))
                    cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)
                
                # Draw center and ID
                center = tuple(tag_center.astype(int))
                cv2.circle(debug_image, center, 5, (0, 0, 255), -1)
                cv2.putText(debug_image, str(tag_id), 
                           (center[0] - 10, center[1] + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                # Highlight leader tag
                if tag_id == self.leader_tag_id:
                    cv2.rectangle(debug_image, 
                                (center[0] - 50, center[1] - 50),
                                (center[0] + 50, center[1] + 50),
                                (0, 255, 255), 3)
                    cv2.putText(debug_image, 'LEADER', 
                               (center[0] - 30, center[1] - 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # Handle object format
                if hasattr(detection, 'corners'):
                    corners = detection.corners.astype(int)
                    for i in range(4):
                        pt1 = tuple(corners[i])
                        pt2 = tuple(corners[(i + 1) % 4])
                        cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)
                    
                    # Draw center and ID
                    center = tuple(detection.center.astype(int))
                    cv2.circle(debug_image, center, 5, (0, 0, 255), -1)
                    cv2.putText(debug_image, str(detection.tag_id), 
                               (center[0] - 10, center[1] + 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    
                    # Highlight leader tag
                    if detection.tag_id == self.leader_tag_id:
                        cv2.rectangle(debug_image, 
                                    (center[0] - 50, center[1] - 50),
                                    (center[0] + 50, center[1] + 50),
                                    (0, 255, 255), 3)
                        cv2.putText(debug_image, 'LEADER', 
                                   (center[0] - 30, center[1] - 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return debug_image


def main(args=None):
    rclpy.init(args=args)
    
    node = AprilTagDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
