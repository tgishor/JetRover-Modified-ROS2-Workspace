#!/usr/bin/env python3

import sys

print("🔍 Testing AprilTag Package Availability")
print("=" * 50)

# Test 1: Check if apriltag_ros package is in ROS2 package list
print("\n1. Checking ROS2 package list...")
import subprocess
try:
    result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True)
    packages = result.stdout.split('\n')
    apriltag_packages = [pkg for pkg in packages if 'apriltag' in pkg]
    
    if apriltag_packages:
        print("✅ Found AprilTag packages:")
        for pkg in apriltag_packages:
            print(f"   - {pkg}")
    else:
        print("❌ No AprilTag packages found in ros2 pkg list")
except Exception as e:
    print(f"❌ Error checking packages: {e}")

# Test 2: Try importing apriltag_ros messages
print("\n2. Testing apriltag_ros Python import...")
try:
    from apriltag_ros.msg import AprilTagDetectionArray
    print("✅ apriltag_ros.msg import successful")
except ImportError as e:
    print(f"❌ apriltag_ros.msg import failed: {e}")

# Test 3: Check apriltag_msgs
print("\n3. Testing apriltag_msgs Python import...")
try:
    from apriltag_msgs.msg import AprilTagDetection
    print("✅ apriltag_msgs import successful")
except ImportError as e:
    print(f"❌ apriltag_msgs import failed: {e}")

# Test 4: Check if apriltag_node executable exists
print("\n4. Checking apriltag_node executable...")
try:
    result = subprocess.run(['ros2', 'pkg', 'executables', 'apriltag_ros'], capture_output=True, text=True)
    if result.returncode == 0 and result.stdout.strip():
        print("✅ apriltag_ros executables found:")
        for exe in result.stdout.strip().split('\n'):
            print(f"   - {exe}")
    else:
        print("❌ No apriltag_ros executables found")
except Exception as e:
    print(f"❌ Error checking executables: {e}")

# Test 5: Check environment sourcing
print("\n5. Checking ROS environment...")
import os
cmake_prefix_path = os.environ.get('CMAKE_PREFIX_PATH', '')
ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')

print(f"CMAKE_PREFIX_PATH contains apriltag: {'apriltag' in cmake_prefix_path}")
print(f"AMENT_PREFIX_PATH contains apriltag: {'apriltag' in ament_prefix_path}")

if 'third_party' in cmake_prefix_path or 'third_party' in ament_prefix_path:
    print("✅ Third party workspace detected in environment")
else:
    print("❌ Third party workspace not in environment")

print("\n" + "=" * 50)
print("🏁 Test Complete")
