#!/bin/bash

echo "🔧 Installing Multi-Robot Formation Dependencies"
echo "=============================================="

# Update package lists
sudo apt update

# Install ROS2 packages for AprilTag detection
echo "📦 Installing ROS2 AprilTag packages..."
sudo apt install -y \
    ros-humble-apriltag-ros \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins

# Optional: Install Python AprilTag library for backup
echo "📦 Installing optional Python libraries..."
sudo apt install -y python3-pip
pip3 install opencv-python

# Try to install Python apriltag (may fail, but that's OK)
echo "📦 Trying to install Python apriltag (optional)..."
pip3 install apriltag || echo "   Python apriltag failed - will use ROS apriltag_ros instead"

echo "✅ Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Build the package: colcon build --packages-select multi_robot_formation"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Set environment variables:"
echo "   export HOST=robot_1"
echo "   export MASTER=robot_1"
echo "4. Test launch: ros2 launch multi_robot_formation formation_with_ros_apriltag.launch.py"
