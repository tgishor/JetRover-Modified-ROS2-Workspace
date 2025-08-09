#!/bin/bash

echo "🔧 Installing Multi-Robot Formation Dependencies"
echo "=============================================="

# Update package lists
sudo apt update

# Install AprilTag detection libraries
echo "📦 Installing AprilTag libraries..."
sudo apt install -y python3-pip
pip3 install apriltag opencv-python

# Install ROS2 packages
echo "📦 Installing ROS2 packages..."
sudo apt install -y \
    ros-humble-apriltag-ros \
    ros-humble-cv-bridge \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs

echo "✅ Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Build the package: colcon build --packages-select multi_robot_formation"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Set environment variables:"
echo "   export HOST=robot_1"
echo "   export MASTER=robot_1"
echo "4. Test launch: ros2 launch multi_robot_formation test_formation.launch.py"
