#!/bin/bash

# Multi-Robot Formation Setup Script
# This script helps you set up the formation control system

echo "🤖 JetRover Multi-Robot Formation Setup"
echo "======================================="

# Check if we're in a ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ERROR: ROS2 not sourced. Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
fi

echo "✅ ROS2 environment: $ROS_DISTRO"

# Check current environment variables
echo ""
echo "Current Environment Variables:"
echo "  HOST: ${HOST:-'Not set'}"
echo "  MASTER: ${MASTER:-'Not set'}"

# Prompt for robot configuration
echo ""
echo "Robot Configuration:"
echo "1. I am the LEADER robot (creates map)"
echo "2. I am the FOLLOWER robot (follows leader)"
echo ""
read -p "Which robot is this? (1/2): " robot_type

case $robot_type in
    1)
        echo ""
        echo "🎯 LEADER Robot Setup"
        echo "===================="
        read -p "Enter this robot's name [leader]: " leader_name
        leader_name=${leader_name:-leader}
        
        echo "Setting environment variables:"
        echo "export HOST=$leader_name"
        echo "export MASTER=$leader_name"
        echo ""
        echo "To make permanent, add to ~/.bashrc:"
        echo "echo 'export HOST=$leader_name' >> ~/.bashrc"
        echo "echo 'export MASTER=$leader_name' >> ~/.bashrc"
        echo ""
        echo "Start SLAM with:"
        echo "ros2 launch slam slam.launch.py"
        ;;
        
    2)
        echo ""
        echo "🎯 FOLLOWER Robot Setup" 
        echo "======================"
        read -p "Enter this robot's name [follower]: " follower_name
        follower_name=${follower_name:-follower}
        
        read -p "Enter leader robot's name [leader]: " leader_name
        leader_name=${leader_name:-leader}
        
        read -p "Enter following distance in meters [0.5]: " distance
        distance=${distance:-0.5}
        
        read -p "Enter AprilTag ID [0]: " tag_id
        tag_id=${tag_id:-0}
        
        echo ""
        echo "Setting environment variables:"
        echo "export HOST=$follower_name"
        echo "export MASTER=$leader_name"
        echo ""
        echo "To make permanent, add to ~/.bashrc:"
        echo "echo 'export HOST=$follower_name' >> ~/.bashrc"
        echo "echo 'export MASTER=$leader_name' >> ~/.bashrc"
        echo ""
        echo "Start formation control with:"
        echo "ros2 launch multi_robot_formation formation_env.launch.py formation_distance:=$distance tag_id:=$tag_id"
        ;;
        
    *)
        echo "❌ Invalid selection"
        exit 1
        ;;
esac

echo ""
echo "📋 AprilTag Setup Reminder:"
echo "=========================="
echo "• Print tag36h11 AprilTag with ID $tag_id"
echo "• Size: 10cm x 10cm"
echo "• Mount on BACK of leader robot"
echo "• Height: ~15cm above ground"
echo "• Ensure follower robot's camera can see it"

echo ""
echo "🚀 Ready to start formation control!"
