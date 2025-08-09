# Multi-Robot Formation Control with AprilTags

This package implements a leader-follower formation control system for two JetRover robots using AprilTag visual markers for localization.

## Overview

- **Robot 1 (Leader)**: Knows its position via SLAM/AMCL, has an AprilTag mounted on its back
- **Robot 2 (Follower)**: Uses camera to detect the leader's AprilTag, localizes itself relative to the leader, then maintains formation while avoiding obstacles

## Key Features

- AprilTag-based relative localization
- Coordinate transformation to shared global map frame
- Formation control maintaining 50cm following distance
- Obstacle avoidance while maintaining formation
- Recovery behavior when AprilTag is lost
- Multi-robot namespace support

## Hardware Requirements

### AprilTag Setup
1. Print an AprilTag from the `tag36h11` family (ID 0 recommended)
2. Mount the tag on the back of the leader robot
3. Default tag size is 10cm x 10cm (configurable)
4. Mount height: ~15cm above ground level

### Camera Setup
- Follower robot needs a working camera (USB camera or depth camera)
- Camera should be mounted facing forward
- Ensure camera topics are publishing correctly

## Installation

1. Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-apriltag-ros python3-opencv python3-apriltag
```

2. Build the workspace:
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select multi_robot_formation
source install/setup.bash
```

## Usage

### Method 1: Environment Variables (Matches Your JetRover System) ⭐ RECOMMENDED
Your JetRover system uses HOST and MASTER environment variables. Use this method:

```bash
# Set environment variables (on follower robot)
export HOST=follower      # This robot's namespace
export MASTER=leader      # Leader robot's namespace

# Launch formation control
ros2 launch multi_robot_formation formation_env.launch.py
```

Optional parameters:
- `formation_distance:=0.5` - Following distance in meters
- `tag_id:=0` - AprilTag ID to track

### Method 2: Direct Namespace Parameters
If you want to override environment variables:

```bash
ros2 launch multi_robot_formation follower_only.launch.py \
    follower_name:=robot2 \
    leader_name:=robot1
```

### Method 3: Full Multi-Robot Launch
Launches both leader and follower robots with complete navigation stack:

```bash
ros2 launch multi_robot_formation multi_robot_formation.launch.py
```

### Manual Node Launch
For debugging or custom setups:

```bash
# AprilTag detector
ros2 run multi_robot_formation apriltag_detector --ros-args -p robot_namespace:=follower

# Follower localization  
ros2 run multi_robot_formation follower_localization --ros-args -p robot_namespace:=follower -p leader_namespace:=leader

# Formation controller
ros2 run multi_robot_formation formation_controller --ros-args -p robot_namespace:=follower -p leader_namespace:=leader
```

## Configuration

Edit `config/formation_params.yaml` to adjust:

- **Formation distance**: How far behind the leader to follow
- **Tag detection parameters**: Tag family, size, camera frame
- **Control parameters**: Max velocities, PID gains, tolerances
- **Obstacle avoidance**: Distance thresholds, behavior settings

## Topics

### Subscribed Topics
- `/{follower}/camera/image_raw` - Camera feed for AprilTag detection
- `/{follower}/camera/camera_info` - Camera calibration data
- `/{follower}/scan` - Laser scan for obstacle avoidance
- `/{follower}/estimated_pose` - Follower's estimated pose

### Published Topics
- `/{follower}/cmd_vel` - Velocity commands to follower robot
- `/{follower}/leader_tag_pose` - Detected AprilTag pose
- `/{follower}/formation_goal` - Desired formation position
- `/{follower}/initialpose` - Initial pose for AMCL localization
- `/{follower}/apriltag_debug_image` - Debug image with detected tags

## Monitoring and Debugging

### Visualization in RViz
1. Add topics to visualize formation:
   - `/{follower}/formation_goal` (PoseStamped)
   - `/{follower}/estimated_pose` (PoseStamped)
   - TF frames for both robots

2. Check AprilTag detection:
   - `/{follower}/apriltag_debug_image` (Image)

### Common Issues

**AprilTag not detected:**
- Check camera is working: `ros2 topic echo /{follower}/camera/image_raw`
- Verify tag is visible and properly lit
- Check tag family and ID match configuration
- Ensure tag size parameter is correct

**Poor formation control:**
- Verify both robots are localized in the same map frame
- Check TF tree is complete: `ros2 run tf2_tools view_frames`
- Adjust PID parameters in config file
- Monitor formation goal vs actual position

**Obstacle avoidance issues:**
- Check laser scan data: `ros2 topic echo /{follower}/scan`
- Adjust obstacle distance threshold
- Verify laser frame is correctly configured

## Parameters Reference

### AprilTag Detector
- `tag_family`: AprilTag family (default: "tag36h11")
- `leader_tag_id`: Tag ID to track (default: 0)
- `tag_size`: Physical tag size in meters (default: 0.10)
- `camera_frame`: Camera frame name

### Formation Controller  
- `formation_distance`: Target following distance (default: 0.5m)
- `formation_angle`: Angle relative to leader (default: π = behind)
- `max_linear_velocity`: Max forward speed (default: 0.3 m/s)
- `max_angular_velocity`: Max rotation speed (default: 1.0 rad/s)
- `obstacle_distance_threshold`: Obstacle avoidance trigger distance

### Localization
- `tag_to_robot_offset_x`: Tag X offset from robot center (default: -0.20m)
- `tag_to_robot_offset_y`: Tag Y offset from robot center (default: 0.0m)  
- `tag_to_robot_offset_z`: Tag height above ground (default: 0.15m)

## Development

To extend or modify the formation behavior:

1. **Custom formation patterns**: Modify `calculate_formation_goal()` in `formation_controller.py`
2. **Advanced obstacle avoidance**: Enhance `apply_obstacle_avoidance()` method
3. **Recovery behaviors**: Implement search patterns in `handle_tag_lost()`
4. **Multiple followers**: Extend system to support N-robot formations

## Troubleshooting

Check system status:
```bash
# Verify nodes are running
ros2 node list | grep formation

# Check topic connections
ros2 topic list | grep -E "(tag_pose|cmd_vel|formation)"

# Monitor TF transforms
ros2 run tf2_ros tf2_echo map leader/base_footprint
ros2 run tf2_ros tf2_echo map follower/base_footprint
```

Enable debug output:
```bash
ros2 param set /follower/formation_controller ros.logging.level DEBUG
```
