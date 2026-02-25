#!/bin/bash
# RViz2 Point Cloud Visualization Script
# Purpose: View Unitree Go2 LiDAR point cloud data in SSH environments
# Generated: $(date '+%Y-%m-%d %H:%M:%S')

echo "Unitree Go2 LiDAR Point Cloud Visualizer"
echo "========================================"

# Set up environment
source /opt/ros/foxy/setup.bash
source cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS2 environment loaded"

# Check X11 forwarding
if [ -z "$DISPLAY" ]; then
    echo "Warning: X11 forwarding not detected"
    echo "Suggestion: Reconnect using 'ssh -X' to enable the graphical interface"
    echo ""
    echo "If X11 is unavailable, choose an alternative approach:"
    echo "1. Generate static point cloud images (Option 3)"
    echo "2. Save point cloud files (Option 4)"
    echo "3. Use web visualization (Option 5)"
    echo ""
    read -p "Continue attempting to launch RViz2? (y/n): " choice
    if [ "$choice" != "y" ]; then
        echo "User cancelled operation"
        exit 1
    fi
fi

echo "Launching RViz2..."
echo "Configuration instructions:"
echo "  1. Add PointCloud2 display type"
echo "  2. Set Topic to: /utlidar/cloud"
echo "  3. Set Fixed Frame to: utlidar_lidar"
echo "  4. Adjust point cloud size and color as desired"
echo ""

# Launch RViz2
ros2 run rviz2 rviz2

echo "RViz2 has exited"
