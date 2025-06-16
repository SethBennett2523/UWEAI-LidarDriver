#!/bin/bash

# Hesai LiDAR Mapping Demonstration Launcher
# 
# WARNING: This script launches the DEMONSTRATION version with extended
# point persistence. This is NOT suitable for real-time applications.

echo ""
echo "========================================="
echo "  HESAI LIDAR MAPPING DEMONSTRATION"
echo "========================================="
echo ""
echo "⚠️  WARNING: DEMONSTRATION MODE ONLY"
echo "⚠️  Extended point persistence (60s)"
echo "⚠️  NOT for real-time applications"
echo ""
echo "This will launch:"
echo "- Hesai demo driver with timing warnings"
echo "- RViz with mapping configuration"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "========================================="
echo ""

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch in parallel using the launch file
ros2 launch hesai_lidar_driver hesai_mapping_demo.launch.py
