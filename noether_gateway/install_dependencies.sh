#!/bin/bash

# Install script for Noether Gateway dependencies
echo "Installing dependencies for Noether Gateway..."

# ROS2 packages
sudo apt update
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rclpy

# Python dependencies
pip install numpy scipy pyyaml

# Make sure scripts are executable
chmod +x ~/robot_ws/src/noether_gateway/scripts/*.py

echo "Dependencies installed successfully!"
echo "Next steps:"
echo "1. Build the packages:"
echo "   cd ~/robot_ws"
echo "   colcon build --packages-select noether_interfaces"
echo "   source install/setup.bash"
echo "   colcon build --packages-select noether_gateway"
echo "   source install/setup.bash"
echo ""
echo "2. Run the tool with:"
echo "   ros2 launch noether_gateway complete_pipeline.launch.py gcode_file:=/path/to/your/file.gcode"