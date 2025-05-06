#!/usr/bin/env python3
"""
Simple script to launch the visualization for a robot G-code file.
This script can be used after processing a G-code file to visualize the results.
"""

import os
import sys
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description='Launch visualization for robot G-code')
    parser.add_argument('file', help='Path to robot G-code file (.robot.gcode will be appended if not present)')
    args = parser.parse_args()
    
    # Ensure the file has the correct extension
    gcode_path = args.file
    if not gcode_path.endswith('.robot.gcode'):
        gcode_path += '.robot.gcode'
    
    # Check if file exists
    if not os.path.exists(gcode_path):
        print(f"Error: Robot G-code file not found: {gcode_path}")
        sys.exit(1)
    
    # Launch the tool path visualizer and RViz2
    print(f"Launching visualization for: {gcode_path}")
    
    # Start the tool path visualizer
    visualizer_proc = subprocess.Popen(['ros2', 'run', 'noether_gateway', 'tool_path_visualizer', gcode_path])
    
    # Start RViz2 with the correct configuration
    rviz_proc = subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', 
                                 os.path.join(os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0], 
                                             'share/noether_gateway/config/tool_paths.rviz')])
    
    # Start static transform publisher
    tf_proc = subprocess.Popen(['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                               '0', '0', '0', '0', '0', '0', 'world', 'map'])
    
    print("Visualization launched. Press Ctrl+C to exit...")
    try:
        visualizer_proc.wait()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
        for proc in [visualizer_proc, rviz_proc, tf_proc]:
            if proc.poll() is None:
                proc.terminate()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())