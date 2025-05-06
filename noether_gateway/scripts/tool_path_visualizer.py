#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import argparse
import re
import os

class ToolPathVisualizer(Node):
    def __init__(self, robot_gcode=None, scale=0.001):
        super().__init__('tool_path_visualizer')
        
        # Store scale factor for unit conversion
        self.scale = scale
        self.get_logger().info(f"Using scale factor: {self.scale} (input units to meters)")
        
        # Debug level
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        # Path visualization publishers
        self.path_marker_pub = self.create_publisher(MarkerArray, '/tool_paths', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/tool_poses', 10)
        self.point_cloud_pub = self.create_publisher(Marker, '/surface_points', 10)
        
        # Load gcode if provided
        self.robot_gcode = robot_gcode
        self.tool_paths = []
        
        # Create a timer for periodic visualization updates
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Tool Path Visualizer started")
        
        # If we have a G-code file, parse it
        if self.robot_gcode:
            self.get_logger().info(f"Loading G-code from {self.robot_gcode}")
            # Dump the first few lines of the G-code file for debugging
            self.dump_gcode_file()
            self.load_gcode(self.robot_gcode)
    
    def dump_gcode_file(self):
        """Dump the contents of the G-code file for debugging"""
        try:
            self.get_logger().info(f"G-code file contents (first 20 lines):")
            with open(self.robot_gcode, 'r') as f:
                lines = f.readlines()
                for i, line in enumerate(lines[:20]):
                    self.get_logger().info(f"Line {i}: {line.strip()}")
                if len(lines) > 20:
                    self.get_logger().info(f"... ({len(lines)} lines total)")
        except Exception as e:
            self.get_logger().error(f"Error reading G-code file: {e}")
    
    def extract_scale_from_gcode(self):
        """Try to extract scale factor from G-code comments"""
        try:
            with open(self.robot_gcode, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if '; Scale factor:' in line:
                        parts = line.split(':')
                        if len(parts) > 1:
                            scale_str = parts[1].strip()
                            if '(' in scale_str:
                                scale_str = scale_str.split('(')[0].strip()
                            try:
                                scale = float(scale_str)
                                self.get_logger().info(f"Extracted scale factor from G-code: {scale}")
                                return scale
                            except ValueError:
                                pass
        except Exception as e:
            self.get_logger().error(f"Error extracting scale: {e}")
        return None
            
    def load_gcode(self, gcode_path):
        """Load a robot G-code file and convert to tool paths"""
        self.get_logger().info(f"Parsing G-code file: {gcode_path}")
        
        # Try to extract scale from G-code file
        extracted_scale = self.extract_scale_from_gcode()
        if extracted_scale is not None:
            self.scale = extracted_scale
            self.get_logger().info(f"Using extracted scale factor: {self.scale}")
        
        try:
            tool_paths = []
            current_path = []
            x = y = z = 0.0
            
            # Regular expressions for G-code parsing
            g_code_re = re.compile(r'G([01])\s')
            x_re = re.compile(r'X([-\d.]+)')
            y_re = re.compile(r'Y([-\d.]+)')
            z_re = re.compile(r'Z([-\d.]+)')
            
            with open(gcode_path, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    
                    # Skip comments and empty lines
                    if not line or line.startswith(';'):
                        continue
                    
                    # Extract G-code command
                    g_match = g_code_re.search(line)
                    if not g_match:
                        continue
                    
                    g_code = g_match.group(1)
                    
                    # Extract coordinates
                    x_match = x_re.search(line)
                    y_match = y_re.search(line)
                    z_match = z_re.search(line)
                    
                    # Update coordinates if present
                    if x_match:
                        x = float(x_match.group(1))
                    if y_match:
                        y = float(y_match.group(1))
                    if z_match:
                        z = float(z_match.group(1))
                    
                    # If G0 moves up (z increase), it's the end of a path
                    if g_code == '0' and z_match and len(current_path) > 0:
                        new_z = float(z_match.group(1))
                        # If z increases significantly, it's the end of a path
                        if new_z > current_path[-1][2] + 1.0:
                            self.get_logger().debug(f"Path break at line {line_num} - Z changed from {current_path[-1][2]} to {new_z}")
                            if len(current_path) > 1:  # Only add paths with multiple points
                                tool_paths.append(current_path)
                            current_path = []
                    
                    # For visualization, add both G0 and G1 moves to see everything
                    # G1 = printing move, G0 = travel move
                    current_path.append((x, y, z))
                    
                    # Debug printing of coordinates
                    self.get_logger().debug(f"Line {line_num}: G{g_code} X{x} Y{y} Z{z}")
            
            # Add the last path if not empty
            if len(current_path) > 1:
                tool_paths.append(current_path)
            
            # Assign the parsed paths to the object
            self.tool_paths = tool_paths
            
            self.get_logger().info(f"Loaded {len(self.tool_paths)} tool paths with {sum(len(p) for p in self.tool_paths)} total points")
            
            # Print the first few points of each path for debugging
            for i, path in enumerate(self.tool_paths):
                if not path:
                    continue
                self.get_logger().info(f"Path {i}: {len(path)} points")
                for j, point in enumerate(path[:3]):  # Print first 3 points
                    self.get_logger().info(f"  Point {j}: X{point[0]} Y{point[1]} Z{point[2]}")
                if len(path) > 3:
                    self.get_logger().info(f"  ... ({len(path)} points total)")
                
        except Exception as e:
            self.get_logger().error(f"Error loading G-code: {e}")
    
    def timer_callback(self):
        """Periodic callback to publish visualization markers"""
        if not self.tool_paths:
            return
            
        # Publish tool paths as marker arrays
        self.publish_path_markers()
        
        # Publish tool poses
        self.publish_tool_poses()
        
        # Publish surface points (created from tool path points)
        self.publish_surface_points()
    
    def publish_path_markers(self):
        """Publish tool paths as marker arrays"""
        marker_array = MarkerArray()
        
        for i, path in enumerate(self.tool_paths):
            if not path or len(path) < 2:  # Skip empty or single-point paths
                continue
                
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tool_paths"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set line properties
            marker.scale.x = 0.001  # Line width in meters (1mm)
            
            # Vary color based on path index for easier distinction
            h = i / max(1, len(self.tool_paths) - 1)
            color = self.hsv_to_rgb(h, 1.0, 1.0)
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            
            # Add points (convert from input units to m for RViz)
            for point in path:
                p = Point()
                p.x = point[0] * self.scale
                p.y = point[1] * self.scale
                p.z = point[2] * self.scale
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        self.path_marker_pub.publish(marker_array)
    
    def publish_tool_poses(self):
        """Publish tool poses for the tool path"""
        # Create a PoseArray with all poses from all paths
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for path in self.tool_paths:
            if not path or len(path) < 2:  # Skip empty or single-point paths
                continue
                
            for i in range(len(path)):
                # Current point
                curr = path[i]
                
                # Create pose - convert to m
                pose = Pose()
                pose.position.x = curr[0] * self.scale
                pose.position.y = curr[1] * self.scale
                pose.position.z = curr[2] * self.scale
                
                # For orientation, look at the next point if available
                if i < len(path) - 1:
                    next_pt = path[i + 1]
                    dx = next_pt[0] - curr[0]
                    dy = next_pt[1] - curr[1]
                    dz = next_pt[2] - curr[2]
                    
                    # Calculate quaternion for orientation (simplified)
                    length = np.sqrt(dx*dx + dy*dy + dz*dz)
                    if length > 0.001:
                        pose.orientation.w = 1.0
                else:
                    pose.orientation.w = 1.0
                
                pose_array.poses.append(pose)
        
        self.poses_pub.publish(pose_array)
    
    def publish_surface_points(self):
        """Publish surface points as a point cloud"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "surface_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set point properties
        marker.scale.x = 0.001  # Point size in meters (1mm)
        marker.scale.y = 0.001
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
        
        # Add points from all paths (convert to m)
        for path in self.tool_paths:
            if not path or len(path) < 2:  # Skip empty or single-point paths
                continue
                
            # Sample points (don't add all to avoid overly dense cloud)
            sample_rate = max(1, len(path) // 100)
            for i in range(0, len(path), sample_rate):
                p = Point()
                p.x = path[i][0] * self.scale
                p.y = path[i][1] * self.scale
                p.z = path[i][2] * self.scale
                marker.points.append(p)
        
        self.point_cloud_pub.publish(marker)
    
    def hsv_to_rgb(self, h, s, v):
        """Convert HSV color to RGB
        h, s, v in [0, 1]
        Returns r, g, b in [0, 1]
        """
        h_i = int(h * 6)
        f = h * 6 - h_i
        p = v * (1 - s)
        q = v * (1 - f * s)
        t = v * (1 - (1 - f) * s)
        
        if h_i == 0:
            r, g, b = v, t, p
        elif h_i == 1:
            r, g, b = q, v, p
        elif h_i == 2:
            r, g, b = p, v, t
        elif h_i == 3:
            r, g, b = p, q, v
        elif h_i == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        
        return (r, g, b)

def main(args=None):
    rclpy.init(args=args)
    
    # Parse arguments
    parser = argparse.ArgumentParser(description='Tool Path Visualizer')
    parser.add_argument('robot_gcode', nargs='?', help='Path to robot G-code file')
    parser.add_argument('--scale', type=float, default=0.001,
                       help='Scale factor for unit conversion (default: 0.001)')
    args = parser.parse_args()
    
    visualizer = ToolPathVisualizer(args.robot_gcode, args.scale)
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()