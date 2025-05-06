'''Copyright [2025] [Walter Glockner]

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.'''


import rclpy
from rclpy.node import Node
import numpy as np
from noether_interfaces.srv import PlanSurface
from noether_interfaces.msg import ToolPath, ToolPathPose
from geometry_msgs.msg import Pose, Point, Quaternion
import logging
import yaml
import tempfile
import os
import subprocess
import importlib

# Try to import Noether Python API modules
try:
    from noether_filtering import mesh_filtering
    from noether_conversions import mesh_conversions
    from shape_msgs.msg import Mesh
    from noether_tpp import surface_raster_planner
    NOETHER_AVAILABLE = True
except ImportError:
    NOETHER_AVAILABLE = False

class Gateway(Node):
    def __init__(self, logger=None):
        super().__init__('noether_gateway')
        self.srv = self.create_service(PlanSurface, 'plan_surface', self.cb)
        self.logger = logger or logging.getLogger(self.__class__.__name__)
        self.get_logger().info("Noether Gateway node started")
        if NOETHER_AVAILABLE:
            self.get_logger().info("Noether Python API is available")
        else:
            self.get_logger().warn("Noether Python API not found. Falling back to CLI mode")
        
    def points_to_yaml(self, points):
        """Convert surface points to a YAML format Noether can process"""
        surface_data = {
            'surface': {
                'points': [[p.x, p.y, p.z] for p in points]
            },
            'planning_parameters': {
                'tool_offset': 0.0,
                'point_spacing': 0.001,
                'raster_spacing': 0.001,
                'tool_orientation': [0, 0, -1]
            }
        }
        
        # Create temporary YAML file
        with tempfile.NamedTemporaryFile(suffix='.yaml', delete=False, mode='w') as f:
            yaml.dump(surface_data, f)
            yaml_path = f.name
            
        return yaml_path
    
    def generate_tool_paths_from_surface(self, yaml_path):
        """Call Noether planning API to generate tool paths"""
        if NOETHER_AVAILABLE:
            # Use direct Python API if available
            return self.generate_paths_with_api(yaml_path)
        else:
            # Fall back to CLI method
            return self.generate_paths_with_cli(yaml_path)
        
    def generate_paths_with_api(self, yaml_path):
        """Generate tool paths using Noether Python API directly"""
        self.get_logger().info("Generating tool paths using Noether Python API")
        
        # Load configuration from YAML
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # Extract points and parameters
        points = np.array(data['surface']['points'])
        params = data['planning_parameters']
        
        # Create mesh from points
        mesh = mesh_conversions.point_cloud_to_mesh(points)
        
        # Apply mesh filtering
        filtered_mesh = mesh_filtering.filter_mesh(mesh)
        
        # Set up raster planner
        planner = surface_raster_planner.SurfaceRasterPlanner()
        planner.config.point_spacing = params['point_spacing']
        planner.config.raster_spacing = params['raster_spacing']
        planner.config.tool_offset = params['tool_offset']
        planner.config.tool_orientation = params['tool_orientation']
        
        # Generate paths
        path_results = planner.plan_paths(filtered_mesh)
        
        # Convert to ROS ToolPath messages
        tool_paths = []
        for segment in path_results.paths:
            tool_path = ToolPath()
            for waypoint in segment.waypoints:
                pose = ToolPathPose()
                pose.pose = waypoint.pose
                pose.feed = waypoint.linear_speed  # Assuming the API provides this
                tool_path.poses.append(pose)
            tool_paths.append(tool_path)
        
        self.get_logger().info(f"Generated {len(tool_paths)} tool paths with API")
        return tool_paths
        
    def generate_paths_with_cli(self, yaml_path):
        """Generate tool paths using Noether CLI"""
        self.get_logger().info("Generating tool paths using Noether CLI")
        
        output_path = yaml_path + '.out.yaml'
        
        # Check if the CLI tool exists before trying to run it
        import shutil
        if not shutil.which('noether_planning_tool'):
            self.get_logger().warn("Noether CLI tool not found in PATH. Skipping CLI method.")
            return self.generate_paths_from_points(yaml_path)
        
        # Actual Noether CLI call
        try:
            result = subprocess.run(
                ['noether_planning_tool', yaml_path, '--output', output_path],
                check=True,
                capture_output=True,
                text=True
            )
            self.get_logger().info(f"Noether CLI output: {result.stdout}")
            
            # Parse the output YAML
            if os.path.exists(output_path):
                with open(output_path, 'r') as f:
                    output_data = yaml.safe_load(f)
                
                # Convert YAML output to ToolPath messages
                tool_paths = []
                if 'tool_paths' in output_data:
                    for path_data in output_data['tool_paths']:
                        path = ToolPath()
                        for pose_data in path_data['poses']:
                            pose = ToolPathPose()
                            pose.pose = Pose()
                            pose.pose.position = Point(
                                x=pose_data['position']['x'],
                                y=pose_data['position']['y'],
                                z=pose_data['position']['z']
                            )
                            pose.pose.orientation = Quaternion(
                                x=pose_data['orientation']['x'],
                                y=pose_data['orientation']['y'],
                                z=pose_data['orientation']['z'],
                                w=pose_data['orientation']['w']
                            )
                            pose.feed = pose_data.get('feed', 50.0)
                            path.poses.append(pose)
                        tool_paths.append(path)
                
                os.unlink(output_path)  # Clean up
                return tool_paths
            else:
                self.get_logger().error(f"Output file {output_path} not found")
                # Fall back to point-based method
                return self.generate_paths_from_points(yaml_path)
                
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            self.get_logger().error(f"CLI call failed: {e}")
            # Fall back to point-based method if CLI fails
            return self.generate_paths_from_points(yaml_path)
    
    def generate_paths_from_points(self, yaml_path):
        """Generate real tool paths from input points"""
        # Load the points from the YAML file
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        points = np.array(data['surface']['points'])
        
        # Get parameters
        raster_spacing = data['planning_parameters']['raster_spacing']
        point_spacing = data['planning_parameters']['point_spacing']
        tool_offset = data['planning_parameters'].get('tool_offset', 0.0)
        
        # Skip triangulation and go directly to grid-based path generation
        self.get_logger().info("Using direct grid-based path generation")
        tool_paths = self.generate_grid_paths(points, data['planning_parameters'])
        
        return tool_paths
        
        def create_raster_paths(self, points, triangulation, normals, raster_spacing, point_spacing, tool_offset):
            """Create raster paths based on height map instead of triangulation"""
            self.get_logger().info("Using simplified raster path generation for robustness")
            tool_paths = []
            
            # Get bounds of the surface
            x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
            y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
            
            # Create a 2D grid for the height map
            x_grid = np.arange(x_min, x_max, point_spacing)
            y_grid = np.arange(y_min, y_max, raster_spacing)
            
            # For each raster line
            for i, y in enumerate(y_grid):
                path = ToolPath()
                # Alternate direction for each line (zig-zag pattern)
                direction = 1 if i % 2 == 0 else -1
                x_values = x_grid if direction == 1 else x_grid[::-1]
                
                for x in x_values:
                    # Find nearest point in the original point cloud
                    point_2d = np.array([x, y])
                    
                    # Calculate distances to all points (only using x,y dimensions)
                    distances = np.sqrt(np.sum((points[:, :2] - point_2d)**2, axis=1))
                    
                    # Get nearest points (top 3)
                    nearest_indices = np.argsort(distances)[:3]
                    
                    # Skip if no close points found
                    if len(nearest_indices) == 0 or distances[nearest_indices[0]] > raster_spacing * 2:
                        continue
                        
                    # Weighted average of z values based on distance
                    weights = 1.0 / (distances[nearest_indices] + 0.0001)  # Avoid division by zero
                    weights = weights / np.sum(weights)  # Normalize weights
                    z = np.sum(weights * points[nearest_indices, 2])
                    
                    # Create pose
                    pose = ToolPathPose()
                    pose.pose = Pose()
                    pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
                    
                    # Simple orientation - just pointing downward
                    pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    pose.feed = 50.0  # Default feed rate in mm/s
                    
                    path.poses.append(pose)
                
                # Only add non-empty paths
                if path.poses:
                    tool_paths.append(path)
            
            self.get_logger().info(f"Created {len(tool_paths)} raster tool paths")
            return tool_paths
    
    def generate_grid_paths(self, points, params):
        """Fallback method to generate simple grid-based paths"""
        self.get_logger().info("Using simple grid-based path generation")
        
        # Simple algorithm to create a raster pattern
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        z_avg = np.mean(points[:, 2])
        
        raster_spacing = params['raster_spacing']
        point_spacing = params['point_spacing']
        
        tool_paths = []
        
        # Create a simple raster pattern
        y_values = np.arange(y_min, y_max, raster_spacing)
        for i, y in enumerate(y_values):
            path = ToolPath()
            direction = 1 if i % 2 == 0 else -1
            x_range = np.arange(x_min, x_max, point_spacing) if direction == 1 else np.arange(x_max, x_min, -point_spacing)
            
            for x in x_range:
                # Find closest z value from original points
                distances = np.sqrt((points[:, 0] - x)**2 + (points[:, 1] - y)**2)
                closest_idx = np.argmin(distances)
                if distances[closest_idx] < raster_spacing:
                    z = points[closest_idx, 2]
                else:
                    z = z_avg
                
                pose = ToolPathPose()
                pose.pose = Pose()
                pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
                pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                pose.feed = 50.0  # Default feed rate in mm/s
                
                path.poses.append(pose)
            
            if path.poses:  # Only add paths that have poses
                tool_paths.append(path)
        
        return tool_paths
    
    def cb(self, req, resp):
        """Callback for the plan_surface service"""
        self.get_logger().info(f"Received planning request with {len(req.surface_points)} points")
        
        try:
            # 1. Convert points to YAML
            yaml_path = self.points_to_yaml(req.surface_points)
            self.get_logger().info(f"Created temporary YAML at {yaml_path}")
            
            # 2. Generate tool paths
            tool_paths = self.generate_tool_paths_from_surface(yaml_path)
            self.get_logger().info(f"Generated {len(tool_paths)} tool paths")
            
            # 3. Fill response
            resp.tool_paths = tool_paths
            
            # Clean up temporary file
            os.unlink(yaml_path)
            
            return resp
        except Exception as e:
            self.get_logger().error(f"Error in planning service: {e}")
            # Return empty response on error
            return resp

def main(args=None):
    rclpy.init(args=args)
    node = Gateway()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Interrupted')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
