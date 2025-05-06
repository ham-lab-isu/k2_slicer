#!/usr/bin/env python3
"""
Noether MoveIt Task Constructor Integration with Improved Planning

A fixed and improved version of the Task Constructor that handles path planning
more effectively and provides better debugging information.

Usage:
    ros2 run noether_gateway noether_task_constructor
    ros2 launch noether_gateway noether_task_constructor.launch.py input_file:=path/to/file.robot.gcode
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import os
import pathlib
import time
import numpy as np
from typing import List, Dict, Any, Optional, Tuple

# Standard ROS2 messages
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

# MoveIt2 messages and services
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import (
    RobotState, Constraints, PositionConstraint, OrientationConstraint,
    RobotTrajectory, MotionPlanRequest, WorkspaceParameters,
    PlanningScene, PlanningOptions, JointConstraint
)
from shape_msgs.msg import SolidPrimitive

# Noether custom messages
from noether_interfaces.msg import ToolPath, ToolPathPose
import math



def make_q(x, y, z, w):
    q = Quaternion()
    q.x, q.y, q.z, q.w = x, y, z, w
    return q

def normalize(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) or 1.0
    q.x, q.y, q.z, q.w = q.x/n, q.y/n, q.z/n, q.w/n
    return q

# Configuration
DEFAULT_SCALE = 0.001             # mm to m conversion
PLANNING_GROUP_NAME = "khi_cx110l"
TCP_LINK = "khi_cx110l_link6"
MAX_VELOCITY = 0.1                # Default execution velocity (m/s)
TRAVEL_VELOCITY = 0.5             # Velocity for travel moves (m/s)
Z_JUMP_THRESHOLD = 0.005          # Threshold for detecting travel moves
ROBOT_FRAME = "world"             # Reference frame for your robot (usually "world" or "base_link")

ROOT2_INV = 1 / math.sqrt(2.0)

DEFAULT_ORIENTATIONS = [
    make_q(0.0,        0.0,        0.0,        1.0),
    make_q(0.0,        ROOT2_INV,  0.0,        ROOT2_INV),
    make_q(ROOT2_INV,  0.0,        0.0,        ROOT2_INV),
    make_q(0.0,        0.0,        ROOT2_INV,  ROOT2_INV),
    make_q(0.5,        0.5,        0.5,        0.5),        # already unit
]

class NoetherTaskConstructor(Node):
    """
    Integrates Noether path planning with MoveIt Task Constructor
    to plan and execute surface processing paths.
    """
    def __init__(self):
        super().__init__('noether_task_constructor')
        
        # Declare and get ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_file', '/home/wglockner/.config/OrcaSlicer/log/Cube4.robot.gcode'),
                ('execute', True),
                ('visualize', True),
                ('scale', DEFAULT_SCALE),
                ('planning_group', PLANNING_GROUP_NAME),
                ('tcp_link', TCP_LINK),
                ('robot_frame', ROBOT_FRAME)
            ]
        )
        
        # Get parameters
        self.input_file = self.get_parameter('input_file').value
        self.execute_trajectories = self.get_parameter('execute').value
        self.visualize_paths = self.get_parameter('visualize').value
        self.scale = self.get_parameter('scale').value
        self.planning_group = self.get_parameter('planning_group').value
        self.tcp_link = self.get_parameter('tcp_link').value
        self.robot_frame = self.get_parameter('robot_frame').value
        
        # Set up logging
        self.get_logger().info("Initializing Noether Task Constructor (Improved Planning)")
        self.get_logger().info(f"Parameters: input_file={self.input_file}, execute={self.execute_trajectories}, visualize={self.visualize_paths}, scale={self.scale}")
        self.get_logger().info(f"Robot config: planning_group={self.planning_group}, tcp_link={self.tcp_link}, frame={self.robot_frame}")
        
        # Initialize MoveIt2 interfaces
        self.init_moveit_interfaces()
        
        # Initialize visualization publishers
        self.init_visualization()
        
        # Storage for tool paths and trajectories
        self.tool_paths = []
        self.planned_trajectories = []
        
        # Store robot state
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Load tool paths if file is provided
        if self.input_file:
            self.load_from_file(self.input_file)
        
        # Set up timer for visualization
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        # Wait for robot state to be available
        timeout = 10.0  # seconds
        start_time = time.time()
        while self.current_joint_state is None:
            rclpy.spin_once(self)  # No timeout parameter 
            time.sleep(0.1)        # Sleep instead
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout waiting for joint states. Continuing without initial state.")
                break
        
        self.get_logger().info("Noether Task Constructor initialized")
        
        # Plan all tool paths
        if self.tool_paths:
            self.plan_all_paths()
            
            # Execute if requested
            if self.execute_trajectories:
                self.execute_all_trajectories()
                
    def joint_state_callback(self, msg):
        """Callback for joint state updates."""
        self.current_joint_state = msg
            
    def init_moveit_interfaces(self):
        """Initialize MoveIt2 interfaces using direct service and action clients."""
        try:
            # Create action client for MoveGroup
            self.move_client = ActionClient(self, MoveGroup, '/move_action')
            if not self.move_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn("Move action server not available, continuing without execution capability")
            else:
                self.get_logger().info("Connected to MoveGroup action server")
            
            # Create service client for Cartesian path planning
            self.cartesian_path_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
            if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("Cartesian path service not available, continuing without planning capability")
            else:
                self.get_logger().info("Connected to Cartesian path planning service")
            
            # Create service client for IK
            self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
            if not self.ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("IK service not available, continuing without IK capability")
            else:
                self.get_logger().info("Connected to IK service")
            
            # Create publisher for trajectory execution
            self.trajectory_pub = self.create_publisher(
                JointTrajectory, 
                '/joint_trajectory_controller/joint_trajectory', 
                10
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt2 interfaces: {e}")
            self.get_logger().info("Continuing in visualization-only mode")
            
    def init_visualization(self):
        """Initialize visualization publishers."""
        self.path_marker_pub = self.create_publisher(MarkerArray, '/task_paths', 10)
        self.current_path_pub = self.create_publisher(Marker, '/current_path', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/tool_poses', 10)
        
    def load_from_file(self, file_path):
        """Load tool paths from a file (either robot G-code or YAML)."""
        # Expand tilde in file path
        expanded_path = os.path.expanduser(file_path)
        path = pathlib.Path(expanded_path)
        
        if not path.exists():
            self.get_logger().error(f"File not found: {file_path} (expanded to {expanded_path})")
            return
            
        self.get_logger().info(f"Loading paths from: {path}")
        
        if path.suffix.lower() == '.gcode' or path.name.endswith('.robot.gcode'):
            self.load_gcode(path)
        else:
            self.get_logger().error(f"Unsupported file type: {path.suffix}")
    
    
            
    def load_gcode(self, gcode_path):
        """Load tool paths from a robot G-code file."""
        self.get_logger().info(f"Parsing G-code file: {gcode_path}")
        
        try:
            # Read the file contents
            with open(gcode_path, 'r') as f:
                gcode_text = f.read()
                
            # Try to extract scale from comments
            import re
            scale_match = re.search(r'; Scale factor: ([\d.]+)', gcode_text)
            if scale_match:
                extracted_scale = float(scale_match.group(1))
                self.get_logger().info(f"Using scale from G-code: {extracted_scale}")
                self.scale = extracted_scale
                
            # Print the first few lines for debugging
            self.get_logger().info(f"G-code file first 5 lines:")
            for i, line in enumerate(gcode_text.splitlines()[:5]):
                self.get_logger().info(f"  Line {i}: {line}")
                
            # Parse G-code into paths
            import re
            g_code_re = re.compile(r'G([01])\s')
            x_re = re.compile(r'X([-\d.]+)')
            y_re = re.compile(r'Y([-\d.]+)')
            z_re = re.compile(r'Z([-\d.]+)')
            f_re = re.compile(r'F([-\d.]+)')
            
            current_path = []
            tool_paths = []
            current_feed = 1000.0  # Default feed rate (mm/min)
            x = y = z = 0.0
            
            for line in gcode_text.splitlines():
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
                f_match = f_re.search(line)
                
                # Update coordinates if present
                if x_match: x = float(x_match.group(1))
                if y_match: y = float(y_match.group(1))
                if z_match: z = float(z_match.group(1))
                if f_match: current_feed = float(f_match.group(1))
                
                # Create tool path point
                if g_code == '1':  # G1 = machining move
                    # Create a ToolPathPose
                    pose = ToolPathPose()
                    pose.pose = Pose()
                    pose.pose.position.x = x * self.scale
                    pose.pose.position.y = y * self.scale
                    pose.pose.position.z = z * self.scale
                    pose.pose.orientation.w = 1.0  # Default orientation
                    pose.feed = current_feed / 60.0 * self.scale  # mm/min to m/s
                    
                    current_path.append(pose)
                    
                elif g_code == '0' and current_path:  # G0 = rapid move, path separator
                    # Complete current path if it has points
                    if len(current_path) > 1:
                        tp = ToolPath()
                        tp.poses = current_path
                        tool_paths.append(tp)
                        self.get_logger().debug(f"Added path with {len(current_path)} points")
                        
                    # Start a new path
                    current_path = []
            
            # Add the last path if not empty
            if len(current_path) > 1:
                tp = ToolPath()
                tp.poses = current_path
                tool_paths.append(tp)
                
            # Store the parsed paths
            self.tool_paths = tool_paths
            self.get_logger().info(f"Loaded {len(tool_paths)} tool paths with {sum(len(p.poses) for p in tool_paths)} points")
            
            # Print a sample of the first path for debugging
            if tool_paths and tool_paths[0].poses:
                first_path = tool_paths[0]
                self.get_logger().info(f"First path has {len(first_path.poses)} points")
                for i, pose in enumerate(first_path.poses[:3]):  # First 3 points
                    pos = pose.pose.position
                    self.get_logger().info(f"  Point {i}: X={pos.x:.6f} Y={pos.y:.6f} Z={pos.z:.6f}")
            
        except Exception as e:
            self.get_logger().error(f"Error loading G-code: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            
    def plan_all_paths(self):
        """Plan trajectories for all tool paths."""
        if not self.tool_paths:
            self.get_logger().warn("No tool paths to plan")
            return
            
        self.get_logger().info(f"Planning trajectories for {len(self.tool_paths)} paths")
        
        # Skip planning in visualization-only mode
        if not hasattr(self, 'cartesian_path_client') or self.cartesian_path_client is None:
            self.get_logger().info("Planning service not available, skipping planning")
            return
            
        # Clear previous trajectories
        self.planned_trajectories = []
        
        # Try to find the best orientation that works for all paths
        best_orientation = self.find_best_orientation()
        if best_orientation:
            self.get_logger().info(f"Using best orientation: x={best_orientation.x:.4f}, y={best_orientation.y:.4f}, z={best_orientation.z:.4f}, w={best_orientation.w:.4f}")
        else:
            self.get_logger().warn("Could not find a good orientation - trying default orientations")
        
        # Plan each path
        for i, tool_path in enumerate(self.tool_paths):
            self.get_logger().info(f"Planning path {i+1}/{len(self.tool_paths)}")
            
            # Determine if this is a travel or machining path
            is_travel = False
            if i > 0:
                # Compare last point of previous path to first point of this path
                prev_z = self.tool_paths[i-1].poses[-1].pose.position.z
                curr_z = tool_path.poses[0].pose.position.z
                is_travel = abs(curr_z - prev_z) > Z_JUMP_THRESHOLD
                
            # Set velocity based on travel vs. machining
            velocity_scale = TRAVEL_VELOCITY if is_travel else MAX_VELOCITY
            
            # Plan the path using the best orientation if found
            trajectory = self.plan_tool_path(tool_path, velocity_scale, best_orientation)
            
            # If planning succeeded, add to the list
            if trajectory:
                self.planned_trajectories.append((trajectory, is_travel))
                self.get_logger().info(f"  ✓ Successfully planned {'travel' if is_travel else 'machining'} path")
            else:
                self.get_logger().error(f"  ✗ Failed to plan path {i+1}")
                
        self.get_logger().info(f"Planned {len(self.planned_trajectories)}/{len(self.tool_paths)} paths successfully")
    
    def find_best_orientation(self):
        """Try to find the best orientation for all paths by testing single points with IK."""
        if not self.tool_paths or not hasattr(self, 'ik_client') or self.ik_client is None:
            return None
            
        self.get_logger().info("Searching for optimal tool orientation...")
        
        # Get a sample of points from different paths
        sample_points = []
        for tp in self.tool_paths:
            if tp.poses:
                # Add first point
                sample_points.append(tp.poses[0].pose.position)
                # Add middle point if path long enough
                if len(tp.poses) > 2:
                    mid_idx = len(tp.poses) // 2
                    sample_points.append(tp.poses[mid_idx].pose.position)
        
        if not sample_points:
            return None
            
        # Test different orientations
        best_orientation = None
        best_success_rate = 0.0
        
        for orientation in DEFAULT_ORIENTATIONS:
            successes = 0
            self.get_logger().info(f"Testing orientation: x={orientation.x:.4f}, y={orientation.y:.4f}, z={orientation.z:.4f}, w={orientation.w:.4f}")
            
            for point in sample_points:
                # Create a pose with this orientation
                test_pose = Pose()
                test_pose.position = point
                test_pose.orientation = orientation
                
                # Check if IK solution exists
                if self.test_ik(test_pose):
                    successes += 1
            
            success_rate = successes / len(sample_points) if sample_points else 0
            self.get_logger().info(f"  Success rate: {success_rate:.2f} ({successes}/{len(sample_points)})")
            
            if success_rate > best_success_rate:
                best_success_rate = success_rate
                best_orientation = orientation
                
                # If found a perfect solution, break early
                if success_rate >= 1.0:
                    break
        
        if best_success_rate > 0:
            self.get_logger().info(f"Best orientation found with success rate: {best_success_rate:.2f}")
            return best_orientation
        
        return None
    
    def test_ik(self, pose):
        """Test if an IK solution exists for a pose."""
        from moveit_msgs.srv import GetPositionIK
        from moveit_msgs.msg import PositionIKRequest
        
        # Create request
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.planning_group
        request.ik_request.robot_state.joint_state = self.current_joint_state if self.current_joint_state else JointState()
        request.ik_request.pose_stamped.header.frame_id = self.robot_frame
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 10000000  # 10ms
        request.ik_request.avoid_collisions = True
        
        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response is None:
            return False
            
        # Check if solution found (MoveIt error code SUCCESS = 1)
        return response.error_code.val == 1
    
        
    def plan_tool_path(self, tool_path, velocity_scale, best_orientation=None):
        """Plan a trajectory for a single tool path using MoveIt2 services with improved planning."""
        if not tool_path.poses:
            return None
            
        # Extract waypoints
        waypoints = []
        
        # Use the best orientation if provided, otherwise use default
        orientation = normalize(best_orientation or DEFAULT_ORIENTATIONS[0])
        
        for pose in tool_path.poses:
            modified_pose = Pose()
            modified_pose.position = pose.pose.position
            modified_pose.orientation = orientation
            waypoints.append(modified_pose)
        
        # Print the first waypoint for debugging
        if waypoints:
            pos = waypoints[0].position
            ori = waypoints[0].orientation
            self.get_logger().info(f"Planning with first waypoint at: [{pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}], orientation: [{ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}, {ori.w:.4f}]")
        
        try:
            # Create robot state message from current joint state
            start_state = RobotState()
            if self.current_joint_state:
                start_state.joint_state = self.current_joint_state
            else:
                self.get_logger().warn("No joint state available - using default start state")
            
            # Create GetCartesianPath request
            request = GetCartesianPath.Request()
            request.header.stamp = self.get_clock().now().to_msg()
            request.header.frame_id = self.robot_frame
            request.start_state = start_state
            request.group_name = self.planning_group
            request.link_name = self.tcp_link
            request.waypoints = waypoints
            request.max_step = 0.01  # 1cm resolution
            request.jump_threshold = 5.0  # Allow some jumps in joint space
            request.avoid_collisions = True
            
            # Call service
            future = self.cartesian_path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response is None:
                self.get_logger().error("Failed to get response from cartesian path service")
                return None
                
            # Check if planning succeeded
            if response.fraction < 0.5:
                self.get_logger().warn(f"Cartesian planning only achieved {response.fraction:.2f} coverage")
                
                # Try with different orientations if the best one failed
                if response.fraction < 0.1 and best_orientation is None:
                    for alt_orientation in DEFAULT_ORIENTATIONS[1:]:  # Skip the first one we already tried
                        self.get_logger().info(f"Trying alternative orientation: x={alt_orientation.x:.4f}, y={alt_orientation.y:.4f}, z={alt_orientation.z:.4f}, w={alt_orientation.w:.4f}")
                        
                        # Create new waypoints with this orientation
                        alt_waypoints = []
                        for pose in tool_path.poses:
                            modified_pose = Pose()
                            modified_pose.position = pose.pose.position
                            modified_pose.orientation = alt_orientation
                            alt_waypoints.append(modified_pose)
                        
                        # Create new request with this orientation
                        alt_request = GetCartesianPath.Request()
                        alt_request.header = request.header
                        alt_request.start_state = request.start_state
                        alt_request.group_name = request.group_name
                        alt_request.link_name = request.link_name
                        alt_request.waypoints = alt_waypoints
                        alt_request.max_step = request.max_step
                        alt_request.jump_threshold = request.jump_threshold
                        alt_request.avoid_collisions = request.avoid_collisions
                        
                        # Call service again
                        alt_future = self.cartesian_path_client.call_async(alt_request)
                        rclpy.spin_until_future_complete(self, alt_future)
                        alt_response = alt_future.result()
                        
                        if alt_response and alt_response.fraction > response.fraction:
                            # If this orientation works better, use it
                            self.get_logger().info(f"Alternative orientation improved coverage to {alt_response.fraction:.2f}")
                            response = alt_response
                            # If good enough, break
                            if alt_response.fraction > 0.9:
                                break
                
                # If planning is still not successful enough, try with reduced points
                if response.fraction < 0.5:
                    self.get_logger().info("Attempting planning with reduced waypoints")
                    
                    # Subsample the waypoints (every Nth point)
                    subsample_factor = 5
                    subsampled_waypoints = waypoints[::subsample_factor]
                    
                    if len(subsampled_waypoints) >= 2:
                        # Create new request with subsampled waypoints
                        sub_request = GetCartesianPath.Request()
                        sub_request.header = request.header
                        sub_request.start_state = request.start_state
                        sub_request.group_name = request.group_name
                        sub_request.link_name = request.link_name
                        sub_request.waypoints = subsampled_waypoints
                        sub_request.max_step = 0.05  # Larger steps for simpler path
                        sub_request.jump_threshold = 10.0  # More permissive jumps
                        sub_request.avoid_collisions = True
                        
                        # Call service again
                        sub_future = self.cartesian_path_client.call_async(sub_request)
                        rclpy.spin_until_future_complete(self, sub_future)
                        sub_response = sub_future.result()
                        
                        if sub_response and sub_response.fraction > response.fraction:
                            self.get_logger().info(f"Subsampled planning improved coverage to {sub_response.fraction:.2f}")
                            response = sub_response
                
                # If still not good enough, try with just endpoints
                if response.fraction < 0.5 and len(waypoints) >= 2:
                    self.get_logger().info("Attempting planning with just endpoints")
                    
                    # Create waypoints with just the first and last point
                    endpoint_waypoints = [waypoints[0], waypoints[-1]]
                    
                    # Create new request
                    end_request = GetCartesianPath.Request()
                    end_request.header = request.header
                    end_request.start_state = request.start_state
                    end_request.group_name = request.group_name
                    end_request.link_name = request.link_name
                    end_request.waypoints = endpoint_waypoints
                    end_request.max_step = 0.1  # Even larger steps
                    end_request.jump_threshold = 20.0  # Very permissive
                    end_request.avoid_collisions = True
                    
                    # Call service again
                    end_future = self.cartesian_path_client.call_async(end_request)
                    rclpy.spin_until_future_complete(self, end_future)
                    end_response = end_future.result()
                    
                    if end_response and end_response.fraction > 0.9:
                        self.get_logger().info("Successfully planned endpoint-only trajectory")
                        response = end_response
                
                # If we still couldn't get a good plan, return None
                if response.fraction < 0.1:
                    return None
                
            # Get planned trajectory
            trajectory = response.solution
            
            # Scale the trajectory based on feed rates from tool path
            #if hasattr(trajectory, "joint_trajectory") and hasattr(trajectory.joint_trajectory, "points") and trajectory.joint_trajectory.points:
                #self.scale_trajectory_velocity(trajectory.joint_trajectory, tool_path, velocity_scale)
                
            return trajectory
            
        except Exception as e:
            self.get_logger().error(f"Error planning cartesian path: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
            
    def scale_trajectory_velocity(self, trajectory, tool_path, velocity_scale):
        def scaled_duration(orig: Duration, factor: float) -> Duration:
            tot_nsec = (orig.sec * 1_000_000_000 + orig.nanosec) * factor
            sec  = int(tot_nsec // 1_000_000_000)
            nsec = int(round(tot_nsec - sec * 1_000_000_000))
            if nsec == 1_000_000_000:    # clamp after rounding
                sec += 1
                nsec = 0
            return Duration(sec=sec, nanosec=nsec)
        """Scale trajectory timing, velocities, and accelerations."""
        if not trajectory.points or not tool_path.poses:
            return

        # average feed from tool‑path (m/s), fall back to velocity_scale
        feeds = [p.feed for p in tool_path.poses if p.feed > 0]
        avg_feed = sum(feeds) / len(feeds) if feeds else velocity_scale

        factor = velocity_scale / avg_feed if avg_feed > 0 else 1.0

        for pt in trajectory.points:
            pt.time_from_start = scaled_duration(pt.time_from_start, factor)
            if pt.velocities:
                pt.velocities     = [v * factor for v in pt.velocities]
            if pt.accelerations:
                pt.accelerations  = [a * factor for a in pt.accelerations]
            
    def execute_all_trajectories(self):
        """Execute all planned trajectories in sequence."""
        if not self.planned_trajectories:
            self.get_logger().warn("No trajectories to execute")
            return
            
        self.get_logger().info(f"Executing {len(self.planned_trajectories)} trajectories")
        
        for i, (trajectory, is_travel) in enumerate(self.planned_trajectories):
            self.get_logger().info(f"Executing {'travel' if is_travel else 'machining'} trajectory {i+1}/{len(self.planned_trajectories)}")
            
            # Visualize current path
            self.publish_current_path(i)
            
            # Execute the trajectory
            success = self.execute_trajectory(trajectory)
            
            if not success:
                self.get_logger().error(f"Failed to execute trajectory {i+1}")
                break
                
            # Add a short delay between trajectories for stability
            time.sleep(0.5)
            
        self.get_logger().info("Execution complete")
        
    
    def execute_trajectory(self, robot_traj):
        """Send the given RobotTrajectory to MoveIt's ExecuteTrajectory action."""
        # instantiate once
        if not hasattr(self, "_exec_client"):
            self._exec_client = ActionClient(self, ExecuteTrajectory,
                                            "/execute_trajectory")
            if not self._exec_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("ExecuteTrajectory action server not available")
                return False

        goal            = ExecuteTrajectory.Goal()
        goal.trajectory = robot_traj

        send_future = self._exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Trajectory execution goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val != 1:          # SUCCESS == 1
            self.get_logger().error(f"Execution failed (error {result.error_code.val})")
            return False

        return True
    
    
            
    def move_to_start(self):
        """Move the robot to a starting position."""
        try:
            # Get the first pose from the first path
            if not self.tool_paths or not self.tool_paths[0].poses:
                self.get_logger().warn("No start pose available")
                return False
                
            start_pose = self.tool_paths[0].poses[0].pose
            
            # Add some Z offset for safety
            approach_pose = Pose()
            approach_pose.position.x = start_pose.position.x
            approach_pose.position.y = start_pose.position.y
            approach_pose.position.z = start_pose.position.z + 0.1  # 10 cm above
            approach_pose.orientation = start_pose.orientation
            
            # Plan and move to approach pose using MoveGroup
            goal = MoveGroup.Goal()
            goal.request.group_name = self.planning_group
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = TRAVEL_VELOCITY
            goal.request.max_acceleration_scaling_factor = 0.5
            
            # Create a position constraint for the target pose
            constraint = Constraints()
            constraint.name = "target_pose"
            
            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = self.robot_frame
            pos_constraint.link_name = self.tcp_link
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Create a small sphere around the target point
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [0.01]  # 1cm radius
            
            pos_constraint.constraint_region.primitives.append(primitive)
            pos_constraint.constraint_region.primitive_poses.append(approach_pose)
            pos_constraint.weight = 1.0
            
            constraint.position_constraints.append(pos_constraint)
            
            # Orientation constraint
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.robot_frame
            orientation_constraint.orientation = approach_pose.orientation
            orientation_constraint.link_name = self.tcp_link
            orientation_constraint.absolute_x_axis_tolerance = 0.1
            orientation_constraint.absolute_y_axis_tolerance = 0.1
            orientation_constraint.absolute_z_axis_tolerance = 0.1
            orientation_constraint.weight = 1.0
            
            constraint.orientation_constraints.append(orientation_constraint)
            
            # Add constraint to request
            goal.request.goal_constraints.append(constraint)
            
            # Send goal and wait for result
            self.get_logger().info("Moving to approach position")
            send_goal_future = self.move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Approach move goal rejected")
                return False
                
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            
            result = get_result_future.result().result
            
            if result.error_code.val != 1:  # SUCCESS
                self.get_logger().error(f"Approach move failed with error code {result.error_code.val}")
                return False
                
            self.get_logger().info("Successfully moved to approach position")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error moving to start: {e}")
            return False
            
    def publish_visualization(self):
        """Publish visualization markers for all paths."""
        if not self.tool_paths:
            return
            
        # Publish all tool paths as marker arrays
        marker_array = MarkerArray()
        
        for i, tool_path in enumerate(self.tool_paths):
            if not tool_path.poses:
                continue
                
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "task_paths"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set line properties
            marker.scale.x = 0.001  # Line width (1mm)
            
            # Color based on path index
            h = i / max(1, len(self.tool_paths) - 1)
            r, g, b = self.hsv_to_rgb(h, 1.0, 1.0)
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0
            
            # Add points
            for tp_pose in tool_path.poses:
                marker.points.append(tp_pose.pose.position)
                
            marker_array.markers.append(marker)
            
        self.path_marker_pub.publish(marker_array)
        
        # Publish all poses as a PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for tool_path in self.tool_paths:
            for tp_pose in tool_path.poses:
                pose_array.poses.append(tp_pose.pose)
                
        self.poses_pub.publish(pose_array)
        
    def publish_current_path(self, path_index):
        """Highlight the current path being executed."""
        if not self.tool_paths or path_index >= len(self.tool_paths):
            return
            
        tool_path = self.tool_paths[path_index]
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set line properties
        marker.scale.x = 0.002  # Line width (2mm)
        
        # Bright color for current path
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points
        for tp_pose in tool_path.poses:
            marker.points.append(tp_pose.pose.position)
            
        self.current_path_pub.publish(marker)
        
    def hsv_to_rgb(self, h, s, v):
        """Convert HSV color to RGB."""
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
    """Main entry point."""
    # Initialize ROS
    rclpy.init(args=args)
    
    try:
        # Create and spin node
        node = NoetherTaskConstructor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        rclpy.shutdown()

if __name__ == '__main__':
    main()