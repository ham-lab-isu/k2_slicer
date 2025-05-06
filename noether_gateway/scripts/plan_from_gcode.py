#!/usr/bin/env python3

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


"""
Script to plan trajectories from G-code using Tesseract motion planner.
This script interfaces with the Noether gateway output to generate robot trajectories.
"""

import os
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from std_srvs.srv import Trigger
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory

class PlanFromGcodeNode(Node):
    def __init__(self, gcode_file=None, save_path=None, execute=False):
        super().__init__('plan_from_gcode_node')
        
        # Store parameters
        self.gcode_file = gcode_file
        self.save_path = save_path
        self.execute = execute
        
        # Set parameter on the planner node
        if self.gcode_file:
            self.declare_parameter('gcode_file', self.gcode_file)
        
        # Create service clients
        self.plan_client = self.create_client(Trigger, '/plan_from_gcode')
        
        # Create subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10)
        
        self.status_sub = self.create_subscription(
            String,
            '/planner_status',
            self.status_callback,
            10)
        
        # Create action client for trajectory execution
        if self.execute:
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/joint_trajectory_controller/follow_joint_trajectory')
        
        # Store latest data
        self.latest_trajectory = None
        self.latest_status = None
        
        self.get_logger().info('Plan from G-code node initialized')

    def wait_for_services(self, timeout_sec=5.0):
        """Wait for required services to be available"""
        if not self.plan_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error('Planning service not available')
            return False
        
        if self.execute and not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Trajectory action server not available')
            return False
        
        return True

    def plan_trajectory(self):
        """Call the planning service"""
        self.get_logger().info(f'Planning trajectory from G-code: {self.gcode_file}')
        
        # Set the G-code file parameter
        if self.gcode_file:
            self.set_parameters([rclpy.parameter.Parameter(
                'gcode_file', rclpy.parameter.Parameter.Type.STRING, self.gcode_file)])
        
        # Call the planning service
        req = Trigger.Request()
        future = self.plan_client.call_async(req)
        
        return future

    def trajectory_callback(self, msg):
        """Callback for joint trajectory messages"""
        self.get_logger().info(f'Received trajectory with {len(msg.points)} points')
        self.latest_trajectory = msg
        
        # Save trajectory if requested
        if self.save_path:
            self.save_trajectory(msg, self.save_path)
        
        # Execute trajectory if requested
        if self.execute:
            self.execute_trajectory(msg)

    def status_callback(self, msg):
        """Callback for status messages"""
        self.get_logger().info(f'Planner status: {msg.data}')
        self.latest_status = msg.data

    def save_trajectory(self, trajectory, save_path):
        """Save trajectory to a file"""
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            
            # Write trajectory to file (simple joint positions format)
            with open(save_path, 'w') as f:
                f.write(f'# Joint trajectory\n')
                f.write(f'# Joint names: {", ".join(trajectory.joint_names)}\n')
                f.write(f'# time_from_start, {", ".join(trajectory.joint_names)}\n')
                
                for point in trajectory.points:
                    time_from_start = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                    positions = [f'{p:.6f}' for p in point.positions]
                    f.write(f'{time_from_start:.6f}, {", ".join(positions)}\n')
            
            self.get_logger().info(f'Saved trajectory to {save_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {e}')
            return False

    def execute_trajectory(self, trajectory):
        """Execute trajectory using action client"""
        if not self.execute:
            return False
        
        self.get_logger().info('Executing trajectory')
        
        # Create action goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.execute_callback)
        
        return True

    def execute_callback(self, future):
        """Callback for trajectory execution result"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory execution rejected')
            return
        
        self.get_logger().info('Trajectory execution accepted')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Callback for trajectory execution result"""
        result = future.result().result
        self.get_logger().info(f'Trajectory execution completed with error code: {result.error_code}')

def main(args=None):
    # Parse arguments
    parser = argparse.ArgumentParser(description='Plan trajectories from G-code using Tesseract')
    parser.add_argument('gcode_file', help='Path to G-code file')
    parser.add_argument('--save', help='Path to save trajectory')
    parser.add_argument('--execute', action='store_true', help='Execute trajectory')
    parser.add_argument('--wait', action='store_true', help='Wait for trajectory completion')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    # Create node
    node = PlanFromGcodeNode(
        gcode_file=parsed_args.gcode_file,
        save_path=parsed_args.save,
        execute=parsed_args.execute
    )
    
    # Wait for services
    if not node.wait_for_services():
        node.get_logger().error('Required services not available')
        rclpy.shutdown()
        return 1
    
    # Plan trajectory
    future = node.plan_trajectory()
    
    # Spin until planning is complete
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node)
    
    # Check result
    if future.done():
        response = future.result()
        
        if response.success:
            node.get_logger().info(f'Planning successful: {response.message}')
        else:
            node.get_logger().error(f'Planning failed: {response.message}')
            rclpy.shutdown()
            return 1
    else:
        node.get_logger().error('Planning service call failed')
        rclpy.shutdown()
        return 1
    
    # Wait for trajectory if requested
    if parsed_args.wait:
        while rclpy.ok() and (node.latest_trajectory is None):
            rclpy.spin_once(node)
    
    # If not waiting for execution, we're done
    if not parsed_args.wait or not parsed_args.execute:
        rclpy.shutdown()
        return 0
    
    # Wait for execution to complete
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
