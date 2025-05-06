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
Noether Task task_constructor CLI

Command-line interface for planning and executing robot motions with MoveIt2
Task task_constructor integration for Noether-generated tool paths.

This tool provides a unified interface to:
1. Load tool paths from G-code files
2. Plan trajectories for all tool paths
3. Execute trajectories sequentially
4. Visualize the planning and execution process
"""

import argparse
import os
import subprocess
import sys
import time
import pathlib

def check_file_exists(file_path):
    """Check if a file exists and return the full path."""
    path = pathlib.Path(file_path)
    if not path.exists():
        print(f"Error: File not found: {file_path}")
        return None
    return path.absolute()

def plan_paths(args):
    """Plan trajectories for tool paths without executing."""
    if not args.file:
        print("Error: Input file is required")
        return False

    file_path = check_file_exists(args.file)
    if not file_path:
        return False

    print(f"Planning trajectories for: {file_path}")
    
    cmd = [
        'ros2', 'launch', 'noether_gateway', 'noether_task_constructor.launch.py',
        f'input_file:={file_path}',
        'execute:=false',
        f'visualize:={str(not args.no_viz).lower()}',
        f'scale:={args.scale}'
    ]
    
    result = subprocess.run(cmd)
    return result.returncode == 0

def execute_paths(args):
    """Plan and execute trajectories for tool paths."""
    if not args.file:
        print("Error: Input file is required")
        return False

    file_path = check_file_exists(args.file)
    if not file_path:
        return False

    print(f"Planning and executing trajectories for: {file_path}")
    
    cmd = [
        'ros2', 'launch', 'noether_gateway', 'noether_task_constructor.launch.py',
        f'input_file:={file_path}',
        'execute:=true',
        f'visualize:={str(not args.no_viz).lower()}',
        f'scale:={args.scale}'
    ]
    
    result = subprocess.run(cmd)
    return result.returncode == 0

def visualize_paths(args):
    """Visualize tool paths without planning or executing."""
    if not args.file:
        print("Error: Input file is required")
        return False

    file_path = check_file_exists(args.file)
    if not file_path:
        return False

    print(f"Visualizing tool paths for: {file_path}")
    
    # Use the existing tool path visualizer
    visualizer_proc = subprocess.Popen([
        'ros2', 'run', 'noether_gateway', 'tool_path_visualizer', 
        str(file_path), '--scale', str(args.scale)
    ])
    
    # Start RViz2 with the configuration
    rviz_proc = subprocess.Popen([
        'ros2', 'run', 'rviz2', 'rviz2', '-d',
        os.path.join(
            os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0],
            'share/noether_gateway/config/task_constructor.rviz'
        )
    ])
    
    # Start static transform publisher
    tf_proc = subprocess.Popen([
        'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
        '0', '0', '0', '0', '0', '0', 'world', 'map'
    ])
    
    print("Visualization launched. Press Ctrl+C to exit...")
    try:
        visualizer_proc.wait()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
        for proc in [visualizer_proc, rviz_proc, tf_proc]:
            if proc.poll() is None:
                proc.terminate()
    
    return True

def process_gcode(args):
    """Process G-code through the Noether pipeline."""
    if not args.file:
        print("Error: Input file is required")
        return False

    file_path = check_file_exists(args.file)
    if not file_path:
        return False

    print(f"Processing G-code file: {file_path}")
    
    # Use the existing orca_to_noether_enhanced script
    cmd = ['ros2', 'run', 'noether_gateway', 'orca_to_noether_enhanced', str(file_path)]
    if args.min_dist:
        cmd.extend(['--min-dist', str(args.min_dist)])
    if args.scale:
        cmd.extend(['--scale', str(args.scale)])
    if args.dry_run:
        cmd.append('--dry-run')
    
    result = subprocess.run(cmd)
    
    if result.returncode != 0:
        print("Error processing G-code file.")
        return False
    
    print("G-code processing complete.")
    return True

def run_complete_pipeline(args):
    """Run the complete pipeline: process, plan, and optionally execute."""
    if not args.file:
        print("Error: Input file is required")
        return False

    file_path = check_file_exists(args.file)
    if not file_path:
        return False

    print(f"Running complete pipeline for: {file_path}")
    
    # 1. Process G-code
    if not process_gcode(args):
        print("Error in G-code processing step. Aborting pipeline.")
        return False
    
    # 2. Get the output .robot.gcode file path
    robot_gcode = file_path.with_suffix('.robot.gcode')
    if not robot_gcode.exists():
        print(f"Error: Robot G-code file not generated: {robot_gcode}")
        return False
    
    # 3. Plan and optionally execute
    if args.execute:
        return execute_paths(argparse.Namespace(
            file=str(robot_gcode),
            no_viz=args.no_viz,
            scale=args.scale
        ))
    else:
        return plan_paths(argparse.Namespace(
            file=str(robot_gcode),
            no_viz=args.no_viz,
            scale=args.scale
        ))

def main():
    """Main entry point for the CLI."""
    parser = argparse.ArgumentParser(
        description='Noether Task task_constructor CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process a G-code file
  noether_task_cli process ~/path/to/model.gcode
  
  # Plan trajectories for a robot G-code file
  noether_task_cli plan ~/path/to/model.robot.gcode
  
  # Execute trajectories for a robot G-code file
  noether_task_cli execute ~/path/to/model.robot.gcode
  
  # Visualize tool paths from a robot G-code file
  noether_task_cli visualize ~/path/to/model.robot.gcode
  
  # Run the complete pipeline
  noether_task_cli run ~/path/to/model.gcode
  
  # Run the complete pipeline and execute trajectories
  noether_task_cli run ~/path/to/model.gcode --execute
"""
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Process command
    process_parser = subparsers.add_parser('process', help='Process G-code')
    process_parser.add_argument('file', help='Path to G-code file')
    process_parser.add_argument('--min-dist', type=float, help='Minimum distance between points')
    process_parser.add_argument('--scale', type=float, default=0.001, help='Scale factor')
    process_parser.add_argument('--dry-run', action='store_true', help='Parse only, do not generate output')
    
    # Plan command
    plan_parser = subparsers.add_parser('plan', help='Plan trajectories')
    plan_parser.add_argument('file', help='Path to robot G-code file')
    plan_parser.add_argument('--no-viz', action='store_true', help='Disable visualization')
    plan_parser.add_argument('--scale', type=float, default=0.001, help='Scale factor')
    
    # Execute command
    execute_parser = subparsers.add_parser('execute', help='Execute trajectories')
    execute_parser.add_argument('file', help='Path to robot G-code file')
    execute_parser.add_argument('--no-viz', action='store_true', help='Disable visualization')
    execute_parser.add_argument('--scale', type=float, default=0.001, help='Scale factor')
    
    # Visualize command
    visualize_parser = subparsers.add_parser('visualize', help='Visualize tool paths')
    visualize_parser.add_argument('file', help='Path to robot G-code file')
    visualize_parser.add_argument('--scale', type=float, default=0.001, help='Scale factor')
    
    # Run command
    run_parser = subparsers.add_parser('run', help='Run complete pipeline')
    run_parser.add_argument('file', help='Path to G-code file')
    run_parser.add_argument('--execute', action='store_true', help='Execute trajectories')
    run_parser.add_argument('--no-viz', action='store_true', help='Disable visualization')
    run_parser.add_argument('--min-dist', type=float, help='Minimum distance between points')
    run_parser.add_argument('--scale', type=float, default=0.001, help='Scale factor')
    
    args = parser.parse_args()
    
    if args.command == 'process':
        process_gcode(args)
    elif args.command == 'plan':
        plan_paths(args)
    elif args.command == 'execute':
        execute_paths(args)
    elif args.command == 'visualize':
        visualize_paths(args)
    elif args.command == 'run':
        run_complete_pipeline(args)
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
