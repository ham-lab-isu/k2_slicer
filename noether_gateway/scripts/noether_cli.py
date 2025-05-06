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
Noether Gateway Command Line Interface
Provides a unified CLI for all operations in the OrcaSlicer to robot path planning workflow
"""

import argparse
import os
import subprocess
import sys
import time

def process_gcode(args):
    """Process G-code through the planning pipeline"""
    print(f"Processing G-code file: {args.file}")
    
    cmd = ['ros2', 'run', 'noether_gateway', 'orca_to_noether_enhanced', args.file]
    if args.dry_run:
        cmd.append('--dry-run')
    
    result = subprocess.run(cmd)
    
    if result.returncode != 0:
        print("Error processing G-code file.")
        return False
    
    print("G-code processing complete.")
    return True

def visualize(args):
    """Launch visualization for a processed G-code file"""
    print(f"Launching visualization for: {args.file}")
    
    # Use the dedicated visualization script
    cmd = ['ros2', 'run', 'noether_gateway', 'visualize_robot_gcode', args.file]
    
    subprocess.run(cmd)
    return True

def run_separate(args):
    """Run process and visualize separately (more reliable)"""
    print(f"Running pipeline with separate steps for: {args.file}")
    
    # Start the gateway service
    gateway_proc = subprocess.Popen(['ros2', 'run', 'noether_gateway', 'noether_gateway'])
    
    try:
        # Give the service time to start
        time.sleep(2)
        
        # Process the G-code file
        process_result = subprocess.run([
            'ros2', 'run', 'noether_gateway', 'orca_to_noether_enhanced', args.file
        ])
        
        if process_result.returncode != 0:
            print("Error processing G-code file.")
            return False
        
        print("G-code processing complete.")
        
        if not args.no_viz:
            # Launch visualization
            robot_gcode = args.file
            if not robot_gcode.endswith('.robot.gcode'):
                robot_gcode += '.robot.gcode'
            
            print(f"Launching visualization for: {robot_gcode}")
            subprocess.run([
                'ros2', 'run', 'noether_gateway', 'visualize_robot_gcode', robot_gcode
            ])
    finally:
        # Shut down the gateway service
        gateway_proc.terminate()
    
    return True

def run_pipeline(args):
    """Run the complete pipeline (process and visualize)"""
    print(f"Running complete pipeline for: {args.file}")
    
    cmd = [
        'ros2', 'launch', 'noether_gateway', 
        'complete_pipeline.launch.py', 
        f'gcode_file:={args.file}',
        f'visualize:={"true" if not args.no_viz else "false"}'
    ]
    
    result = subprocess.run(cmd)
    
    if result.returncode != 0:
        print("Error in launch file. Trying separate approach...")
        return run_separate(args)
    
    return True

def start_service(args):
    """Start the Noether gateway service"""
    print("Starting Noether gateway service...")
    
    cmd = ['ros2', 'run', 'noether_gateway', 'noether_gateway']
    
    # Start in a non-blocking way
    process = subprocess.Popen(cmd)
    
    # Wait for service to start
    time.sleep(2)
    
    if process.poll() is not None:
        print("Error starting gateway service.")
        return False
    
    print("Gateway service started.")
    print("Press Ctrl+C to stop...")
    
    try:
        process.wait()
    except KeyboardInterrupt:
        process.terminate()
        print("\nGateway service stopped.")
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Noether Gateway - OrcaSlicer to Robot Path Planning CLI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Process a G-code file
  noether_cli process ~/path/to/model.gcode
  
  # Visualize processed G-code
  noether_cli visualize ~/path/to/model.robot.gcode
  
  # Run the complete pipeline
  noether_cli run ~/path/to/model.gcode
  
  # Run the pipeline with separate steps (more reliable)
  noether_cli run-separate ~/path/to/model.gcode
  
  # Start the gateway service
  noether_cli service
"""
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Process command
    process_parser = subparsers.add_parser('process', help='Process G-code')
    process_parser.add_argument('file', help='Path to G-code file')
    process_parser.add_argument('--dry-run', action='store_true', help='Parse only, do not call service')
    
    # Visualize command
    visualize_parser = subparsers.add_parser('visualize', help='Visualize robot G-code')
    visualize_parser.add_argument('file', help='Path to robot G-code file')
    
    # Run command
    run_parser = subparsers.add_parser('run', help='Run complete pipeline')
    run_parser.add_argument('file', help='Path to G-code file')
    run_parser.add_argument('--no-viz', action='store_true', help='Skip visualization')
    
    # Run-separate command
    run_separate_parser = subparsers.add_parser('run-separate', help='Run pipeline with separate steps')
    run_separate_parser.add_argument('file', help='Path to G-code file')
    run_separate_parser.add_argument('--no-viz', action='store_true', help='Skip visualization')
    
    # Service command
    service_parser = subparsers.add_parser('service', help='Start gateway service')
    
    args = parser.parse_args()
    
    if args.command == 'process':
        process_gcode(args)
    elif args.command == 'visualize':
        visualize(args)
    elif args.command == 'run':
        run_pipeline(args)
    elif args.command == 'run-separate':
        run_separate(args)
    elif args.command == 'service':
        start_service(args)
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
