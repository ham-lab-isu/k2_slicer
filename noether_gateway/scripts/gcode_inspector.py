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
limitations under the License.

G-code Inspector - analyzes and reports on the structure of a G-code file
'''
import sys
import re
import argparse

def inspect_gcode(filename):
    """Inspect and report on the structure of a G-code file"""
    print(f"Analyzing G-code file: {filename}")
    
    # Regular expressions
    g0_re = re.compile(r'G0\s')
    g1_re = re.compile(r'G1\s')
    x_re = re.compile(r'X([-\d.]+)')
    y_re = re.compile(r'Y([-\d.]+)')
    z_re = re.compile(r'Z([-\d.]+)')
    f_re = re.compile(r'F([-\d.]+)')
    
    # Counters
    total_lines = 0
    comment_lines = 0
    empty_lines = 0
    g0_count = 0
    g1_count = 0
    
    # Track paths
    current_path = []
    paths = []
    
    # Track coordinate values
    x_values = []
    y_values = []
    z_values = []
    
    try:
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                total_lines += 1
                line = line.strip()
                
                # Skip comments and empty lines
                if not line:
                    empty_lines += 1
                    continue
                
                if line.startswith(';'):
                    comment_lines += 1
                    continue
                
                # Check for G-code commands
                is_g0 = g0_re.search(line) is not None
                is_g1 = g1_re.search(line) is not None
                
                if is_g0:
                    g0_count += 1
                    # If we find a G0, it could be the start of a new path
                    if current_path:
                        paths.append(current_path)
                        current_path = []
                
                if is_g1:
                    g1_count += 1
                
                # Extract coordinates
                x_match = x_re.search(line)
                y_match = y_re.search(line)
                z_match = z_re.search(line)
                
                # Log any issue with parsing
                if (is_g0 or is_g1) and not (x_match or y_match or z_match):
                    print(f"Line {line_num}: G-code without coordinates: {line}")
                
                # Extract coordinates if present
                try:
                    if x_match:
                        x = float(x_match.group(1))
                        x_values.append(x)
                    if y_match:
                        y = float(y_match.group(1))
                        y_values.append(y)
                    if z_match:
                        z = float(z_match.group(1))
                        z_values.append(z)
                except ValueError as e:
                    print(f"Line {line_num}: Error parsing coordinate: {e} in line: {line}")
                
                # Add point to current path for G1 moves
                if is_g1 and x_match and y_match and z_match:
                    current_path.append((float(x_match.group(1)), 
                                         float(y_match.group(1)), 
                                         float(z_match.group(1))))
        
        # Add the last path
        if current_path:
            paths.append(current_path)
        
        # Calculate statistics
        path_lengths = [len(p) for p in paths]
        total_points = sum(path_lengths)
        
        # Print report
        print("\n=== G-code File Analysis ===")
        print(f"Total lines: {total_lines}")
        print(f"Comment lines: {comment_lines}")
        print(f"Empty lines: {empty_lines}")
        print(f"G0 commands (rapid moves): {g0_count}")
        print(f"G1 commands (linear moves): {g1_count}")
        print(f"Number of paths: {len(paths)}")
        print(f"Total points in paths: {total_points}")
        
        if path_lengths:
            print(f"Min path length: {min(path_lengths)}")
            print(f"Max path length: {max(path_lengths)}")
            print(f"Average path length: {sum(path_lengths) / len(paths):.1f}")
        
        if x_values:
            print(f"X-range: {min(x_values):.3f} to {max(x_values):.3f}")
        if y_values:
            print(f"Y-range: {min(y_values):.3f} to {max(y_values):.3f}")
        if z_values:
            print(f"Z-range: {min(z_values):.3f} to {max(z_values):.3f}")
        
        # Check for potential issues
        print("\n=== Potential Issues ===")
        if g0_count == 0:
            print("WARNING: No G0 rapid moves found. This may indicate an issue with the G-code format.")
        if g1_count == 0:
            print("WARNING: No G1 linear moves found. This may indicate an issue with the G-code format.")
        if len(paths) == 0:
            print("WARNING: No complete paths were found.")
        
        # Print sample of the first few lines of the file
        print("\n=== Sample Lines ===")
        with open(filename, 'r') as f:
            lines = f.readlines()
            for i in range(min(10, len(lines))):
                print(f"{i+1}: {lines[i].strip()}")
        
        return True
    except Exception as e:
        print(f"Error analyzing G-code file: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='G-code Inspector')
    parser.add_argument('file', help='Path to G-code file')
    args = parser.parse_args()
    
    inspect_gcode(args.file)

if __name__ == '__main__':
    main()
