#!/usr/bin/env python3
"""orca_to_noether_full.py – v1.3

* Restores `generate_robot_gcode()` inside this file so there is no missing
  import.  The helper writes `<input>.robot.gcode` in millimetres (as most
  robot post‑processors expect) while ROS traffic remains in metres.

CLI example:
    ros2 run noether_gateway orca_to_noether_full \
        $HOME/part.gcode --min-dist 0.2 --scale 0.001
"""

from __future__ import annotations
import argparse, logging, math, pathlib, re, sys, time
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, Pose, Point, Quaternion
from rclpy.node import Node
from noether_interfaces.srv import PlanSurface
from noether_interfaces.msg import ToolPath, ToolPathPose

# Default scaling factor (will be replaced by command-line argument)
DEFAULT_MM_TO_M = 0.001  # Convert mm to m

# ────────── logging ──────────

def setup_logging(level: str = "INFO") -> logging.Logger:
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO),
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    return logging.getLogger("orca_bridge")

# ────────── parsing ──────────
FLOAT = r"[-+]?[0-9]*\.?[0-9]+"
G_RE = re.compile(r"G0?[123]")
NUM_RE = re.compile(r"([XYZEFIJR])(" + FLOAT + ")")
LAYER_RE = re.compile(r";.*LAYER:?\s*(\d+)")

@dataclass
class PathPoint:
    x: float; y: float; z: float; feed: float; e: float; tag: str  # PRINT/TRAVEL/…

def parse_gcode(gcode: pathlib.Path, log: logging.Logger) -> List[PathPoint]:
    pts: List[PathPoint] = []
    state = dict(x=None, y=None, z=None, f=0.0, e=0.0)
    for line in gcode.read_text().splitlines():
        line_nocomment = line.partition(';')[0].strip()
        if not line_nocomment: continue
        if not G_RE.match(line_nocomment.split()[0]): continue
        params = {k: float(v) for k, v in NUM_RE.findall(line_nocomment)}
        for k, attr in (('X','x'),('Y','y'),('Z','z'),('F','f'),('E','e')):
            if k in params: state[attr] = params[k]
        if None in (state['x'], state['y'], state['z']): continue
        e_delta = params.get('E',0.0)
        tag = "PRINT" if abs(e_delta)>0 else "TRAVEL"
        pts.append(PathPoint(state['x'],state['y'],state['z'],state['f'],state['e'],tag))
    log.info("Parsed %d valid points", len(pts))
    return pts

# ────────── preprocessing (distance‑filter only) ──────────

def dist(a: PathPoint,b: PathPoint): return math.hypot(a.x-b.x,a.y-b.y)

def preprocess(pts: Sequence[PathPoint], min_dist: float, log: logging.Logger)->List[PathPoint]:
    if min_dist<=0: return list(pts)
    out=[pts[0]]
    for p in pts[1:]:
        if dist(p,out[-1])>=min_dist: out.append(p)
    log.info("Distance filter kept %d / %d", len(out), len(pts))
    return out

# ────────── Skip Noether completely and generate paths directly ──────────

def create_tool_paths_directly(pts: Sequence[PathPoint], scale: float, log: logging.Logger):
    """Create tool paths directly from original G-code points - skip Noether service"""
    log.info(f"Creating tool paths directly from {len(pts)} points with scale factor {scale}")
    
    # Group points into layers/paths based on Z values
    layers = {}
    current_z = None
    
    for p in pts:
        # Round Z to 2 decimals to group layers properly
        rounded_z = round(p.z, 2)
        if rounded_z not in layers:
            layers[rounded_z] = []
        layers[rounded_z].append(p)
    
    log.info(f"Found {len(layers)} unique Z layers")
    
    # Convert these layers to tool paths
    tool_paths = []
    
    for z, points in sorted(layers.items()):
        if len(points) < 2:  # Skip singleton points/layers
            continue
            
        # Create a new tool path for this layer
        tool_path = ToolPath()
        
        for point in points:
            pose = ToolPathPose()
            pose.pose = Pose()
            pose.pose.position = Point(
                x=point.x * scale,
                y=point.y * scale,
                z=point.z * scale
            )
            pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            pose.feed = point.feed * scale  # Convert feed rate
            
            tool_path.poses.append(pose)
        
        tool_paths.append(tool_path)
        log.info(f"Created tool path with {len(tool_path.poses)} poses at Z={z}")
    
    log.info(f"Created {len(tool_paths)} tool paths directly")
    return tool_paths

# ────────── G‑code writer ──────────

def generate_robot_gcode(tool_paths, outfile: pathlib.Path, scale: float, log: logging.Logger):
    log.info(f"Received {len(tool_paths)} tool paths to write with scale factor {scale}")
    
    # Log details about each tool path
    for i, tp in enumerate(tool_paths):
        log.info(f"Tool path {i}: {len(tp.poses)} poses")
    
    # Create the parent directory if it doesn't exist
    outfile.parent.mkdir(parents=True, exist_ok=True)
    
    with outfile.open('w') as g:
        g.write("; robot G‑code generated by Orca‑Noether bridge\n")
        g.write(f"; Scale factor: {scale} (mm to m)\n")
        g.write("G90\nG21\n\n")
        total=0
        for i,tp in enumerate(tool_paths):
            if not tp.poses: 
                log.info(f"Skipping tool path {i} - no poses")
                continue
            
            p0=tp.poses[0].pose.position
            log.info(f"Writing tool path {i} with {len(tp.poses)} poses. First point: X{p0.x/scale:.3f} Y{p0.y/scale:.3f} Z{p0.z/scale:.3f}")
            g.write(f"G0 X{p0.x/scale:.3f} Y{p0.y/scale:.3f} Z{p0.z/scale+5:.3f}\n")
            g.write(f"G0 Z{p0.z/scale:.3f}\n")
            
            for pose in tp.poses:
                pos=pose.pose.position; feed=pose.feed*60  # mm/min
                g.write(f"G1 X{pos.x/scale:.3f} Y{pos.y/scale:.3f} Z{pos.z/scale:.3f} F{feed:.1f}\n")
                total+=1
                
            p_last=tp.poses[-1].pose.position; g.write(f"G0 Z{p_last.z/scale+5:.3f}\n\n")
        g.write("G0 Z50\nM30\n")
        
    log.info(f"Wrote {total} poses to {outfile}")
    
    # Dump the first few lines of the file for debugging
    log.info(f"G-code file first 10 lines:")
    with outfile.open('r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines[:10]):
            log.info(f"Line {i}: {line.strip()}")

# ────────── CLI main ──────────

def main():
    ap=argparse.ArgumentParser(description="Orca→Noether (units fixed) v1.3")
    ap.add_argument('file',type=pathlib.Path)
    ap.add_argument('--scale',type=float,default=DEFAULT_MM_TO_M, 
                   help=f'Scale factor for unit conversion (default: {DEFAULT_MM_TO_M})')
    ap.add_argument('--min-dist',type=float,default=0.05, 
                   help='Minimum distance between points for filtering (default: 0.05)')
    ap.add_argument('--dry-run',action='store_true', 
                   help='Parse only, do not generate output')
    ap.add_argument('--log',default='INFO', 
                   help='Logging level (default: INFO)')
    args=ap.parse_args()

    log=setup_logging(args.log)
    log.info(f"Using scale factor: {args.scale} (input units to meters)")
    
    pts=parse_gcode(args.file,log)
    pts=preprocess(pts,args.min_dist,log)
    
    if args.dry_run: 
        return
        
    # Skip Noether service completely - create tool paths directly from G-code points
    tpaths = create_tool_paths_directly(pts, args.scale, log)
    generate_robot_gcode(tpaths, args.file.with_suffix('.robot.gcode'), args.scale, log)

if __name__=='__main__':
    main()