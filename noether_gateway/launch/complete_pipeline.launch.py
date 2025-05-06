#!/usr/bin/env python3
"""
complete_pipeline.launch.py — Orca-to-Noether full pipeline with 
optional visualization and MoveIt2 robot toolpath execution.
Tested on ROS 2 Humble.
"""
# — std / typing —————————————————————————————————————————————————————————
import os
from typing import List
# — launch API ———————————————————————————————————————————————————————————
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    pkg_name = "noether_gateway"
    rviz_cfg = os.path.join(
        get_package_share_directory(pkg_name), "config", "tool_paths.rviz"
    )
    
    # MoveIt2 resources
    moveit_pkg = "moveit_resources_panda_moveit_config"
    moveit_launch_file = os.path.join(
        get_package_share_directory(moveit_pkg), 
        "launch", 
        "demo.launch.py"
    )
    
    # — launch-file arguments ——————————————————————————————————————————
    declare_gcode = DeclareLaunchArgument(
        "gcode_file",
        description="Absolute path to OrcaSlicer *.gcode file",
    )
    declare_vis = DeclareLaunchArgument(
        "visualize", default_value="true", description="Enable RViz visualization"
    )
    declare_scale = DeclareLaunchArgument(
        "scale", default_value="0.001", description="Scale factor for unit conversion (default is mm to m)"
    )
    declare_use_robot = DeclareLaunchArgument(
        "use_robot", default_value="false", description="Enable MoveIt2 robot for toolpath execution"
    )
    
    gcode_file = LaunchConfiguration("gcode_file")
    visualize = LaunchConfiguration("visualize")
    scale = LaunchConfiguration("scale")
    use_robot = LaunchConfiguration("use_robot")
    
    # — core nodes / processes ———————————————————————————————————————
    gateway_node = Node(
        package=pkg_name,
        executable="noether_gateway",
        name="noether_gateway",
        output="screen",
    )
    
    bridge_proc = ExecuteProcess(
        name="orca_bridge",
        cmd=["ros2", "run", pkg_name, "orca_to_noether_enhanced", gcode_file, "--scale", scale],
        output="screen",
    )
    
    # Build <file>.robot.gcode (replace, don't append)
    robot_file = PythonExpression(
        ['"', gcode_file, '"', ".replace('.gcode', '.robot.gcode')"]
    )
    
    visualizer_proc = ExecuteProcess(
        name="tool_path_visualizer",
        cmd=["ros2", "run", pkg_name, "tool_path_visualizer", robot_file, "--scale", scale],
        output="screen",
        condition=IfCondition(visualize),
    )
    
    # MoveIt2 demo launch for Panda robot (only if use_robot is true)
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'false',  # We'll use our own RViz with custom config
        }.items(),
        condition=IfCondition(use_robot),
    )
    
    # Toolpath planner node (only if use_robot is true)
    toolpath_planner = Node(
        package="toolpath_planner",
        executable="toolpath_planner",
        name="toolpath_planner",
        output="screen",
        condition=IfCondition(use_robot),
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        condition=IfCondition(visualize),
    )
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        output="screen",
        condition=IfCondition(visualize),
    )
    
    # — defer visualization and robot toolpath execution until bridge finishes ———
    launch_after_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_proc,
            on_exit=[
                LogInfo(
                    msg=TextSubstitution(
                        text="G-code processing complete — launching visualization and robot tools..."
                    )
                ),
                visualizer_proc,
                rviz_node,
                static_tf,
                toolpath_planner,
            ],
        ),
    )
    
    return LaunchDescription(
        [
            declare_gcode,
            declare_vis,
            declare_scale,
            declare_use_robot,
            gateway_node,
            bridge_proc,
            # Start MoveIt2 immediately if robot execution is enabled
            moveit_demo,
            launch_after_bridge,
        ]
    )