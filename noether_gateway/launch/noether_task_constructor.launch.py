#!/usr/bin/env python3
"""
Launch file for the Noether MoveIt2 Task Constructor
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Get parameters
    input_file = LaunchConfiguration('input_file').perform(context)
    execute = LaunchConfiguration('execute').perform(context)
    visualize = LaunchConfiguration('visualize').perform(context)
    scale = LaunchConfiguration('scale').perform(context)
    
    # Task Constructor node - pass all parameters via ROS parameters instead of CLI arguments
    task_constructor_node = Node(
        package='noether_gateway',
        executable='noether_task_constructor',
        name='noether_task_constructor',
        parameters=[{
            'input_file': input_file,
            'execute': execute.lower() == 'true',
            'visualize': visualize.lower() == 'true',
            'scale': float(scale),
            'planning_group': 'khi_cx110l',  # Update with your robot's group name
            'tcp_link': 'khi_cx110l_link6',      # Update with your robot's TCP link
            'robot_frame': 'world'          # Update with your robot's reference frame
        }],
        output='screen'
    )
    
    # RViz node (if visualization is enabled)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('noether_gateway'),
        'config', 'task_commander.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(visualize),
        output='screen'
    )
    
    # Static TF node for map frame
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        condition=IfCondition(visualize),
        output='screen'
    )
    
    return [task_constructor_node, rviz_node, tf_node]

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'input_file',
            default_value='',
            description='Path to input file (.robot.gcode or .yaml)'
        ),
        
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Execute trajectories after planning'
        ),
        
        DeclareLaunchArgument(
            'visualize',
            default_value='true',
            description='Visualize paths and trajectories'
        ),
        
        DeclareLaunchArgument(
            'scale',
            default_value='0.001',
            description='Scale factor for unit conversion'
        ),
        
        # Setup and launch nodes
        OpaqueFunction(function=launch_setup)
    ])