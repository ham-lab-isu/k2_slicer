from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'noether_gateway'
    
    # Get the path to the RViz configuration file
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    rviz_config_path = os.path.join(rviz_config_dir, 'tool_paths.rviz')
    
    # Launch arguments
    robot_gcode = LaunchConfiguration('robot_gcode')
    
    # Declare launch arguments
    declare_robot_gcode = DeclareLaunchArgument(
        'robot_gcode',
        default_value='',
        description='Path to the robot G-code file to visualize'
    )
    
    # Launch the visualizer node
    visualizer_node = Node(
        package=package_name,
        executable='tool_path_visualizer',
        name='tool_path_visualizer',
        output='screen',
        parameters=[],
        arguments=[robot_gcode]
    )
    
    # Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create a static transform publisher for the map frame
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    return LaunchDescription([
        declare_robot_gcode,
        static_tf_node,
        visualizer_node,
        rviz_node
    ])