from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'noether_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.py'))),
        # Include scripts
        ('lib/' + package_name, 
         ['scripts/orca_to_noether_enhanced.py', 
          'scripts/tool_path_visualizer.py',
          'scripts/noether_cli.py',
          'scripts/visualize_robot_gcode.py',
          'scripts/gcode_inspector.py',
          'scripts/pose_array_to_traj.py',
          'scripts/noether_task_constructor.py',
          'scripts/noether_task_constructor_cli.py']),
        # Include RViz config
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wglockner',
    maintainer_email='walterg@iastate.edu',
    description='Noether Gateway and OrcaSlicer to Robot bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'noether_gateway = noether_gateway.gateway_node:main',
            'orca_to_noether_enhanced = scripts.orca_to_noether_enhanced:main',
            'tool_path_visualizer = scripts.tool_path_visualizer:main',
            'noether_cli = scripts.noether_cli:main',
            'visualize_robot_gcode = scripts.visualize_robot_gcode:main',
            'gcode_inspector = scripts.gcode_inspector:main',
            'pose_array_to_traj = scripts.pose_array_to_traj:main',
            'noether_task_constructor = scripts.noether_task_constructor:main',
            'noether_task_constructor_cli = scripts.noether_task_constructor_cli:main',
        ],
    },
)
